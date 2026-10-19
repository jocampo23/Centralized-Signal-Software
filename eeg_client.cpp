// eeg_client.cpp
// Device client for the g.tec Unicorn Hybrid Black EEG headset.
//
// What this program does, start to finish:
//   1. reads the command line (server address, which headset, packet size...)
//   2. finds the headset over bluetooth and opens it through g.tec's C API
//   3. starts acquisition: the headset streams 250 scans per second, where a
//      scan is one snapshot of all 17 values (8 EEG electrodes, 3 accelerometer,
//      3 gyroscope, battery, counter, validation)
//   4. reads those scans in blocks and hands each block to the central sender
//      (central_sender.cpp), which wraps it as [u32 length][u32 seq][f64 t0][floats]
//      and ships it to the central server over TCP
//   5. on Ctrl+C, stops acquisition and closes the headset cleanly
//   6. if the headset drops (bluetooth hiccup, battery dies, ...), closes it
//      and tries again every second until it comes back
//
// Build: `make eeg_client` on Windows with the real Unicorn.lib, or
// `make eeg_client_mock` anywhere to run against the fake headset in
// mock/unicorn_mock.cpp (see the README).

// central_sender.cpp holds ServerLink (the TCP connection to the central
// server) and now_seconds() (our monotonic clock). It's included directly
// rather than compiled separately; its #pragma once keeps it from being
// pulled in twice.
#include "central_sender.cpp"

// g.tec's header: declares every UNICORN_* function, the error codes, and the
// constants we use (UNICORN_SAMPLING_RATE, UNICORN_SERIAL_LENGTH_MAX, ...).
// The functions themselves live in Unicorn.dll (or in the mock).
#include "unicorn.h"

#include <atomic>      // std::atomic, for the Ctrl+C flag
#include <csignal>     // std::signal, SIGINT, SIGTERM
#include <cstdlib>     // std::atoi, std::exit, std::_Exit
#include <cstring>     // strnlen
#include <stdexcept>   // std::runtime_error

//different order the Unicorn API requires (from the g.tec C API guide):
//   GetAvailableDevices -> OpenDevice -> (queries/config) -> StartAcquisition
//   -> GetData loop -> StopAcquisition -> CloseDevice
// While acquiring, GetData is the ONLY call allowed, so every query (device
// info, channel indices, ...) has to happen between Open and Start.
// Breaking this order gets you UNICORN_ERROR_OPERATION_NOT_ALLOWED.

// The headset always samples at 250 Hz; this goes into the stream header
// so the server knows the rate, and is used to back-date block timestamps.

const double RATE = UNICORN_SAMPLING_RATE;   // 250 Hz

// =============================================================================
// Command line
// =============================================================================

// Everything that used to be hardcoded, now settable from the command line.
// The defaults below are what you get when a flag isn't passed.

struct Options {
    std::string host   = "127.0.0.1";   // central server IP; 127.0.0.1, currently set to my mac but we will change for windows
    std::string port   = "9125";        // must match the port TCP_server listens on
    std::string serial;                 // empty = use the one paired headset
    uint32_t block     = 1;             // scans per packet, change this to get different packets per second
                                        // (1 -> 250 packets/s, 25 -> 10 packets/s); bigger blocks mean
                                        // less network overhead but more latency per packet
    bool test_signal   = false;         // TRUE = headset sends g.tec's built-in test signal, not real EEG
                                        // (handy to check the whole pipeline before anyone wears the cap)
    bool list_only     = false;         // just print paired headsets and exit
};

// Printed for --help or when a flag is wrong. Goes to stderr so it never
// mixes with real output (e.g. the serial list from --list on stdout).

void usage(const char* prog) {
    std::cerr <<
        "usage: " << prog << " [options]\n"
        "  --host <ip>        central server address (default __________)\n" // change this for windows pc
        "  --port <port>      central server port (default 9125)\n"
        "  --serial <serial>  headset to open, e.g. UN-2021.05.36\n"
        "                     (default: the only paired headset)\n"
        "  --block <scans>    scans per packet (default 1 = 250 packets/s)\n"
        "  --test-signal      stream the headset's built-in test signal\n"
        "  --list             list paired headsets and exit\n";
}

// Walks argv (the words typed after the program name) and fills in Options.
// Flags that take a value (--host 1.2.3.4) consume the next word too.
// Exit code 2 = bad usage, 0 = --help.

Options parse_args(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; i++) {   // argv[0] is the program name, so start at 1
        const std::string a = argv[i];
        
        // grabs the value after a flag like --port 9125, and moves i past it.
        // If the flag is the last word there's no value, so that's a usage error.
        
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) { usage(argv[0]); std::exit(2); }
            return argv[++i];
        };
        if      (a == "--host")        opt.host = value();
        else if (a == "--port")        opt.port = value();
        else if (a == "--serial")      opt.serial = value();
        else if (a == "--block")       opt.block = static_cast<uint32_t>(std::atoi(value().c_str()));
        else if (a == "--test-signal") opt.test_signal = true;
        else if (a == "--list")        opt.list_only = true;
        
        // anything else: --help / -h prints usage and exits fine, an unknown
        // flag prints usage and exits with an error
            
        else { usage(argv[0]); std::exit(a == "--help" || a == "-h" ? 0 : 2); }
    }
    
    // atoi returns 0 for non-numbers, and GetData refuses 0 scans anyway
    if (opt.block == 0) { std::cerr << "--block must be at least 1\n"; std::exit(2); }
    return opt;
}

// =============================================================================
// Error handling
// =============================================================================

// Every UNICORN_* call returns an int: UNICORN_ERROR_SUCCESS (0) or an error
// code from unicorn.h. We turn failures into exceptions so the happy path
// reads top to bottom, and catch them in main() to retry.
//
// The exception carries the manufacturer's error text plus the error number,
// so we can look it up in unicorn.h. Common ones:
//   4  UNICORN_ERROR_OPEN_DEVICE_FAILED   headset off / out of range / wrong serial
//   6  UNICORN_ERROR_BUFFER_OVERFLOW      we didn't call GetData fast enough
//   8  UNICORN_ERROR_OPERATION_NOT_ALLOWED call made in the wrong order
//   9  UNICORN_ERROR_CONNECTION_PROBLEM   bluetooth link dropped

struct UnicornError : std::runtime_error {
    int code;   // kept so callers could react differently per error later
    UnicornError(int c, const std::string& what) : std::runtime_error(what), code(c) {}
};

// Just printing out manufacturer error in addition to the error number.
// Wrap any UNICORN_* call whose failure should abort the current session.

void check(int error) {
    if (error != UNICORN_ERROR_SUCCESS)
        throw UnicornError(error, std::string(UNICORN_GetLastErrorText()) + " (error " + std::to_string(error) + ")");
}

// =============================================================================
// Graceful shutdown
// =============================================================================
// Ctrl+C only sets a flag; the acquisition loop notices it within one GetData
// call (4 ms at --block 1), then stops acquisition and closes the device.
// A second Ctrl+C quits immediately, in case something is stuck (e.g. the
// sender waiting on a server that isn't answering).
//
// Why not just clean up inside the signal handler? A handler can interrupt
// the program anywhere, even halfway through a UNICORN_* call, so it may only
// do tiny "async-signal-safe" things like setting an atomic flag. The real
// cleanup happens back in normal code once the loop sees the flag.

// true once Ctrl+C (or a kill) arrives; read by every loop in the program
std::atomic<bool> g_stop{false};

// exchange() sets the flag and returns what it was before: false on the first
// Ctrl+C (so we just ask the loops to stop), true on the second (so we bail
// out right away). _Exit skips destructors, which is the point when stuck.
// 130 is the usual exit code for "killed by Ctrl+C".

extern "C" void on_signal(int) {
    if (g_stop.exchange(true)) std::_Exit(130);
}

// Route the "please stop" signals to on_signal instead of the default
// behavior, which kills the process on the spot without closing the headset.
void install_signal_handlers() {
    std::signal(SIGINT, on_signal);    // Ctrl+C
    std::signal(SIGTERM, on_signal);   // `kill <pid>`, or the OS asking us to quit
#ifdef SIGBREAK
    std::signal(SIGBREAK, on_signal);   // Ctrl+Break on Windows
#endif
}

// sleep that wakes up early if we're shutting down. A plain 1 s sleep would
// make Ctrl+C feel laggy during retries; checking every 50 ms doesn't.
void sleep_unless_stopping(double seconds) {
    const double until = now_seconds() + seconds;
    while (!g_stop && now_seconds() < until)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

// =============================================================================
// Device
// =============================================================================
// Owns the handle and always runs Stop -> Close, whether we leave because of
// Ctrl+C, a dropped Bluetooth link, or an exception.
//
// This is the RAII pattern: the destructor runs automatically when a Device
// goes out of scope, including while an exception is flying through, so the
// cleanup can't be forgotten on any exit path.

struct Device {
    UNICORN_HANDLE handle = 0;   // 0 = not open; OpenDevice fills in a real one
    bool acquiring = false;      // set after StartAcquisition succeeds

    // Safe to call more than once: each step only runs if it's still needed.
    // Failures here are printed, not thrown, because close() also runs from
    // the destructor, and throwing from a destructor ends the program.
    
    void close() {
        if (acquiring) {
            // tells the headset to stop streaming; it stays connected
            if (UNICORN_StopAcquisition(handle) != UNICORN_ERROR_SUCCESS)
                std::cerr << "Unicorn: stop acquisition failed: " << UNICORN_GetLastErrorText() << "\n";
            acquiring = false;
        }
        if (handle != 0) {
            // frees the bluetooth connection; CloseDevice also zeroes the handle
            if (UNICORN_CloseDevice(&handle) != UNICORN_ERROR_SUCCESS)
                std::cerr << "Unicorn: close device failed: " << UNICORN_GetLastErrorText() << "\n";
            handle = 0;
        }
    }
    ~Device() { close(); }
};

// Serials of headsets already paired with this computer. onlyPaired = TRUE is
// the quick lookup; FALSE would run a long blocking bluetooth scan.
//
// GetAvailableDevices is a "call twice" API: pass NULL first and it only
// tells you how many there are, so you know how much memory to allocate;
// call again with a buffer and it fills in the serials.

std::vector<std::string> paired_headsets() {
    uint32_t count = 0;
    check(UNICORN_GetAvailableDevices(nullptr, &count, TRUE));   // first call just counts
    if (count == 0) return {};

    // UNICORN_DEVICE_SERIAL is a fixed char[14] array, and std::vector can't
    // hold raw arrays, so we allocate plain bytes for `count` serials and
    // view them as an array of UNICORN_DEVICE_SERIAL.
    std::vector<char> raw(count * UNICORN_SERIAL_LENGTH_MAX);
    auto* serials = reinterpret_cast<UNICORN_DEVICE_SERIAL*>(raw.data());
    check(UNICORN_GetAvailableDevices(serials, &count, TRUE));

    // copy into std::strings; strnlen stops at the terminating '\0' but never
    // reads past the 14 bytes, even if the API didn't terminate the string
    std::vector<std::string> out;
    for (uint32_t i = 0; i < count; i++)
        out.emplace_back(serials[i], strnlen(serials[i], UNICORN_SERIAL_LENGTH_MAX));
    return out;
}

// Picks which headset to open. Returns "" if there's nothing to open yet
// (headset off / not paired), in which case we just retry.
//   --serial given     -> use it as-is, no lookup
//   exactly one paired -> use that one
//   none paired        -> "" (main loop retries in a second)
//   several paired     -> stop and ask the user to pick with --serial

std::string choose_serial(const Options& opt) {
    if (!opt.serial.empty()) return opt.serial;

    const auto found = paired_headsets();
    if (found.size() == 1) return found[0];
    if (found.empty()) {
        std::cerr << "Unicorn: no paired headset found, pair it in the OS bluetooth settings first\n";
        return "";
    }
    // more than one: we can't guess, the user has to pick
    std::cerr << "Unicorn: several paired headsets, choose one with --serial:\n";
    for (const auto& s : found) std::cerr << "  " << s << "\n";
    std::exit(2);
}

// Warns if the bluetooth adapter isn't the one shipped with the headset;
// other adapters are a common cause of dropped samples. Only a warning: the
// headset can still work with other adapters, just less reliably.

void check_bluetooth_adapter() {
    UNICORN_BLUETOOTH_ADAPTER_INFO info{};   // {} zero-fills the struct
    if (UNICORN_GetBluetoothAdapterInfo(&info) != UNICORN_ERROR_SUCCESS) return;   // can't tell, say nothing
    if (!info.isRecommendedDevice)
        std::cerr << "Unicorn: warning, bluetooth adapter '" << info.name
                  << "' is not the one shipped with the headset (" << UNICORN_RECOMMENDED_BLUETOOTH_DEVICE_NAME << ")\n";
    if (info.hasProblem)
        std::cerr << "Unicorn: warning, bluetooth adapter '" << info.name << "' reports a problem\n";
}

// Position of a named channel inside a scan, or -1 if it isn't acquired.
// With the default configuration every channel is on and the positions are
// fixed (Battery Level = 14, Counter = 15), but asking the API means this
// keeps working if someone later turns channels off with SetConfiguration.
int channel_index(UNICORN_HANDLE handle, const char* name) {
    uint32_t index = 0;
    if (UNICORN_GetChannelIndex(handle, name, &index) != UNICORN_ERROR_SUCCESS) return -1;
    return static_cast<int>(index);
}

// =============================================================================
// Data checks
// =============================================================================
// The headset numbers every scan in its "Counter" channel. A jump means scans
// were lost over bluetooth, which our packet seq can't see (seq only catches
// losses on the network side). Counter is a float, so it stays exact for
// 2^24 scans, about 18 hours at 250 Hz.
//
// So between the two we can tell where data went missing:
//   counter gap -> lost between the headset and this computer (bluetooth)
//   seq gap     -> lost between this computer and the server (network),
//                  detected on the server side
//
// It also watches the Battery Level channel (percent) and warns as it drops.

struct ScanChecks {
    int counter_ch = -1;   // position of "Counter" in a scan, -1 = not available
    int battery_ch = -1;   // position of "Battery Level" in a scan, -1 = not available

    bool have_counter = false;   // false until we've seen the first scan of a session
    uint32_t last_counter = 0;   // counter value of the previous scan
    uint64_t lost_scans = 0;     // running total across the whole run, for the status line

    float battery = -1;                // latest battery reading, -1 = none yet
    float next_battery_warning = 20;   // percent; warns at 20, 10, 0

    // Called at the start of every session. The headset restarts its counter
    // when acquisition restarts, so comparing against the old session's last
    // value would report a bogus gap. lost_scans is deliberately kept.
    void reset() {
        have_counter = false;
        next_battery_warning = 20;
    }

    // Looks at one block straight from GetData. The block is laid out scan
    // after scan: [scan0: ch0..ch16][scan1: ch0..ch16]..., so value `c` of
    // scan `s` sits at block[s * channels + c].
    void inspect(const float* block, uint32_t scans, uint32_t channels) {
        for (uint32_t s = 0; s < scans; s++) {
            const float* scan = block + s * channels;   // start of scan s

            if (counter_ch >= 0) {
                const auto counter = static_cast<uint32_t>(scan[counter_ch]);
                if (have_counter && counter <= last_counter) {
                    // shouldn't happen mid-session; report it rather than
                    // computing a huge negative "gap"
                    std::cerr << "Unicorn: counter went backwards (" << last_counter << " -> " << counter << ")\n";
                } else if (have_counter && counter != last_counter + 1) {
                    // e.g. 299 -> 301 means scan 300 never reached us
                    const uint32_t gap = counter - last_counter - 1;
                    lost_scans += gap;
                    std::cerr << "Unicorn: " << gap << " scan(s) lost over bluetooth (counter "
                              << last_counter << " -> " << counter << ")\n";
                }
                last_counter = counter;
                have_counter = true;
            }
        }

        // Battery changes slowly, so only the newest scan in the block matters.
        if (battery_ch >= 0) {
            battery = block[(scans - 1) * channels + battery_ch];
            if (battery < next_battery_warning) {
                std::cerr << "Unicorn: battery low, " << battery << "%\n";
                // move the threshold below the current level so we warn once
                // per 10% step instead of on every block (250 times a second)
                while (battery < next_battery_warning) next_battery_warning -= 10;
            }
        }
    }
};

// =============================================================================
// Main loop
// =============================================================================
// One connect/acquire session. Returns normally on Ctrl+C, throws if the
// headset drops so main() can retry.
//
// `link` and `checks` belong to main() and outlive each session on purpose:
// the server connection and the packet seq counter carry on across headset
// reconnects, so the server sees one continuous stream.

void run_session(const Options& opt, ServerLink& link, ScanChecks& checks) {
    const std::string serial = choose_serial(opt);
    if (serial.empty()) return;   // nothing paired yet; main() waits and retries

    // Declared before anything can throw, so its destructor always runs
    // Stop -> Close no matter where we leave this function.
    Device dev;
    std::cerr << "Unicorn: opening " << serial << "\n";
    // Open paired device (connects over bluetooth, can take a few seconds)
    check(UNICORN_OpenDevice(serial.c_str(), &dev.handle));

    // Nice-to-have info for the log. Not wrapped in check(): if it fails we
    // can still stream, so it's not worth aborting the session over.
    UNICORN_DEVICE_INFORMATION info{};
    if (UNICORN_GetDeviceInformation(dev.handle, &info) == UNICORN_ERROR_SUCCESS)
        std::cerr << "Unicorn: connected to " << info.serial << ", firmware " << info.firmwareVersion
                  << ", device version " << info.deviceVersion << "\n";

    // Get number of values per scan (channels)
    uint32_t channels = 0;   // 17: 8 EEG, 3 accel, 3 gyro, battery, counter, validation
    check(UNICORN_GetNumberOfAcquiredChannels(dev.handle, &channels));
    // tell the server what the headset actually reports. The header packet
    // carrying this is only written when the sender (re)connects, which
    // happens on the first send() below, so it always has the real value.
    link.channels = static_cast<uint16_t>(channels);

    // look these up now, GetChannelIndex isn't allowed once acquisition starts
    checks.counter_ch = channel_index(dev.handle, "Counter");
    checks.battery_ch = channel_index(dev.handle, "Battery Level");
    checks.reset();

    // Turns the EEG headset on: from here it streams 250 scans/s whether we
    // read them or not, so the loop below has to keep up.
    // FALSE = real signal, TRUE = g.tec's test signal (--test-signal)
    check(UNICORN_StartAcquisition(dev.handle, opt.test_signal ? TRUE : FALSE));
    dev.acquiring = true;   // only now does close() need to call StopAcquisition
    std::cerr << "Unicorn: acquiring " << channels << " channels at " << RATE << " Hz"
              << (opt.test_signal ? " (TEST SIGNAL)" : "") << ", sending to "
              << opt.host << ":" << opt.port << "\n";

    // one block of data to be sent over the wire (number of scans x channels per scan)
    // where a scan is one singular snapshot of all values across the EEG at a moment in time
    std::vector<float> block(opt.block * channels);
    double next_status = now_seconds() + 5;   // when to print the next status line

    while (!g_stop) {
        // blocks until opt.block scans have arrived; has to keep up with 250 Hz
        // or the API's buffer overflows (UNICORN_ERROR_BUFFER_OVERFLOW).
        // The last argument is the buffer size in floats (not bytes), which
        // the API uses to make sure it won't write past the end.
        check(UNICORN_GetData(dev.handle, opt.block, block.data(), static_cast<uint32_t>(block.size())));

        // timestamp of the first scan in the block. We only know when the
        // block finished arriving (now), so back-date by the scans before
        // the last one: with --block 25 the first scan is 24/250 s older.
        const double t0 = now_seconds() - (opt.block - 1) / RATE;

        checks.inspect(block.data(), opt.block, channels);
        // hand the raw floats to the central sender; it adds seq + t0, the
        // length prefix, and (re)connects to the server if needed
        link.send(t0, block.data(), block.size() * sizeof(float));

        // every 5 s, one line so you can see it's alive without flooding the
        // console (printing 250 lines a second would itself slow the loop)
        if (now_seconds() >= next_status) {
            std::cerr << "Unicorn: " << link.seq << " packets, " << checks.lost_scans << " scans lost";
            if (checks.battery >= 0) std::cerr << ", battery " << checks.battery << "%";
            std::cerr << (link.sock.is_open() ? "" : ", server not connected") << "\n";
            next_status += 5;
        }
    }
    // dev's destructor runs Stop -> Close
}

int main(int argc, char** argv) {
    const Options opt = parse_args(argc, argv);
    install_signal_handlers();

    // handy when comparing behavior across machines; the mock reports 0
    std::cerr << "Unicorn C API " << UNICORN_GetApiVersion() << "\n";

    // --list: print serials to stdout (one per line, easy to copy into
    // --serial) and exit without opening anything
    if (opt.list_only) {
        const auto found = paired_headsets();
        if (found.empty()) std::cout << "no paired Unicorn headsets\n";
        for (const auto& s : found) std::cout << s << "\n";
        return 0;
    }

    check_bluetooth_adapter();

    // Our connection to the central server. Nothing goes over the network
    // yet: ServerLink connects lazily on the first send(). The channel count
    // here is a placeholder that run_session() overwrites with the real one.
    ServerLink link{opt.host, opt.port, "UnicornEEG", UNICORN_TOTAL_CHANNELS_COUNT, RATE, FORMAT_FLOAT32};
    ScanChecks checks;

    // Keep (re)running sessions until Ctrl+C. Each pass opens the headset,
    // streams until something breaks, and cleans up (Device's destructor)
    // before we get back here.
    while (!g_stop) {
        try {
            run_session(opt, link, checks);
        } catch (const std::exception& e) {
            //printing to the error stream and retrying upon error
            std::cerr << "Unicorn: " << e.what() << ", retrying\n";
        }
        // time buffer between retry attempts, so a missing headset doesn't
        // turn into a tight loop hammering bluetooth
        sleep_unless_stopping(1.0);
    }

    // final summary; seq counts every block created, sent or not
    std::cerr << "Unicorn: stopped after " << link.seq << " packets, "
              << checks.lost_scans << " scans lost over bluetooth\n";
    return 0;
}
