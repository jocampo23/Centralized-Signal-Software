// Fake Unicorn C API so eeg_client can be built and run without the headset
// (and on a Mac, where g.tec ships no library). Implements every function in
// unicorn.h with the same signatures, so the client code doesn't change:
// link this file instead of Unicorn.lib.
//
// It pretends one headset, UN-MOCK.00.01, is paired, and streams 17 channels
// at 250 Hz in real time. It also enforces the API's command order (e.g.
// GetData before StartAcquisition, or GetChannelIndex during acquisition, fail
// with UNICORN_ERROR_OPERATION_NOT_ALLOWED) so ordering bugs show up here.
//
// Environment variables to simulate trouble:
//   UNICORN_MOCK_DROP_EVERY=N     skip one Counter value every N scans
//                                 (looks like scans lost over bluetooth)
//   UNICORN_MOCK_BATTERY=P        battery level in percent (default 100)
//   UNICORN_MOCK_FAIL_AFTER=S     GetData fails with a connection problem
//                                 S seconds into the first acquisition
//
// The real API's buffer size isn't documented; the mock overflows when the
// caller falls more than BUFFER_SCANS behind, which is a guess.
//
// How it fits together: unicorn.h declares the functions inside
// extern "C" { ... }, so the definitions below get plain C names, exactly
// like the ones exported by Unicorn.dll. The linker can't tell the
// difference, which is why eeg_client.cpp needs no #ifdefs for the mock.
//
// What it does NOT simulate: real EEG (the signals are clean sine/square
// waves), bluetooth timing jitter, SetConfiguration, or more than one headset.

#include "unicorn.h"

#include <chrono>    // steady_clock, for real-time pacing
#include <cmath>     // std::sin, std::fmod, M_PI
#include <cstdlib>   // std::getenv, std::atof
#include <cstring>   // std::strcmp, std::strncpy, std::memset
#include <string>    // std::string for the last error text
#include <thread>    // std::this_thread::sleep_until

// Everything in this anonymous namespace is private to this file: helpers and
// state the client should never see. Only the UNICORN_* functions further
// down are visible to the linker.
namespace {

// steady_clock never jumps (unlike the wall clock), so pacing stays smooth
// even if the system time changes while the mock is running
using Clock = std::chrono::steady_clock;

const char*    MOCK_SERIAL  = "UN-MOCK.00.01";        // the one "paired" headset (13 chars + '\0' fits in 14)
const UNICORN_HANDLE MOCK_HANDLE = 0x5EED;            // any non-zero value works; 0 means "no handle"
const uint32_t BUFFER_SCANS = 4 * UNICORN_SAMPLING_RATE;   // overflow after 4 s (1000 scans) unread

// Channel names in scan order, as GetChannelIndex reports them on the real
// headset. Positions match the *_CONFIG_INDEX constants in unicorn.h
// (EEG at 0, accelerometer at 8, gyroscope at 11, battery 14, counter 15,
// validation 16).

const char* CHANNEL_NAMES[UNICORN_TOTAL_CHANNELS_COUNT] = {
    "EEG 1", "EEG 2", "EEG 3", "EEG 4", "EEG 5", "EEG 6", "EEG 7", "EEG 8",
    "Accelerometer X", "Accelerometer Y", "Accelerometer Z",
    "Gyroscope X", "Gyroscope Y", "Gyroscope Z",
    "Battery Level", "Counter", "Validation Indicator",
};

// Everything the fake headset remembers between calls. One global instance,
// since the mock only ever has one headset.
struct MockState {
    bool open = false;          // between OpenDevice and CloseDevice
    bool acquiring = false;     // between StartAcquisition and StopAcquisition
    bool test_signal = false;   // what StartAcquisition was asked for
    bool failed_once = false;   // UNICORN_MOCK_FAIL_AFTER fires only once per run
    Clock::time_point start;    // when acquisition started; scan k is due at start + k/250 s
    uint64_t delivered = 0;     // scans handed out since StartAcquisition
    uint32_t counter = 0;       // value of the Counter channel
    std::string last_error = "No error.";   // returned by UNICORN_GetLastErrorText
} g;

// Reads a number from an environment variable, or `fallback` if it isn't set.
// Read on every use (not cached) so the switches are trivial to reason about.
double env_number(const char* name, double fallback) {
    const char* v = std::getenv(name);
    return v ? std::atof(v) : fallback;
}

// Every API function ends with `return fail(...)` or `return ok()`, so the
// last error text always matches the last return code, like the real API.
int fail(int code, const char* text) {
    g.last_error = text;
    return code;
}

int ok() {
    g.last_error = "No error.";
    return UNICORN_ERROR_SUCCESS;
}

// A handle is only good while the device is open and it's the one we gave
// out; a stale handle after CloseDevice gets UNICORN_ERROR_INVALID_HANDLE.
bool valid(UNICORN_HANDLE h) { return g.open && h == MOCK_HANDLE; }

// Fills one scan (17 floats) for time t (seconds since acquisition start),
// in the same channel order and units as the real headset.
void make_scan(float* scan, double t) {
    // channels 0-7: EEG, in microvolts
    for (int ch = 0; ch < UNICORN_EEG_CHANNELS_COUNT; ch++) {
        if (g.test_signal) {
            // square wave, like a calibration signal: +100 for 0.25 s, -100 for 0.25 s (2 Hz)
            scan[ch] = std::fmod(t, 0.5) < 0.25 ? 100.0f : -100.0f;
        } else {
            // a different alpha-ish sine per electrode (8, 9, ... 15 Hz) so
            // channels are easy to tell apart when plotted
            scan[ch] = static_cast<float>(20.0 * std::sin(2 * M_PI * (8.0 + ch) * t));
        }
    }
    
    // channels 8-10: accelerometer, as if the headset sits still and level
    scan[UNICORN_ACCELEROMETER_CONFIG_INDEX + 0] = 0.0f;
    scan[UNICORN_ACCELEROMETER_CONFIG_INDEX + 1] = 0.0f;
    scan[UNICORN_ACCELEROMETER_CONFIG_INDEX + 2] = 1.0f;   // gravity, in g
    // channels 11-13: gyroscope, not rotating
    for (int i = 0; i < UNICORN_GYROSCOPE_CHANNELS_COUNT; i++)
        scan[UNICORN_GYROSCOPE_CONFIG_INDEX + i] = 0.0f;
    // channel 14: battery percent, fixed (set UNICORN_MOCK_BATTERY to test warnings)
    scan[UNICORN_BATTERY_CONFIG_INDEX] = static_cast<float>(env_number("UNICORN_MOCK_BATTERY", 100));

    // channel 15: counter, +1 per scan. With UNICORN_MOCK_DROP_EVERY=N we
    // skip a value every N scans (299 -> 301), which is exactly what a scan
    // lost over bluetooth looks like to the client.
    const auto drop_every = static_cast<uint32_t>(env_number("UNICORN_MOCK_DROP_EVERY", 0));
    g.counter++;
    if (drop_every > 0 && g.counter % drop_every == 0) g.counter++;
    scan[UNICORN_COUNTER_CONFIG_INDEX] = static_cast<float>(g.counter);
    // channel 16: validation indicator, always "valid" here
    scan[UNICORN_VALIDATION_CONFIG_INDEX] = 1.0f;
}

}  // namespace

// =============================================================================
// The fake API. Same order as unicorn.h. Each function checks its arguments
// and the device state the way the real one does, then does the minimum
// needed to look real to the client.
// =============================================================================

// Real versions are like 1.18; 0 makes it obvious in the log that this is the mock.
UNICORN_API float UNICORN_GetApiVersion() { return 0.0f; }   // 0 = mock

UNICORN_API const char* UNICORN_GetLastErrorText() { return g.last_error.c_str(); }

// Always reports the recommended adapter with no problems, so the client's
// adapter warning stays quiet under the mock.
UNICORN_API int UNICORN_GetBluetoothAdapterInfo(UNICORN_BLUETOOTH_ADAPTER_INFO* info) {
    if (!info) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Invalid parameter.");
    std::memset(info, 0, sizeof(*info));
    std::strncpy(info->name, UNICORN_RECOMMENDED_BLUETOOTH_DEVICE_NAME, UNICORN_STRING_LENGTH_MAX - 1);
    std::strncpy(info->manufacturer, UNICORN_RECOMMENDED_BLUETOOTH_DEVICE_MANUFACTURER, UNICORN_STRING_LENGTH_MAX - 1);
    info->isRecommendedDevice = TRUE;
    info->hasProblem = FALSE;
    return ok();
}

// Supports the "call twice" pattern: devices == NULL only reports the count,
// otherwise *count says how big the caller's array is and we fill it in.
UNICORN_API int UNICORN_GetAvailableDevices(UNICORN_DEVICE_SERIAL* devices, uint32_t* count, BOOL onlyPaired) {
    if (!count) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Invalid parameter.");
    if (!onlyPaired) { *count = 0; return ok(); }   // the mock headset is already paired
    if (devices) {
        if (*count < 1) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Device array too small.");
        std::memset(devices[0], 0, UNICORN_SERIAL_LENGTH_MAX);   // zero first so the string is terminated
        std::strncpy(devices[0], MOCK_SERIAL, UNICORN_SERIAL_LENGTH_MAX - 1);
    }
    *count = 1;
    return ok();
}

// Only the mock serial opens; anything else fails like a headset that's off
// or out of range. Opening twice without closing fails too, which is how the
// tests confirm the client really calls CloseDevice before reconnecting.
UNICORN_API int UNICORN_OpenDevice(const char* serial, UNICORN_HANDLE* hDevice) {
    if (!serial || !hDevice) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Invalid parameter.");
    if (std::strcmp(serial, MOCK_SERIAL) != 0) return fail(UNICORN_ERROR_OPEN_DEVICE_FAILED, "Could not open device.");
    if (g.open) return fail(UNICORN_ERROR_OPEN_DEVICE_FAILED, "Device already open.");
    g.open = true;
    *hDevice = MOCK_HANDLE;
    return ok();
}

// Takes a pointer because the real API zeroes the caller's handle on close.
// Also ends acquisition, in case the caller skipped StopAcquisition.
UNICORN_API int UNICORN_CloseDevice(UNICORN_HANDLE* hDevice) {
    if (!hDevice || !valid(*hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    g.open = false;
    g.acquiring = false;
    *hDevice = 0;
    return ok();
}

// Starts the clock: from now on scans "arrive" at 250 Hz whether or not
// anyone reads them. The counter restarts, like on the real headset.
UNICORN_API int UNICORN_StartAcquisition(UNICORN_HANDLE hDevice, BOOL testSignalEnabled) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    if (g.acquiring) return fail(UNICORN_ERROR_OPERATION_NOT_ALLOWED, "Operation not allowed during acquisition.");
    g.acquiring = true;
    g.test_signal = testSignalEnabled != FALSE;
    g.start = Clock::now();
    g.delivered = 0;
    g.counter = 0;
    return ok();
}

// Stopping when not acquiring is an error on the real API too, so a client
// that calls Stop twice will see it here.
UNICORN_API int UNICORN_StopAcquisition(UNICORN_HANDLE hDevice) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    if (!g.acquiring) return fail(UNICORN_ERROR_OPERATION_NOT_ALLOWED, "Operation not allowed during non-acquisition.");
    g.acquiring = false;
    return ok();
}

// Not supported: always refuses (after the same state checks as the real
// call), so a client relying on custom configurations fails loudly here
// instead of silently getting the default 17 channels.
UNICORN_API int UNICORN_SetConfiguration(UNICORN_HANDLE hDevice, UNICORN_AMPLIFIER_CONFIGURATION* configuration) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    if (g.acquiring) return fail(UNICORN_ERROR_OPERATION_NOT_ALLOWED, "Operation not allowed during acquisition.");
    if (!configuration) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Invalid parameter.");
    return fail(UNICORN_ERROR_INVALID_CONFIGURATION, "The mock only supports the default configuration.");
}

// Reports the default configuration: all 17 channels on, with their names.
// Units and ranges are left empty; the client doesn't use them yet.
UNICORN_API int UNICORN_GetConfiguration(UNICORN_HANDLE hDevice, UNICORN_AMPLIFIER_CONFIGURATION* configuration) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    if (g.acquiring) return fail(UNICORN_ERROR_OPERATION_NOT_ALLOWED, "Operation not allowed during acquisition.");
    if (!configuration) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Invalid parameter.");
    std::memset(configuration, 0, sizeof(*configuration));
    for (int ch = 0; ch < UNICORN_TOTAL_CHANNELS_COUNT; ch++) {
        std::strncpy(configuration->Channels[ch].name, CHANNEL_NAMES[ch], 31);   // name is char[32]
        configuration->Channels[ch].enabled = TRUE;
    }
    return ok();
}

// The heart of the mock. Behaves like the real call:
//   - blocks until the requested scans would have arrived at 250 Hz
//   - fails with BUFFER_OVERFLOW if the caller fell too far behind
//   - checks the destination buffer is big enough (length is in floats)
// plus the optional simulated bluetooth drop (UNICORN_MOCK_FAIL_AFTER).
UNICORN_API int UNICORN_GetData(UNICORN_HANDLE hDevice, uint32_t numberOfScans, float* destinationBuffer, uint32_t destinationBufferLength) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    if (!g.acquiring) return fail(UNICORN_ERROR_OPERATION_NOT_ALLOWED, "Operation not allowed during non-acquisition.");
    if (numberOfScans == 0 || !destinationBuffer ||
        destinationBufferLength < numberOfScans * UNICORN_TOTAL_CHANNELS_COUNT)
        return fail(UNICORN_ERROR_INVALID_PARAMETER, "Destination buffer too small.");

    // seconds since StartAcquisition
    const double elapsed = std::chrono::duration<double>(Clock::now() - g.start).count();

    // simulated bluetooth drop: one failure, S seconds into the first
    // acquisition. The client should close, reopen, and carry on.
    const double fail_after = env_number("UNICORN_MOCK_FAIL_AFTER", 0);
    if (fail_after > 0 && !g.failed_once && elapsed >= fail_after) {
        g.failed_once = true;
        return fail(UNICORN_ERROR_CONNECTION_PROBLEM, "Connection problem (simulated by mock).");
    }

    // scans the "headset" has produced so far that nobody has read yet.
    // If the client stalled (e.g. stuck in a network connect) this grows
    // until it passes BUFFER_SCANS and we report an overflow.
    const double backlog = elapsed * UNICORN_SAMPLING_RATE - static_cast<double>(g.delivered);
    if (backlog > BUFFER_SCANS)
        return fail(UNICORN_ERROR_BUFFER_OVERFLOW, "Acquisition buffer overflow (simulated by mock).");

    // wait until the last requested scan would have arrived, like the real call.
    // Pacing is based on start + scan number (not "now + 4 ms"), so small
    // sleep inaccuracies don't add up into drift over a long run. If the
    // client is behind, `ready` is already in the past and this returns
    // immediately, letting it catch up.
    const auto ready = g.start + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(static_cast<double>(g.delivered + numberOfScans) / UNICORN_SAMPLING_RATE));
    std::this_thread::sleep_until(ready);

    // fill the caller's buffer scan after scan: [scan0 17 floats][scan1 17 floats]...
    for (uint32_t s = 0; s < numberOfScans; s++) {
        const double t = static_cast<double>(g.delivered) / UNICORN_SAMPLING_RATE;   // this scan's time
        make_scan(destinationBuffer + s * UNICORN_TOTAL_CHANNELS_COUNT, t);
        g.delivered++;
    }
    return ok();
}

// Looks a name up in CHANNEL_NAMES. Names are exact and case-sensitive
// ("Battery Level", not "battery"), same as the real API.
UNICORN_API int UNICORN_GetChannelIndex(UNICORN_HANDLE hDevice, const char* name, uint32_t* channelIndex) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    if (g.acquiring) return fail(UNICORN_ERROR_OPERATION_NOT_ALLOWED, "Operation not allowed during acquisition.");
    if (!name || !channelIndex) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Invalid parameter.");
    for (uint32_t ch = 0; ch < UNICORN_TOTAL_CHANNELS_COUNT; ch++) {
        if (std::strcmp(name, CHANNEL_NAMES[ch]) == 0) {
            *channelIndex = ch;
            return ok();
        }
    }
    return fail(UNICORN_ERROR_INVALID_PARAMETER, "Unknown channel name.");
}

// Always 17, since the mock can't turn channels off.
UNICORN_API int UNICORN_GetNumberOfAcquiredChannels(UNICORN_HANDLE hDevice, uint32_t* numberOfAcquiredChannels) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    if (g.acquiring) return fail(UNICORN_ERROR_OPERATION_NOT_ALLOWED, "Operation not allowed during acquisition.");
    if (!numberOfAcquiredChannels) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Invalid parameter.");
    *numberOfAcquiredChannels = UNICORN_TOTAL_CHANNELS_COUNT;
    return ok();
}

// Serial plus obviously fake version strings, so logs show it's the mock.
UNICORN_API int UNICORN_GetDeviceInformation(UNICORN_HANDLE hDevice, UNICORN_DEVICE_INFORMATION* deviceInformation) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    if (g.acquiring) return fail(UNICORN_ERROR_OPERATION_NOT_ALLOWED, "Operation not allowed during acquisition.");
    if (!deviceInformation) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Invalid parameter.");
    std::memset(deviceInformation, 0, sizeof(*deviceInformation));
    deviceInformation->numberOfEegChannels = UNICORN_EEG_CHANNELS_COUNT;
    std::strncpy(deviceInformation->serial, MOCK_SERIAL, UNICORN_SERIAL_LENGTH_MAX - 1);
    std::strncpy(deviceInformation->firmwareVersion, "mock", UNICORN_FIRMWARE_VERSION_LENGTH_MAX - 1);
    std::strncpy(deviceInformation->deviceVersion, "1.0", UNICORN_DEVICE_VERSION_LENGTH_MAX - 1);
    return ok();
}

// Digital outputs aren't used by the client; these just accept the call
// (outputs always read back as all low) so nothing breaks if someone tries.
UNICORN_API int UNICORN_SetDigitalOutputs(UNICORN_HANDLE hDevice, uint8_t) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    return ok();
}

UNICORN_API int UNICORN_GetDigitalOutputs(UNICORN_HANDLE hDevice, uint8_t* digitalOutputs) {
    if (!valid(hDevice)) return fail(UNICORN_ERROR_INVALID_HANDLE, "Invalid handle.");
    if (!digitalOutputs) return fail(UNICORN_ERROR_INVALID_PARAMETER, "Invalid parameter.");
    *digitalOutputs = 0;
    return ok();
}
