# Project Roadmap and TODOs
```
For this side project, we've elected to build and handle signal transfer across
machines from scratch using C++. This consists of having a central server on the
windows machine with a listening socket that's always looking for a signal on
a port we define. This will be the receiver server. This server would be the 
backend of the stack diagram we discussed last time. 

On any other machine, we would run a sender (yes, we only have two machines so
we'll run a sender adn a receiver on one but for the sake of project abstraction
imagine we have more than two). To send a given signal, we would need to:

a) have a device/modality or type handler (a separate client per device), something that checks that that device is connected
to the computer or is currently sending it signals or that signals of this type
are being currently sent to the computer. This device handler would then take
that signal, and use the central sender code, which is the same for all signals,
to send the data to the central server.

b) The device handlers/clients are different for different devices and data types. For example,
the Trigino EMG uses its own TCP-server based program to communicate with devices called TCU. The program
documentation is only available with the SDK kit which comes with the device and explains
how to build clients to communicate with the device. Since we don't have it yet,
we can look for an open source client that someone built for their own Trigino EMG device
and make a wrapper around it if it's in C++; but I have only found one in Python. We could
try to use it to figure out how TCU works and build our own C++ client from there (or just
wait until we get our hand on the SDK kit). The EEG has a C API; a compiled library 
plus a header file listing its functions, which can be called directly from C++ actually so
in that regard, it would be easier to figure it out and use the predefined 
commands to discover devices and gather signal (we will litearlly just make a wrapper for the
function calls we need). For midi I am pretty sure it's handled natively by the OS so we could just make a 
wrapper for it with the RtMidi C++ library. We'll need to also look into audio
and video for the facial recognition work type file handlers but I am pretty sure these would also
just be wrapper over an existing library situation. 

c) the central sender opens up a data stream for every modality, and homogenizes 
messages over the wire / network as such:

Length in the following is used as a delimiter so the central server knows
what begins and ends where since TCP just sends a continuous byte stream, so it's
just defining taht the following length of bytes means X to th central server.
First packet over Type A (EEG for example) stream: [length (of whole header)] [header]
                                                   [header] --> [u16 name length][name][u16 channels][f64 rate][u8 format]
                                                   example  --> EEG:   [23][10][UnicornEEG][17][250.0] [1]
                                                   length is byte size, format is just the format of the data sent as designated by 
                                                   manufacturer, rate is in Hz, and channels is how many values are being sent over the 
                                                   stream. For example, our EEG unicorn device does a scan over 17 values for every 
                                                   snapshot of a system state it gets, this includes the values recorded by the 8 electrode
                                                   channels, the battery level, etc.., so 17 values defining device data.

All other packets                                : [length] [body] 
                                                   [body]  --> [u32 seq][f64 t0][signal from device]
                                                   seq is this packet's number in the sequence of all
                                                   packets sent, this gets set per creation and it doesn't
                                                   depend on whether or not the packet arrived, which would help the central server catch
                                                   any missing data, t0 is just the time stamp, which will be used later for time
                                                   syncing between the different machines. Important Note because I have noticed
                                                   that this created a bit of confusion during our last meeting -- there are two
                                                   types of syncing we need to be doing here, rate syncing, which we can handle 
                                                   later, and packet / message syncing across the differnt machines, which aligns that this 
                                                   signal from this [ex EEG] stream was observed at the same time as that signal [ex midi] on the other machine.
                                                   So I am specifically talking about data exchange between machines here.
                                                   This is for the latter.

d) signal syncing across stream                  : This is what I have mentioned earler, we need to sync up signals sent over differnt machines,
                                                   and there's an existing software that already does this (although we've elected not to use it
                                                   and to build things from scratch due to some compatibility concerns), but it handles signal
                                                   syncing (not rate syncing) very well by using a protocal that defines an internal time stamp for
                                                   each sender or devices, queries the central server for its own central clock, then calculates its 
                                                   offset from the central server, and we use this offset as our guid when matching up signals over
                                                   data streams. I think we'll just do what they're doing idk it works.

                                                   
e) GUI                                          : Last bit, frontend and user friednly stuff. Build a graphical interface.


All TODOs for this side project:


Backend: 

Task                                         Level of Doneness

Build a central receiver server              Jintae built the skeleton, plumbing not 
                                             workable yet.(10/16 UPDATE done and tested by Jintae)

Build device / modality clients              Not Done, would be one file per signal type,
                                             some will be easier than others.

                                             EEG Client:
                                             10/19 UPDATE: Added elegant shutdown, testing, CLI port/serial,
                                             headset discovery, Counter/battery checks, and mock Unicorn
                                             library for testing without the headset to eeg_client,
                                             still missing serials. Also added unicorn.h, definitions for all
                                             functions we're using from API.

                                             Not yet tested, still needs uniforn.lib for windows machine. 
                                             

Build reusable sender                        Done, needs error handling, right now does
                                             not handle lost connections.
                                             
Handle machine (not rate) syncing            Laid ground-work, not done yet.

Add some sort of data saving / recording     Done by Jintae (10/16).
capabilities post receival


Frontend:

Build a user friendly interface that 
allows for stopping, starting, marking,      Not Done.
and playback of the signal streams.        








```
