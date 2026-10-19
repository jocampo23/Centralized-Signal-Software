CXX      := clang++
CXXFLAGS := -std=c++17 -Wall -Wextra -g -pthread

# Standalone asio (header-only), needed by central_sender.cpp.
# macOS: brew install asio   (headers land in /opt/homebrew/include)
ASIO_INC ?= /opt/homebrew/include

# unicorn.h from g.tec's C API (MIT licensed), see third_party/unicorn
UNICORN_INC ?= third_party/unicorn
# folder holding Unicorn.lib / Unicorn.dll (Windows only), for the real client
UNICORN_LIB ?= third_party/unicorn

INCLUDES := -I$(ASIO_INC) -I$(UNICORN_INC)

TARGETS := TCP_server eeg_client_mock

all: $(TARGETS)

TCP_server: TCP_server.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

# EEG client against the fake Unicorn library: no headset needed, builds on
# macOS/Linux. See the README for the simulation switches.
eeg_client_mock: eeg_client.cpp central_sender.cpp mock/unicorn_mock.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ eeg_client.cpp mock/unicorn_mock.cpp

# EEG client against the real headset. Windows only: g.tec ships the C API as
# Unicorn.lib + Unicorn.dll (64-bit). Put Unicorn.dll next to the .exe to run.
eeg_client: eeg_client.cpp central_sender.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -D_WIN32_WINNT=0x0A00 -o $@ eeg_client.cpp \
		-L$(UNICORN_LIB) -lUnicorn -lws2_32 -lmswsock

clean:
	rm -rf $(TARGETS) eeg_client eeg_client.exe *.dSYM

.PHONY: all clean
