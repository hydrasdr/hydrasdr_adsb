# hydrasdr_adsb

Mode S / ADS-B 1090 MHz receiver for HydraSDR devices.<br>
It decodes the 12-bit HydraSDR I/Q stream at 2.5, 5 or 10 MS/s, live through libhydrasdr or from a recording.<br>
Each 0.5 us chip is integrated on a 0.1 us grid, so every sample counts, where a 2.4 MS/s receiver has 2 or 3 samples per microsecond.<br>
Outputs: AVR on stdout, Beast, SBS and AVR over TCP.<br>

## Install

The Linux packages need hydrasdr-host-tools 1.1.4 or later from the hydrasdr-host releases: it brings libhydrasdr and the udev rules.<br>
Install both packages in one command, for example:

    sudo apt install ./hydrasdr-host-tools-<distro>-v1.1.4.deb ./hydrasdr_adsb-<distro>-v<version>.deb
    sudo dnf install ./hydrasdr-host-tools-<distro>-v1.1.4.rpm ./hydrasdr_adsb-<distro>-v<version>.rpm
    sudo pacman -U ./hydrasdr-host-tools-Arch-Linux-v1.1.4.pkg.tar.zst ./hydrasdr_adsb-Arch-Linux-v<version>.pkg.tar.zst

The Windows zip and the macOS archive carry libhydrasdr.<br>

## Build

CMake 3.16 or later and a C99 compiler.<br>
Live input needs libhydrasdr 1.1.3 or later; without it, only recordings are read.<br>

Linux, macOS, Raspberry Pi, with libhydrasdr installed (found by pkg-config):

    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build

A libhydrasdr that is not installed, from a release zip or a source build (libhydrasdr.dylib on macOS):

    cmake -S . -B build -DHYDRASDR_INCLUDE_DIR=.../libhydrasdr/src \
        -DHYDRASDR_LIBRARY=.../libhydrasdr.so

On Windows without these two options, CMake 3.19 or later downloads the latest hydrasdr-host release from GitHub, checked by SHA-256.<br>
`-DHYDRASDR_DOWNLOAD=OFF` builds without network access.<br>

MSYS2 MinGW64:

    cmake -S . -B build_mingw64 -G Ninja
    cmake --build build_mingw64

Visual Studio 2026 (for Visual Studio 2022: "Visual Studio 17 2022"), and the Windows zip:

    cmake -S . -B build_vs2026 -G "Visual Studio 18 2026" -A x64
    cmake --build build_vs2026 --config Release
    cmake --build build_vs2026 --config Release --target package

The zip holds hydrasdr_adsb.exe, hydrasdr.dll, libusb-1.0.dll (the MinGW64 build of libusb 1.0.30, no Visual C++ runtime needed), README.md and the license texts.<br>
The RFOne needs no driver on Windows: it takes WinUSB automatically (WCID).<br>

## Usage

    hydrasdr_adsb [options]              receive from a HydraSDR
    hydrasdr_adsb --ifile FILE [options] read a recording

    --serial HEX      device serial number (default: first found)
    --rate HZ         2500000, 5000000 (default) or 10000000
    --linearity-gain N
                      linearity gain preset (default: the device default)
    --sensitivity-gain N
                      sensitivity gain preset, instead of linearity
    --bias-tee        power the antenna through the RF input
    --record FILE     write the ADC stream to FILE, a .raw12 name, with
                      FILE.sigmf-meta
    --duration SEC    stop after SEC seconds
    --ifile FILE      read a recording: SigMF, 2-channel WAV, or raw
    --iformat FMT     raw sample format: cs16, cs12, cf32, raw12
                      (default: from a .cs12 or .raw12 name, cs16
                      otherwise)
    --raw             print frames as *HEX; on stdout
    --mlat            with --raw: @TIMESTAMP before each frame
    --no-fix          no single-bit error correction
    --lat DEG         receiver latitude
    --lon DEG         receiver longitude
    --max-range KM    drop positions farther from the receiver
                      (default 450 with --lat/--lon, 0 = off)
    --net             serve Beast, SBS and AVR over TCP
    --net-bind-address ADDR  (default 127.0.0.1)
    --net-bo-port N   Beast output port (default 30005, 0 = off)
    --net-sbs-port N  SBS BaseStation port (default 30003)
    --net-ro-port N   AVR output port (default 30002)
    --throttle        read a recording at its real rate
    --help

- The gain presets map to each device's own gain stages, so a value works on any HydraSDR; the program prints the range at start.<br>
  An RFOne with an indoor FM antenna needs linearity 20 of 21.
- Use 5 MS/s on an RFOne: its 2.5 MS/s stream passes only +/-0.85 MHz, too narrow for the reply pulses.
- The bias tee is off unless `--bias-tee` is given, and off again at exit.
- The first RF port is selected, as hydrasdr_rx does, and 12-bit USB packing is used when the device has it.
- Decoding runs on its own thread.<br>
  Samples that do not fit its queue (0.4 s at 10 MS/s) are dropped and counted, and timestamps stay right.
- The `--mlat` timestamp is the start of the preamble in 12 MHz ticks from the first sample.

## Recordings

SigMF pairs give their own rate and format: from HydraSDR Studio (`x.cs16` and `x.cs16.sigmf-meta`) or standard (`x.sigmf-data` and `x.sigmf-meta`).<br>

`--record FILE.raw12` writes the ADC stream as the device sends it, nothing rounded, with FILE.raw12.sigmf-meta.<br>
The format is the HydraSDR Studio .raw12: 12-bit codes packed 8 in 12 bytes, at twice the I/Q rate.<br>
While recording, the live decode and a replay both convert it with src/raw12.c, a copy of the libhydrasdr 1.1.4 int16 DDC, so a replay gives the live frames byte for byte.<br>
The device must send packed 12-bit ADC codes, as the RFOne does.<br>
Samples lost on USB or dropped by a full queue are written as mid-scale codes and listed in the meta file; a replay skips them as the live decode did.<br>
A .raw12 file without a meta file takes its rate from a Studio name (`<freq>Hz_<rate>SPS_...`) or from `--rate`.<br>

.cs12 files hold I/Q rounded to 12 bits, so their replay can differ slightly from the live decode.<br>

## Network outputs

`--net` opens these ports on 127.0.0.1.<br>
`--net-bind-address 0.0.0.0` serves other computers, and a port set to 0 stays closed.<br>

    30005  Beast, with the 12 MHz timestamp and signal level
    30003  SBS BaseStation, as dump1090 writes it
    30002  AVR, one *HEX; line per frame

A client more than 256 KB behind is disconnected.<br>
`--throttle` replays a recording at its real rate, to try a client without a receiver.<br>

### readsb and tar1090

readsb takes the Beast port and writes the JSON for tar1090; its own ports stay off:

    readsb --net-only --net-bo-port 0 --net-sbs-port 0 --net-ro-port 0 \
        --net-connector 127.0.0.1,30005,beast_in --write-json DIR

### Virtual Radar Server 2.4

Run `hydrasdr_adsb --net`, then in Virtual Radar Server open Tools > Options..., Receivers, and add a receiver:

    Enabled          checked
    Format           BaseStation
    Connection type  Network
    Address          127.0.0.1
    Port             30003

After Save, the aircraft show on the "Local address" web page of Virtual Radar Server (checked with 2.4.4 on Windows 11).<br>
For Virtual Radar Server on another computer, run hydrasdr_adsb with `--net-bind-address 0.0.0.0` and enter the address of the HydraSDR computer (see Security).<br>

## Security

- The ports only send: what a client sends is read and thrown away, and no port takes a command.
- There is no authentication and no encryption: keep the default 127.0.0.1 unless the network is trusted.
- A port takes at most 16 clients; more are closed at once.
- Frames are untrusted input, since anyone with a transmitter can send a valid parity.<br>
  Table lookups and output lines are bounded, and the aircraft table stops at 4096 entries.

## Frame acceptance

- DF17/18: CRC residual 0, or one corrected bit outside the format field.
- DF11: residual 0, or an interrogator code in the low 7 bits (CL 0 to 4) for
  an address already seen.
- DF0/4/5/16/20/21: the residual is the address; accepted when that address
  was seen on a DF11 or DF17/18 frame in the last 60 s.

## License

MIT, see LICENSE. src/raw12.c and src/raw12.h come from libhydrasdr and keep
their own copyright notices.
