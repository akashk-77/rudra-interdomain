# rudra-interdomain

Software for an autonomous soil-sampling rover, built by Team-A.

# rudra-interdomain

Software for an autonomous soil-sampling rover, built by Team Rudra.

The rover collects soil samples, runs chemistry tests on them, and sends sensor readings back to a base station. This repo holds the code for the telemetry link between the rover and a laptop, and for the camera-based soil tests.

A heads-up before you dive in: this is a work in progress. The telemetry pipeline runs in simulation, but a lot of the real hardware (radio, calibration, vision integration) isn't hooked up yet.
MPU - Raspberry Pi 4B
MCU - STM32 Nucleo

---

## What's in here

```
rudra-interdomain/
├── mpu/                    Everything that runs on the rover's main processor
│   ├── communication/      Sensor filtering, telemetry packets, link queue, UDP sender
│   ├── detection/          OpenCV tests for the soil chemistry (peroxide, iodine)
│   ├── sensors/            (empty for now)
│   └── system/             (empty for now)
├── server/                 Base-station side: packet decoder and UDP listener
├── tests/                  Copies of the telemetry test and sample packet generator
├── lib/                    Third-party Arduino libraries for the sensors
└── scripts/                (empty for now)
```

---

## How it works

### Telemetry: rover to base station

`mpu/communication/` is the path a sensor reading takes on its way to the laptop:

1. **`sensor_filter`** validates each reading (range checks, rate limits, outlier gating, stuck-sensor and stale-data detection) and runs a 1-D Kalman filter. It never reports missing data as zero. It reports an explicit quality: `UNKNOWN`, `VALID`, `STALE` or `INVALID`.
2. **`telemetry_packet`** packs the filtered values into a small binary packet, kept tiny on purpose since long-range radios only carry a few dozen bytes. The format is documented at the top of `telemetry_packet.h`.
3. **`link_queue`** has two halves. On the rover, a FIFO holds packets during an outage and drops the oldest first when full. On the base station, a tracker spots duplicates, late packets and gaps in the sequence numbers.
4. **`udp_sender`** is a demo that fakes three sensors (moisture, temperature, pH), pushes them through the whole pipeline, and sends them over UDP.

On the laptop, `server/decode_packet.py` decodes the packets (standard library only) and `server/udp_listener.py` listens for them and reports lost or duplicate packets.

The radio hasn't been chosen yet, so UDP stands in for it for now.

### Soil detection (OpenCV)

`mpu/detection/` has two standalone C++ programs that look at a photo of a soil sample in a clear test tube:

- **`h202_test/bubble_detect.cpp`** counts bubbles for the peroxide test. It ignores printed text and scale ticks on the tube, finds round blobs at several sizes, and checks each one has a closed boundary before counting it.
- **`iodine_test/iodine_test.cpp`** looks for the iodine/starch colour change. Violet or blue means positive, yellow or amber means negative. It can use a white reference patch to cancel out lighting.

Both have an options list in the header comment, including `--roi`, `--json` and `--select`.

---

## Simulation

### Physics simulation

We've set the rover up in two simulators: Gazebo, which uses DART as its physics engine, and Webots, which uses ODE. Webots turned out to be the better fit for us. It's more predictable, and physics and gravity act on objects more uniformly than they did in Gazebo.

### Communication simulation

For the network side we use CORE as our network simulator. The final readings are sent from the MPU to the base station over TCP and UDP servers.

---

## Getting started

You'll need a C compiler (`gcc`) and Python 3. OpenCV 4 (with `g++` and `pkg-config`) is only needed for the detection tools.

### Build

Everything is built with plain gcc. The telemetry code for the rover lives in `mpu/communication`:

```bash
cd mpu/communication

# telemetry tests
gcc -Wall -o test_telemetry test_telemetry.c telemetry_packet.c sensor_filter.c link_queue.c -lm

# UDP sender (Linux)
gcc -o udp_sender udp_sender.c telemetry_packet.c sensor_filter.c link_queue.c -lm
```

The detection tools need OpenCV 4:

```bash
cd mpu/detection/h202_test
g++ -O2 -std=c++17 bubble_detect.cpp -o bubble_detect $(pkg-config --cflags --libs opencv4)

cd ../iodine_test
g++ -O2 -std=c++17 iodine_test.cpp -o iodine_test $(pkg-config --cflags --libs opencv4)
```

### Run the telemetry tests

```bash
cd mpu/communication
./test_telemetry
```

These cover the filter, the packet format, the queue and an end-to-end sensor-to-base-station path.

### Try the telemetry link (rover side → laptop)

On the laptop:

```bash
cd server
python3 udp_listener.py          # listens on UDP port 5005 by default
```

On the Pi (or the same machine, using `127.0.0.1`):

```bash
cd mpu/communication
./udp_sender <laptop-ip> [port]
```

### Use the detection tools

From each tool's folder:

```bash
cd mpu/detection/h202_test
./bubble_detect image.jpeg --roi x,y,w,h

cd ../iodine_test
./iodine_test image.png --roi x,y,w,h
```

The sample images in each folder are good for a first try.

---

## Where things stand

**Working in simulation:**
- Sensor filter, packet format, link queue, and the UDP pipeline

**Not done yet:**
- A radio link over Wifi
- Sensor calibration (the ranges and noise values in the filter config are placeholders)
- Real sensor reads feeding the telemetry sender (it uses fake data for now)
- The `sensors/`, `system/` and `scripts/` folders are empty

**To do:**
the telemetry files exist in more than one place (`server/`, `tests/` and `mpu/communication/`). The copies of the packet code in `server/` and `mpu/communication/` are identical, so if one changes, the other needs to as well. Consolidating them is on the to-do list. All the sensor readings are fake simulated values, the real values will be plugged in once the sensors and the MCU are present physically and tested. UART communication between the MCU and MPU needs to be established once the MCU is present physically.

**Known Limitations:**
 - Camera angle, height , and proper pictures of the test tubes with specific lighting and region of interest have not been provided, so the OpenCV detection for the chemical test is expected to have inaccuracies.
 - Range of the communications between MPU and base station has not been tested due to unavailability of power source.

---

## Third-party libraries

`lib/` contains Arduino libraries for the sensor hardware, kept under their own licences:

- Adafruit TCS34725 (colour sensor)
- Adafruit AS7343 (spectral sensor)
- Adafruit BusIO
- Adafruit PWM Servo Driver
- OneWire and Arduino Temperature Control Library (DS18B20 temperature probes)

---

## Team

Built for Team Rudra inter domain team-A. Code contributions from Akash and Dhruv.
