# tumbller-esp32s3
First version of tumbller with esp32s3 

Getting Started
---------------

### Requirements

* [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation/index.html). It works with the GUI too, which is based on the CLI. Only CLI documented as clear to explain and automate.
* A serial console like `cu` on Linux-compatible systems, RealTerm and PuTTY on Windows, etc.

### First time settings

When your ESP32 board is connected to your development machine, a device file is created. We need to tell PlatformIO which file that is for working properly. It usually needs be done only once (per board!)

```shell
platformio device list
```

The command outputs a list of available devices. Example output:

```shell
/dev/cu.BLTH
------------
Hardware ID: n/a
Description: n/a

/dev/cu.Bluetooth-Incoming-Port
-------------------------------
Hardware ID: n/a
Description: n/a

/dev/cu.usbserial-D309JH4R
--------------------------
Hardware ID: USB VID:PID=0403:6015 SER=D309JH4R LOCATION=20-1.2
Description: FT231X USB UART - FT231X USB UART
```

In this output, the device file turns out to be `/dev/cu.usbserial-D309JH4R`. On Windows, you usually get names like `COM3`.

Please copy this name and paste in inside `platformio.ini` file as the value for `upload_port`:

```shell
upload_port = /dev/cu.usbserial-D309JH4R

# Typically on Windows:
upload_port = COM3
```

### Usage

Assuming first-time settings are complete, building is:

```shell
platformio run
```

(First run is long as the toolchain collects dependencies for the target board; way faster from the second time!)

Upload to the board: `platformio run --target upload`

# TODO
# Serial console on Linux-compatible systems: cu -s 115200 -l /dev/tty.usbserial
