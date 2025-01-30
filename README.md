# Compiling Raspberry Pi Pico C/C++ programs on Fedora

This repository contains hints and tips on how to configure your
Fedora workstation to cross compile Raspberry Pi Pico C/C++ programs.

## Raspberry Pi Pico Introduction

The [Raspberry Pi Pico board](https://www.raspberrypi.org/documentation/pico/getting-started/), based on the RP2040 microcontroller, is an interesting MCU for hardware and software engineers.
In 2021, I purchased several Pico boards and soldered headers onto the PCB.

When the [Raspberry Pi Pico 2 board](https://www.raspberrypi.com/products/raspberry-pi-pico-2/), based on the RP2350, was announced in August 2024, I purchased a Raspberry Pi Pico 2 Wireless.

Since I run [Fedora Linux](fedoraproject.org/) on my laptop, I wanted to cross
compile the RP2040 / RP2350 example programs using the
[Raspberry Pi Pico C/C++ SDK](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf).
The SDK guide provides instructions on how to install the SDK and examples on a Raspberry Pi.
While I have lots of Raspberry Pi devices, I don't use them as a development
platform for obvious performance reasons. The installation scripts assume a
Raspberry Pi OS running on a Raspberry Pi 4 or an equivalent Debian-based Linux distribution.

## Install Fedora dependencies

My Fedora 41 system has most of the `dnf grouplist "C Development Tools and Libraries"` packages
already installed, so this guide doesn't cover how to install binutils, gcc, gcc-c++, cmake, make, doxygen, git toolchains.
If you're new to C/C++ development on Fedora, there are better guides for those steps.
This section of the tutorial details how to install the ARM cross compiler packages.

### Install the ARM cross compiler dependencies

There's a great [Fedora Embedded wiki](https://fedoraproject.org/wiki/Embedded) that provided
helpful hints.

Here's a minimal list of dependencies that will pull in additional required packages.
You will need approximately 1GB of disk space.

```bash
sudo dnf install gcc-arm-linux-gnu \
 arm-none-eabi-gcc-cs-c++ \
 arm-none-eabi-gcc-cs \
 arm-none-eabi-binutils \
 arm-none-eabi-newlib
```

### Clone the pico-sdk, pico-examples, pico-extras repositories

The `pico-examples` repository ([https://github.com/raspberrypi/pico-examples](https://github.com/raspberrypi/pico-examples))
provides a set of example applications that are written using the
`pico-sdk` ([https://github.com/raspberrypi/pico-sdk](https://github.com/raspberrypi/pico-sdk)).

Run the following commands to clone the pico-sdk, pico-examples, pico-extras repositories
to an appropriate development subdirectory on your system.

```bash
git clone git@github.com:raspberrypi/pico-sdk.git
git clone git@github.com:raspberrypi/pico-examples.git
git clone git@github.com:raspberrypi/pico-extras.git
```

There is a `tinyusb` git submodule directory that you should clone too.

```bash
cd ./pico-sdk/src/rp2_common/tinyusb
git submodule update --init
cd -
```

The `picotool` utility requires the mbedtls submodule for its signing and hashing features.

```bash
cd ./pico-sdk
git submodule update --init lib/mbedtls
cd -
```

### Additional dependencies

To compile the various `picow_bt_example_*` examples, the programs needs the PyCryptodome python package.

- PyCryptodome required to calculate GATT Database Hash but not installed (using random value instead)
- Please install PyCryptodome, e.g. 'pip3 install pycryptodomex' or 'pip3 install pycryptodome'

```bash
pip3 install pycryptodomex
```

### More examples are available but require additional configuration

- Skipping universal examples as PICO_RISCV_TOOLCHAIN_PATH and PICO_ARM_TOOLCHAIN_PATH are not defined
- Skipping TinyUSB dual examples, as TinyUSB hw/mcu/raspberry_pi/Pico-PIO-USB submodule unavailable
- Skipping FreeRTOS examples as FREERTOS_KERNEL_PATH not defined

### Compile the Pico Examples

Depending on which RP Pico device you have, set the `PICO_PLATFORM` cmake flag to **[rp2040, rp2350, rp2350_riscv]**.
Valid `PICO_BOARD` flags are **[pico, pico_w, pico2, pico2_w]**.
Set these parameters in `pico_examples/CMakeLists.txt`, define them on the `cmake` command line, or as environment variables.
For instance, add this line at the top of `pico_examples/CMakeLists.txt`

```cmake
set(PICO_BOARD pico2_w)
```

The next step will take a few minutes.

```bash
cd pico-examples
export PICO_PLATFORM=[rp2040, rp2350, rp2350_riscv]
export PICO_BOARD=[pico, pico_w, pico2, pico2_w]
cmake -DPICO_SDK_PATH=../pico-sdk -DPICO_EXTRAS_PATH=../pico-extras -DPICO_PLATFORM=$PICO_PLATFORM -DPICO_BOARD=$PICO_BOARD .

make
```

If you have a `pico_w` or a `pico2_w` board, you might want to compile the wifi examples and enable the board to connect to your network.
In that case, specify the WIFI_SSID and WIFI_PASSWORD on the `cmake` command line (or in the `CMakeList.txt`)

```bash
cd pico-examples
export PICO_PLATFORM=[rp2040, rp2350, rp2350_riscv]
export PICO_BOARD=[pico, pico_w, pico2, pico2_w]
export YOUR_WIFI_SSID=<your_wifi_ssid>
export YOUR_WIFI_PASSWORD=<your_wifi_password>
cmake -DPICO_SDK_PATH=../pico-sdk -DPICO_EXTRAS=1 -DPICO_EXTRAS_PATH=../pico-extras -DPICO_PLATFORM=$PICO_PLATFORM -DPICO_BOARD=$PICO_BOARD -DWIFI_SSID=$your_wifi_ssid -DWIFI_PASSWORD=$your_wifi_password .

make
```

The compiler generates a **.uf2** binary for each sample program.

### Flashing the RP2040 / RP2350 Binary

The simplest method to load software onto a RP2040 or RP2350 based board is by mounting it as a USB Mass Storage Device. Connect your Fedora laptop to your Raspberry Pi Pico / Pico2 using a Micro-USB cable, making sure that you hold down the BOOTSEL button to force it into USB Mass Storage mode. You can drag the **.uf2** file onto the board to program the flash. The RP2040 / RP2350 will reboot, unmounting itself as a Mass Storage Device, and start to run the flashed code.

![BOOTSEL button](https://projects-static.raspberrypi.org/projects/getting-started-with-the-pico/725a421f3b51a5674c539d6953db5f1892509475/en/images/Pico-bootsel.png)

There's a nice little [animated gif](https://www.raspberrypi.org/documentation/pico/getting-started/static/92dabbc476b6b5ac7600c85a2df88200/Blink-an-LED-FINAL.gif).

## Run the Pico Examples

The `pico-examples` repository contains a variety of sample programs.  The Raspberry Pi Foundation team and community have created some great examples that showcase the capabilities of the RP2040 / RP2350. I flashed and wired up a few of them on a breadboard.

### Hello World (USB) Example

Start off with the [Hello World program](https://github.com/raspberrypi/pico-examples/tree/master/hello_world) for Pico.

- Hold down the BOOTSEL button to force it into USB Mass Storage mode.
- Drag or copy the `pico-examples/hello_world/usb/hello_usb.uf2` to the **RPI-RP2** mounted volume.
- Open a serial terminal on `/dev/ttyACM0`

  ```bash
  $ miniterm.py /dev/ttyACM0 115200
  --- Miniterm on /dev/ttyACM0  115200,8,N,1 ---
  --- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
  Hello, world!
  Hello, world!
  Hello, world!
  Hello, world!
  Hello, world!

  --- exit ---
  ```

### Blink Example

Blink the onboard LED by flashing the [Blink program](https://github.com/raspberrypi/pico-examples/tree/master/blink) for Pico.  For more tips you can read the Raspberry Pi Blog
["How to blink an LED with Raspberry Pi Pico in C"](https://www.raspberrypi.org/blog/how-to-blink-an-led-with-raspberry-pi-pico-in-c/)

- Hold down the BOOTSEL button to force it into USB Mass Storage mode.
- Drag or copy the `pico-examples/blink/blink.uf2` to the **RPI-RP2** mounted volume.

### DHT Temperature Sensor Example

The [DHT Temperature / Humidity sensor program](https://github.com/raspberrypi/pico-examples/blob/master/gpio/dht_sensor) reads a DHT-11 / DHT-22 via GPIO pin 15.

- Hold down the BOOTSEL button to force it into USB Mass Storage mode.
- Drag or copy the `pico-examples/gpio/dht_sensor/dht.uf2` to the **RPI-RP2** mounted volume.
- Wire a DHT sensor on a breadboard.  (The [Raspberry Pi Pico C SDK Guide](https://datasheets.raspberrypi.org/pico/raspberry-pi-pico-c-sdk.pdf) pg73 has a nice wiring diagram.)
  ![DHT Pico image](images/pico-dht22.jpg)
- Open a serial terminal on `dev/ttyACM0`

  ```bash
  $ miniterm.py /dev/ttyACM0 115200
  --- Miniterm on /dev/ttyACM0  115200,8,N,1 ---
  --- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
  Humidity = 22.8%, Temperature = 18.6C (65.5F)
  Humidity = 22.9%, Temperature = 18.6C (65.5F)
  Humidity = 22.7%, Temperature = 18.6C (65.5F)
  Humidity = 22.5%, Temperature = 18.6C (65.5F)
  Humidity = 22.6%, Temperature = 18.6C (65.5F)

  --- exit ---
  ```

### Neopixel WS2812B RGB LEDs Example

Every IoT hobbyist has a Neopixel and loves a fully addressable blinky light.  Use the [ws2812 program](https://github.com/raspberrypi/pico-examples/tree/master/pio/ws2812) to blink an external RGB LED.

- Hold down the BOOTSEL button to force it into USB Mass Storage mode.
- Drag or copy the `pico-examples/pio/ws2812/pio_ws2812.uf2` to the **RPI-RP2** mounted volume.
- Wire a Neopixel to your Raspberry Pi Pico.

  ![Neopixel Pico image](images/pico-neopixel.jpg)

### WiFi Example - Pico W / Pico2 W httpd

To demonstrate connecting the Pico_w / Pico2_w to your WiFi, the [httpd](https://github.com/raspberrypi/pico-examples/tree/master/pico_w/wifi/httpd) example is useful. This example uses the [cyw43 library](https://www.raspberrypi.com/documentation/pico-sdk/networking.html#group_cyw43_driver) and CYW43439A0 WiFi chip on the Raspberry Pi Pico W and Pico2 W microcontrollers.

```bash
$ miniterm.py /dev/ttyACM0 115200
--- Miniterm on /dev/ttyACM0  115200,8,N,1 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
Connected.

Ready, running httpd at 192.168.1.107
mdns host name PicoWF544.local

Ready, running httpd at http://192.168.1.107
```

### Bonus - picotool

If you want to use the picotool utility to inspect / load programs onto the RP2040,
Appendix B of the [SDK guide](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
details the installation instructions.  You will need to install these Fedora packages:

```bash
sudo dnf install libusb1-devel libusb1
```

### Board Specifications

- [RP2040 wiring diagram](https://www.raspberrypi.org/documentation/pico/getting-started/static/15243f1ffd3b8ee646a1708bf4c0e866/Pico-R3-Pinout.svg)
- [RP2350 pinout diagram](https://datasheets.raspberrypi.com/picow/pico-2-w-pinout.pdf)

### Author

- [John Walicki](https://github.com/johnwalicki)

___

Enjoy!  Give me [feedback](https://github.com/johnwalicki/RaspPi-Pico-Examples-Fedora/issues) if you have suggestions on how to improve this tutorial.

## License

This tutorial is licensed under the BSD License.
