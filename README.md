# AVR Microcontroller Library

This library provides a collection of functions to simplify programming microcontrollers. It includes functions for configuring pins, controlling peripherals, and generating interrupts.

## Supported Microcontrollers

- Atmega328p (Arduino Uno)	https://www.e-lab.de/downloads/DOCs/mega328P.pdf

## Installation

To use this library in your project, simply download this repository and place it in your filesystem.

I use PlatformIO IDE, which is set up as a simple PlatformIO project in this repository.

I have an Arduino UNO with the Atmega328p mounted and the AVR ISP MK2 for programming the controller, as specified in the `platformio.ini` file.

The library is located in the `/lib` folder. Simply copy `/lib/MyAVR` to your project's individual library folder and include `MyAVR.hpp` in your `main.cpp`.

## Usage

1. Include the library header file in your program:
```cpp
#include <MyAVR.hpp>
```

2. Initialize the microcontroller by creating the Object:
```cpp
MyAtmega328p myAVR(F_CPU);
```

3. Use the provided functions to control pins, peripherals, and interrupts:
```cpp
myAVR.initUart(9600);
myAVR.setGpioConfig(inputPullup, DDRD, PORTD, PD3);
```

## PlatformIO

In the `platformio.ini` file, you can configure various settings such as fuse bits, CPU frequency, compiler, and build commands for the AVR ISP MKII.

```ini
[env:program_via_AVRISP_mkII]
platform = atmelavr
framework = arduino
board = ATmega328P
board_build.f_cpu = 8000000

upload_protocol = custom
upload_port = usb
board_fuses.lfuse = 0xE2
board_fuses.hfuse = 0xDE
board_fuses.efuse = 0xFD
upload_flags = 
	-C
	$PROJECT_PACKAGES_DIR/tool-avrdude/avrdude.conf
	-p
	$BOARD_MCU
	-P
	$UPLOAD_PORT
	-c
	avrispmkII
upload_command = avrdude $UPLOAD_FLAGS -U flash:w:$SOURCE:i
```

Feel free to modify this `ini` file to suit your configuration. I use the Atmega328p mounted on an Arduino UNO board with my AVR ISP MK2 connected to the board for development projects before transferring the Atmega328p to its new environment.

## Documentation
The detailed documentation for this project can be found under [doc/software/html/index.html](doc/software/html/index.html)

## Contributing

We welcome contributions from the community!

First, read the [general contributing conventions for projects from me](CONTRIBUTING.md).

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Commit your changes (`git commit -am 'Add new feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Create a new Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
