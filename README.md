# Pico DShot
This repo is being developed to use a RPi Pico to send dshot commands to ESCs.
This is a work in progress.


## Setting up the repo

- git clone repo
- git submodule init
- git submodule update
- `cd lib/extern/pico-sdk; git submodule update --init` <-- This was required for `TinyUSB`
This repo is being developed to use a RPi Pico to send dshot commands to ESCs.
This is a work in progress.

## Running the Unit Tests

We use [Unity Test](https://github.com/ThrowTheSwitch/Unity) as the framework for our tests. The CMake file was inspired from the [Rainer Poisel](https://www.poisel.info/posts/2019-07-15-cmake-unity-integration/), [Throw The Switch](http://www.throwtheswitch.org/build/cmake) and [Testing with CMake and CTest](https://cmake.org/cmake/help/book/mastering-cmake/chapter/Testing%20With%20CMake%20and%20CTest.html). 
Assuming all the submodules have been setup

```terminal
mkdir test/build && cd $_
cmake ..
cmake --build .
ctest --verbose
```

---
## Code Overview

```
|-- lib
    |-- dshot
    |-- config
    |-- tts
    |-- shoot
    |-- utils
|-- src
    | -- main.cpp
|-- test
    | -- test_dshot.cpp
```

- `lib/` contains the libraries used to implement dshot on the pico
    - `dshot/` is a standalone module that is used to convert a dshot command to a frame that can be sent using pwm hardware
    - `config/` a header file for dshot globals, which are used to configure the hardware in `tts/`
    - `tts/` contains the hardware implementations for dma, pwm, alarm pool
    - `shoot/` implements sending dshot commands using the pwm hw as well as a repeating timer. A future merge request will add telem over uart in this module
    - `utils/` just an led flashing for debugging

- `src/` contains `main.cpp` which uses serial input to send dshot commands
- `test/` contains a unit test for `dshot/` as well as a cmake config file

Dependency Graph:

```
|-- shoot
|   |-- tts
|   |   |-- config
|   |-- dshot
|-- utils
```

## To Do
- [ ] Setup a UART port
- [ ] use a simple dshot cmd with telemetry to see if uart outputs
- [ ] May be necessary to set irq priority of uart to be lower than DMA. Note the the DMA timer must be sufficiently slow (which it should be). Otherwise, it may be better for uart interrupt to have a higher priority(!)


- [ ] Convert `dshot/` module to just a header file using static inlines (make sure to check it works with the unit tests)
- [ ] Move `config.h` to `tts/dshotglobals.h` . Wrap variables in a namespace and remove prefix `DSHOT_` (a bit difficult because dshot namespace already exists)
- [ ] `tts.h` can be renamed `dshothw.h`?
- [ ] Maybe change / clarify the use of *code* and *command*

## Backlog
- [ ] attempt proper arm sequence
- [ ] Try: `DSHOT_SPEED = DEBUG ? 0.008 : 1200 // kHz` (this may get rid of some other ternaries on DEBUG)
- [ ] Add validation to ensure PWM, DMA, repeating timer have been setup correctly. MCU_CLK could take in a measured reading. Use `lib/extern/pico-sdk/src/rp2_common/hardware_clocks/scripts/vcocalc.py` to find valid sys clock frequencies.
- [ ] Currently dma writes to a PWM counter compare. This actually writes to two dma channels (because upper / lower 16 bits are separate counters). Hence we render one dma channel useless. Is it possible to implement this in a better way?
- [ ] Do we need to use the Arduino framework? Or can we just use the Pico SDK and import libraries 3rd party libs if necessary? If the latter, we could either consider [Wiz IO](https://github.com/Wiz-IO/wizio-pico) or check out [this post](https://community.platformio.org/t/include-pico-stdlib-h-causes-errors/22997). 
- [ ] Transfer `print_*_setup` functions to logging / utils? Maybe check `refactor` branch. Maybe include the serial printf lib. There is an interesting post on [printable classes](https://forum.arduino.cc/t/printable-classes/438816)
- [ ] Scheme to represent DShot codes. enum is possible for special codes, but shouldn't be used for throttle values?
- [ ] Explore the idea of using / generating a look up table to convert dshot to pwm counter values (and vice versa for bidir dshot). 
Can this be hooked up with Programmable IO?
Memory usage: 2^16 command x 32 bit word = 32k x 64 bit word (that might be too much). 
- [ ] I believe that the uart and serial are initialised in `stdio_init_all()` (called in `main.cpp`). Perhaps we should use `stdio_usb_init()` and `stdio_uart_init()` separately. For telemetry (not yet committed), Peter used `stdio_uart_init()` after `stdio_init_all()` and didn't have errors.

---
## DShot Protocol

A flight controller (Pico) sends commands to the ESC to control the motor.
DShot is a digital protocol which typically sends commands using PWM hardware (note that we are *not* sending PWM frames, but are rather piggy backing off of its hardware to send dshot frames!). 
DShot is has discretised resolution, where as PWM was analogue on FC side (albeit discretised on the ESC due to hw limitations), 
however the advantages in accuracy, precision, resolution and communication outweigh this.

A Dshot *command* is constructed as follows:
- 11 bit number representing the *code* (0 to 2047)
- 1 bit telemetry flag
- 4 bit CRC

To transmit this *command* we construct a *frame* by converting the bits to a voltage signal. Each bit is represented as a duty cycle on pwm hw
- 0 = \< 33%
- 1 = > 75%
- Note that a prefix and suffix 0 duty cycle (not a 0 bit!) is required in transmission to tell each frame apart

The period of each bit (i.e. the period set in the PWM hw) is determined by by the DShot freq.
i.e. DShot150 sets the PWM hw freq to 150 kHz

Dshot *Codes*:
0 - Disarm (though not implemented?)
1 - 47: reserved for special use
48 - 2047: Throttle (2000 steps of precision)

For example, to make the motors beep the DShot frame is constructed as follows:

`Code = 1`, `Telemetry = 1`

→ First 12 bits of the command `0x003`

→ `CRC = 0 XOR 0 XOR 3 = 3`

→ `Command = 0x0033`

→ Transmitted from left to right, the frame is: `LLLL LLLL LLHH LLHH`

(Please note that this nomenclature is not standard, but I believe it is clear)

### Interesting Aside

It takes some time to calculate and send a dshot frame. To mitigate the effect of calculation, the implementation can use two buffers to store the DMA frame: a main buffer to store the current frame being sent by DMA, and a second buffer to store the next frame. This second buffer is only used if DMA is still sending the current frame, otherwise we default to storing the next frame in the main buffer. Further optimisation can be done by checking if the current dshot code and telem are the same as the next one, in which case there is no need to recalculate the frame. 

---
### UART Telemetry

This section is about acquiring Telemetry using the UART TX wire on the ESC.
Note that you may need to solder a wire on the ESC TX pad and hook it up to a GPIO UART RX pin on the pico. 

When sending a DShot frame, a *telemetry* bit can be set. 
Doing so will tell the ESC to send telemetry on the UART wire. 
The telemetry data consists of 10 bytes at 115200 baud. 
(I believe the nominal voltage is 3.6 V, but it might be a range from 2.5 V to 5.5 V depending on what the pull-up resistor is connected to on the microncontroller.) The bytes returned are as follows:

| Byte | Description           | Unit |
| ---- | --------------------- | ---- |
| 0    | Temperature           | 
| 1    | Voltage high byte     |
| 2    | Voltage low byte      |
| 3    | Current high byte     |
| 4    | Current low byte      |
| 5    | Consumption high byte |
| 6    | Consumption low byte  |
| 7    | Rpm high byte         |
| 8    | Rpm low byte          |
| 9    | 8-bit CRC             |

// Timing of UART
One *packet* of telemetry takes more time to send than a DShot frame:
// Calculate time to send a UART packet
// Calculate time(s) to send a DShot frame / or quote values and past calcs above

Note that the ESC will respond to every UART request.
This means that one must time the telem bit on the DShot command such that we don't overload it. 

Apparently the telemetry bit ~~accuracy~~ increases by using a pull-up resistor. I am still getting pretty bad bit rate (seems like CRC is only ever 50% right). 

Implementation ideas:

1. Use a repeating timer to set the telem flag. The dshot repeating timer will send the frame and then reset the telem flag to 0 (on every dshot callback)
   1. This adds extra instructions to the dshot callback
   2. Note: there could be interrupt race conditions considering the scenario below, if the priority of Telem rt is higher. However, [this post](https://forums.raspberrypi.com/viewtopic.php?t=328648) says that setting up the rt on the same alarm pool gives them ewual priority (i.e. callback isn't interrupted!)
      1. Dshot rt function sends frame
      2. Telem rt interrupts and sets telem flag; then returns
      3. Dshot frame func resumts, and resets telem flag...
      4. hence this telem flag is missed!

2. Use a repeating timer to set the telem flag, immediately send a dshot frame and then reset the telem flag 
   1. This can potentially impact the timing of dshot frames sent
   2. Note that the send dshot frame functions checks if the previous frame has been sent yet, so the above will not impinge on this 

3. Set the telem frequency, and calculate a "counter compare". Each time a dshot frame is sent, a counter will increment and compare against the telem counter compare; if it is the same, then we set the telem flag, otherwise reset the telem flag.
   1. This is essentially implementing a timer! But it doesn't interrupt the flow so much. 

4. 

---
## Thrust Test Stand

Aim: Gather data on motor performance

- linear ramp test (how long do we ramp for?)
- step response
- code a pid loop, and see it's effect on motor performance

- Interestingly, since the number of dshot values is discretised, 
it might be possible to create a table characterising the response 
from one throttle value to another!?

- In `send_dshot_frame()`, there could be a safety measure to see if the motors are responding, and only then send a command. If not, don't send a command, and the ESC will disarm itself. 

Possible pico telemetry:
- Count number of dshot frames sent
- Measure how long each main loop takes
- Measure min and max dshot update freq

---
## Sources

### USB Driver
- Use [Zadig](https://zadig.akeo.ie/) to install drivers for the RPi boot interface. This makes the flashing experience a LOT better! A thread on the [platform io forum](https://community.platformio.org/t/official-platformio-arduino-ide-support-for-the-raspberry-pi-pico-is-now-available/20792/9) goes in to more detail. [This thread[(https://community.platformio.org/t/raspberry-pi-pico-upload-problem/22809/7) also linked to this [github comment](https://github.com/platformio/platform-raspberrypi/issues/2#issuecomment-828586398) mentions to use Zadig. 

### Docs and Sample Implementation
- [Pico SDK API Docs](https://raspberrypi.github.io/pico-sdk-doxygen/modules.html). Some quick links: [dma](https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__dma)
- [Documentation on the Pico](https://www.raspberrypi.com/documentation/microcontrollers/?version=E0C9125B0D9B) incl spec, datasheets, [pinout](https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf), etc.
- [Pico examples](https://github.com/raspberrypi/pico-examples) from the rpi github incl `dma/`. There's an interesting example on pairing an adc with dma [here](https://github.com/raspberrypi/pico-examples/blob/master/adc/dma_capture/dma_capture.c). Note that when viewing pico examples, they use `#include "pico/stdlib.h"`. This is *not* to be used in the *Arduino* framework! as explained in [this post](https://community.platformio.org/t/include-pico-stdlib-h-causes-errors/22997). 

- [PlatformIO Documentation on Pico](https://docs.platformio.org/en/stable/boards/raspberrypi/pico.html#board-raspberrypi-pico). It only mentions the Arduino framework, but more seem to be avaialable (see other links here). 
- [Unity Assertion Reference](https://github.com/ThrowTheSwitch/Unity/blob/master/docs/UnityAssertionsReference.md) is a useful guide handbook for unit testing with Unity framework. 
- [Unit testing on Embedded Target using PlatformIO](https://piolabs.com/blog/insights/unit-testing-part-2.html)

### Explanations of DShot

- [Betaflight DShot Wiki](https://github.com/betaflight/betaflight/wiki/DSHOT-ESC-Protocol)
- [Spencer's HW Blog](https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs) has a quick overview on the DShot protocol, list of the dshot command codes (which shd be sourced somewhere in the [betaflight repo](https://github.com/betaflight/betaflight)), and implmenetation overviews using scp and dma. 
- [This post](https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/) has a simple explanation of dshot with a few examples
- [DShot - the missing handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/) has supported hw, dshot frame example, arming, telemetry, bi-directional dshot
- [BLHeli dshot special command spec](https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt)
- [Missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#special-commands) also has a good explanation of commands
[Original RC Groups post on dshot](https://www.rcgroups.com/forums/showthread.php?2756129-Dshot-testing-a-new-digital-parallel-ESC-throttle-signal)
- [SiieeFPV](https://www.youtube.com/watch?v=fNLxHWd0Bvg) has a YT vid explaining DMA implementation on a *Kinetis K66*. This vid was a useful in understanding what was needed for our implementation. 

### Other

- [Wiz IO Pico](https://github.com/Wiz-IO/wizio-pico): seems like an alternative to the Arduino framework used in PlatformIO? More details can be found on their [Baremetal wiki](https://github.com/Wiz-IO/wizio-pico/wiki/BAREMETAL)
- [Rpi Pico Forum post](https://forums.raspberrypi.com/viewtopic.php?t=332483). This person has balls to try and implemenet dshot using assembly!!

- List of [Arduino libraries for the RP2040](https://www.arduinolibraries.info/architectures/rp2040). I've not used any yet, but it might be best to use them as reference instead of importing them?
- Interesting post on [processing serial without blocking](http://www.gammon.com.au/serial)

- [Upload port required issue](https://github.com/platformio/platform-raspberrypi/issues/2). I don't think this issue will be faced if using Zadig

- [BLHeli 32 Manual](https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20manual%20ARM%20Rev32.x.pdf)

NOTE: (Although we don't use this functionality), a common implmentation measuring timer uses 32 bit time (note that 64 bit is possible using `timer_hw->timerawl` but more effort..)
- `timer_hw->timerawl` returns a 32 bit number representing the number of microseconds since power on
- This will overflow at around 72 hrs, so must assume that this longer than the operation timer of the pico
- This has been the cause of many accidental failures in the past :)

### Bidirectional DShot

- [DShot - the missing handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/) also talks about bi-directional dshot
- [RPi Pico Forum thread on Evaluating PWM signal](https://forums.raspberrypi.com/viewtopic.php?t=308269). This could be useful for bidirectional dshot
- [Pico Example to measure duty cycle](https://github.com/raspberrypi/pico-examples/blob/master/pwm/measure_duty_cycle/measure_duty_cycle.c)
- [Interesting PR on the first implementation of bidir dshot](https://github.com/betaflight/betaflight/pull/7264). This discussion alludes to the *politics* of the protocol
- Note that we need to check the FW version on the ESC to see if it supports bidir (and EDT). [BLHeli Passthrough](https://github.com/BrushlessPower/BlHeli-Passthrough) is implemented as an Arduino Lib for ESP32 and some Arduinos. A good exercise would be to add support for the Pico. [This](https://github.com/betaflight/betaflight/blob/master/src/main/drivers/serial_escserial.c#L943) may be a betaflight implementation of passthrough, but I couldn't understand it. [BLHeli Suite](https://github.com/bitdump/BLHeli) is also needed.
- Researching this topic, I came across DMA "burst" mode, which apparently helps in transitioning from send to receive. Not sure, but maybe a starting point can be achieved from [this post](http://forum.chibios.org/viewtopic.php?t=5677)
- [Chained DMA](https://vanhunteradams.com/Pico/DAC/DMA_DAC.html) may be useful to switch from write to read configuration
