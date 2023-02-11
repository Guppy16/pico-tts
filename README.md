# Pico DShot

---
## DShot Protocol

A flight controller (Pico) sends commands to the ESC to control the motor.
DShot is a digital protocol utilising PWM to send these commands.
DShot is necessarily slower and discretised compared to analogue PWM, 
however the advantages in accuracy, precision and communication outweigh this.

To send a command to the ESC, we send a _packet_, constructed as follows:
- 11 bit number representing the code (0 to 2047)
- 1 bit telemetry flag
- 4 bit CRC
- final 0 bit to signify end of packet. Not sure, but it looks like some ppl just set the duty cycle to 0 before and after transmission

Each bit represents a PWM duty cycle
- 0 = \< 33%
- 1 = > 75%

The PWM freq is determined by by the DShot speed:
- DShot150 has f = 150 kHz, etc.

---
### DShot Commands
NOTE: need a good source for this.
[Temp src](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#special-commands)

0 - Disarm (though not implemented?)
1 - 47: reserved for special use
48 - 2047: Throttle (2000 steps of precision)

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

// Time taken to receive telem packet over uart:
80 bits / 115200 baudrate = 695 us

Note that this doesn't take in to accounte the time taken to send a dshot frame with the telem bit set!

// Calculate time(s) to send a DShot frame / or quote values and past calcs above (note we use 20 bits for the frame instead of 16 bits for the command)
20 bits / 150 kHz = 133 us

20 bits / 1200 kHz = 17 us

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
## Scheme

- Main aim: setup a pid loop / kalman filter to perform a linear ramp?
- Interestingly, since the number of dshot values is discretised, 
it might be possible to create a table characterising the response 
from one throttle value to another!?

- Move ramp_motor / arm_motor to utils with a state machine
- create a flag in main.cpp to check if a character should be repeated

The ESC needs commands to be sent within a set interval (currently undetermined). In order to satisfy this, the code is modelled as follows:

NOTE: Timer interrupt should be longer than the time taken to send a command from the buffer using DMA. This is so that there is enough time for the Main loop to send the next command before the interrupt is raised!

Timer raises interrupt at set-interval rate
    This will restart DMA (i.e. re-send last known command)
    OR a safety measure can be done to check if the system is responding, else send a disarm cmd / NOT send a cmd so that ESC disarms by itself

Main loop:
    Calculate next command to send
    Reset Timer interrupt
    Wait for current command to finish
    Send command

Debugging:
    Measure how many times interrupt is called
    Measure how long each main loop takes
    Measure minimum time before motor sends command

Additions:
    Look at using a two buffers. One to store the current command being executed. The second one should store the next command to write. The idea is that this elimantes the  overhead in copying from the temporary buffer to the main buffer; instead, we are swapping the reference to the buffer where DMA gets data from. however, this complicated things due to having to keep track of which buffer is which..

NOTE: 32 bit time is used
- `timer_hw->timerawl` returns a 32 bit number representing the number of microseconds since power on
- This will overflow at around 72 hrs
- We assume that this is longer than the operation time before the microcontroller is restarted
- NOTE that this has been the cause of many accidental failures in the past..

---
## To Do
- [ ] Setup a UART port
- [ ] use a simple dshot cmd with telemetry to see if uart outputs
- [ ] Add unit tests for converting functions that interpret uart telem data (e.g. crc, unit conversions)

- [ ] May be necessary to set irq priority of uart to be lower than DMA. Note the the DMA timer must be sufficiently slow (which it should be). Otherwise, it may be better for uart interrupt to have a higher priority(!)

- :tick: Transfer PWM setup to `tts/`
- [ ] Transfer DMA setup to `tts/`
- [ ] Transfer repeating timer to `tts/`
- [ ] Transfer print config to logging / utils? Maybe check `refactor` branch
- [ ] Rename `config.h` to `dshot_config.h`. Wrap variables in a namespace and remove prefix `DSHOT_`.
- [ ] `tts.h` can be renamed `dshot_hw.h`?
- [ ] Transfer dma frame sending to `shoot.h`

## Backlog
- [ ] attempt proper arm sequence
- [ ] Try: DSHOT_SPEED = DEBUG ? 0.008 : 1200 kHz
- [ ] Add validation to ensure PWM, DMA, repeating timer have been setup correctly
- [ ] Currently dma writes to a PWM counter compare. This actually writes to two dma channels (because upper / lower 16 bits are separate counters). Hence we render one dma channel useless. Is it possible to implement this in a better way?
- [ ] Do we need to use the Arduino framework? Or can we just use the Pico SDK and import libraries 3rd party libs if necessary? If the latter, we could either consider [Wiz IO](https://github.com/Wiz-IO/wizio-pico) or check out [this post](https://community.platformio.org/t/include-pico-stdlib-h-causes-errors/22997). 

---
## Functions

- :tick: calculating duty cycle of bits from dshot speed
- enum to represent codes
- function in the processor to handle telemtry. Can this be handled in HW?
- code to packet (i.e. calculate checksum)

- Write some tests for these function!

NOTE:
- When re-setting DMA configuration, I tried just resetting the read address (assuming that the remaining configuration would stay the same). This did not work; one must re-configure the channel on every transfer. This seems inefficient.

---
## Test Data

Command: 1, Tel: 1
0x0033
Transmitted from left to right (I think)
LLLL LLLL LLHH LLHH


---
## Sources

### USB Driver issues with Windows
- Use [Zadig](https://zadig.akeo.ie/) to install drivers for the RPi boot interface. This makes the flashing experience a LOT better! A thread on the [platform io forum](https://community.platformio.org/t/official-platformio-arduino-ide-support-for-the-raspberry-pi-pico-is-now-available/20792/9) goes in to more detail. [This thread](https://community.platformio.org/t/raspberry-pi-pico-upload-problem/22809/7) also linked to this [github comment](https://github.com/platformio/platform-raspberrypi/issues/2#issuecomment-828586398) mentions to use Zadig. [This thread](https://github.com/raspberrypi/picotool/issues/20) also mentions an issue related to Windows not connectingto the Pico.

### Docs and Sample Implementation
- [Pico SDK API Docs](https://raspberrypi.github.io/pico-sdk-doxygen/modules.html). Some quick links: [dma](https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__dma)
- [Documentation on the Pico](https://www.raspberrypi.com/documentation/microcontrollers/?version=E0C9125B0D9B) incl spec, datasheets, [pinout](https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf), etc.
- [Pico examples](https://github.com/raspberrypi/pico-examples) from the rpi github incl `dma/`. [pwm led fade](https://github.com/raspberrypi/pico-examples/blob/master/pwm/led_fade/pwm_led_fade.c) motivated this repo's implementation. There's an interesting example on pairing an adc with dma [here](https://github.com/raspberrypi/pico-examples/blob/master/adc/dma_capture/dma_capture.c). Note that when viewing pico examples, they use `#include "pico/stdlib.h"`. This is *not* to be used in the *Arduino* framework! as explained in [this post](https://community.platformio.org/t/include-pico-stdlib-h-causes-errors/22997). 

- [PlatformIO Documentation on Pico](https://docs.platformio.org/en/stable/boards/raspberrypi/pico.html#board-raspberrypi-pico). It only mentions the Arduino framework, but more seem to be avaialable (see other links here). 

### Explanations of DShot

- [Spencer's HW Blog](https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs) has a quick overview on the DShot protocol, list of the dshot command codes (which shd be sourced somewhere in the [betaflight repo](https://github.com/betaflight/betaflight)), and implmenetation overviews using scp and dma. 
- [This post](https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/) has a simple explanation of dshot with a few examples
- [DShot - the missing handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/) has supported hw, dshot frame example, arming, telemetry, bi-directional dshot
- [rcgroups DShot thread](https://www.rcgroups.com/forums/showthread.php?2756129-Dshot-testing-a-new-digital-parallel-ESC-throttle-signal)

### Telemetry using UART

- [Kiss ESC Telemetry Datasheet](http://www.rcgroups.com/forums/showatt.php?attachmentid=8524039&d=1450424877)
- [Pico Advanced UART example with RX interrupt](https://github.com/raspberrypi/pico-examples/blob/master/uart/uart_advanced/uart_advanced.c)
- [Betaflight's implementation](https://github.com/betaflight/betaflight/blob/master/src/main/sensors/esc_sensor.c)
- [This post on BLHeli bitdump](https://github.com/bitdump/BLHeli/issues/431) mentions to use a 1k pull-up resistor to VCC to decrease error rate
- [UART with DMA and IRQ](https://forums.raspberrypi.com/viewtopic.php?p=2022767&hilit=pico+reset+timer#p2022767) this post may be useful for a more efficient implementation


### Other

- [Wiz IO Pico](https://github.com/Wiz-IO/wizio-pico): seems like an alternative to the Arduino framework used in PlatformIO? More details can be found on their [Baremetal wiki](https://github.com/Wiz-IO/wizio-pico/wiki/BAREMETAL)
- [Rpi Pico Forum post](https://forums.raspberrypi.com/viewtopic.php?t=332483). This person has balls to try and implemenet dshot using assembly!!

- List of [Arduino libraries for the RP2040](https://www.arduinolibraries.info/architectures/rp2040). I've not used any yet, but it might be best to use them as reference instead of importing them?

- [Upload port required issue](https://github.com/platformio/platform-raspberrypi/issues/2). I don't think this issue will be faced if using Zadig

- [Saleae logic analyzer](https://www.az-delivery.uk/en/products/saleae-logic-analyzer). [SW Download](https://support.saleae.com/logic-software/sw-download)

- [Android App for BLHeli 32](https://www.rcgroups.com/forums/showthread.php?3143134-Android-APP-for-BLHeli_32)
- [Betaflight wiki](https://github.com/betaflight/betaflight/wiki)
- [i programmer](https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1) had a good primer on PWM with the Pico
-  A good explanation of using a [dac with dma](https://vanhunteradams.com/Pico/DAC/DMA_DAC.html)
-  The ESC we are testing on is *Little Bee CloudPhoenix 50A ESC BLHeli32 (3-6S)* from [hobbyrc](https://www.hobbyrc.co.uk/little-bee-cloudphoenix-50a-esc-blheli32-3-6s)