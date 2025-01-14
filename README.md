# ISM_Emulator
## Emulate ISM-Band Remote Sensors

This program implements `rtl_433`-recognizable ISM-band (433Mhz in the US) remote sensors on an Arduinio Uno or Sparkfun SAMD21.

`ISM_Emulator` provides the general model and `class` definitions for remote sensor emulation.  Its use is demonstrated in the implementation of several specific devices, including the Acurite AR609TXC temperature/humidity sensor and the Lacrosse WS7000-20 temperature/humidity/barometric-pressure sensor.  The transmissions from this program are recognized by `rtl_433` as those corresponding devices, and monitoring of the MQTT publications from `rtl_433` can be displayed by `DNT` or monitored with an MQTT client or via `rtl_watch`.

## Prototyping Code

In addition to the `.ino` code that implements specific devices, this distribution includes `C++` code that can be used to develop algorithms to create descriptions of the waveform prior to implementing on an Arduino-like device.  The prototypes do not require Arduino-like devices or transmitters but do expedite development of the Arduino code to instantiate the device.

## Requirements for Device Implementation

Implementation of one of the sensor devices requires:

* An Arduino or similar microcontroller that can be programmed through the Arduino IDE;
* An Arduino IDE installed on your host computer system;
* A 433MHZ transmitter (or other frequency as permitted in your locale), available from Amazon for ~$2US each but frequently sold as transmitter and receiver pairs packaged 5 or 6 pairs per package; 
* A temperature/humidity sensor.  This program uses a Bosch BME688 (available from Adafruit or Sparkfun for ~$19US), but other sensors such as the Silicon Labs Si7021, available from Adafruit or Sparkfun for ~$10US, would work equally well.

If your host microcontroller supports it, you might find the BME688 devices with Qwiic to be easiest to wire up.

## Use

Begin by cloning `ISM_Emulator` from http://github.com/hdtodd/ISM_Emulator .

To explore the prototyping code, connect to the `prototypes` directory and compile one of the `.cpp` files with `g++ -std=c++11 <prototype>.cpp`, then execute it with `./a.out`.

To implement one of the specific devices:

* Connect your components:
	* Arduino I2C or SPI connection to the BME688 or other sensor
	* Transmitter to VCC and ground, and data pin to an Arduino GPIO pin (pin 4 in this example)
	* USB connection from host to microcontroller for power and programming.
* If you have a BME688, install the library for it in your IDE; if you're using a different type of sensor, install its library and modify the `ISM_Emulator.ino` code to replace the BME calls with the equivalent for your sensor.
* Start the Arduino IDE
  	* Ensure that you have installed the libraries for `<Wire.h>`, `<SPI.h>`, `<Adafruit_Sensor.h>`, and `"Adafruit_BME680.h"` (or other libraries if you are using other sensor communication systems).
	* Open the `.ino` file from the `ISM_Emulator` directory you'd like to emulate (e.g. `AR609`).
	* Compile and download the code to your Arduino
	* Monitor execution with the Serial Monitor screen of your Arduino IDE.
* Monitor the transmitted sensor readings using `rtl_433`; enable MQTT publishing on the `rtl_433` host and monitor with an MQTT client or other tools such as `DNT` (http://github.com/hdtodd/DNT) or `rtl_watch` (http://github.com/hdtodd/rtl_watch).

## The `ISM_Emulator` Program

### Device Class

The `ISM_Emulator` code provides a base class containing structure definitions, variables, and procedures that can be inherited and expanded to emulate specific devices.

The device implemention program drives a 433MHz transmitter (or local ISM-band transmitter) to send temperature/humidity readings using the protocol specific to the device (Acurite 609TXC or Lacrosse WS7000-20, for example).  See the device file in the `rtl_433` distribution (https://github.com/merbanan/rtl_433) for details about the packet format for the specific device, or examine the prototype code procedure `make_wave`.  The data packet format created here matches the format recognized by `rtl_433`.

The transmitted waveform is a series of up/down voltages (pulses) that turn the ISM transmitter on/off (OOK) followed by timing gaps of various durations.  The duration indicates the type of signal (PWM -- pulse-width modulation).  The message is on-off keying/pulse-width modulation..

### The Acurite 609TXC

Transmission format is:

* 2 pulses followed by sync delays;
* 1 pulse followed by a sync-gap delay; 
* A 40-bit message of pulses followed by a short or long delay (short = 0, long = 1); and
* A final pulse follow by interval gap.

The function `AR609.make_wave()` shows how the waveform for any OOK/PWM device can be quickly specified for emulation.

Most ISM devices REPEAT the message 2-5 times per transmission to increase the possibility of correct reception (since this is a simplex communication system -- no indication that the information was correctly received).  This AR609 repeats the message 3 times per transmission.

The Acurite 609TXC sends only temperature and humidity data.

### The Lacrosse WS7000-20

Transmission format is:

* Ten "0" high-low pulses
* One "1" high-low pulse
* A 56-bit message transmitted as 12 4-bit nibbles, a CheckXOR nibble, and a CheckSum nibble, all transmitted least-significant-bit first, and with each 4-bit nibble followed by a "1" high-low pulse
* An inter-message gap pulse.

The WS7000-20 does not appear to repeat messages within a transmission, and this emulator is set to send only one message per transmission.

The Lacrosse WS7000-20 transmits temperature, humidity, and barometric pressure readings and so uses more of the data available from the BME 68x sensor.

## Program Structure

When asserting/deasserting voltage to the signal pin, timing is critical.  The strategy of this program is to have the "playback" -- the setting of voltages at specific times to convey information -- be as simple as possible to minimize computer processing delays in the signal-setting timings.  So the program generates a "waveform" as a series of program commands in an array to tell when to assert/deassert voltages and to delay the specified times to communicate information.  Those very simple commands in the array represent the waveform.

The playback, then, just retrieves the commands to assert/deassert voltages or delay specific length of time, and then executes them, with minimal processing overhead.  The result is that the various timings, as reported by `rtl_433 -A`, have very little variation within a transmission.

The program provides extensive feedback about its operation by printing to the serial or usbserial monitor if `#define DEBUG` is enabled (it is, by default).

This program was modeled, somewhat, on Joans `pigpiod` (Pi GPIO daemon) code for waveform generation.  But because the microcontroller devices such as Arduinos are single-program rather than multitasking OSes, the code here does not need to provide for contingencies as those in multi-tasking environments -- it just generates the waveform description which a subsequent code module uses to drive the transmitter voltages.

### Setup

Using the specific implementation of the AR609TXC as an example ...

The setup() procedure creates the BME688 object and sets its operational parameters.

The BME688 code for reading temp/press/hum/VOC was adapted from the Adafruit demo program http://www.adafruit.com/products/3660
The BME688 temperature reading may need calibration against an external thermometer.  The DEFINEd parameter `BME_TEMP_OFFSET` can be used to perform an adjustment, if needed.

### Loop
The loop() procedure samples the sensor (BME688), reformats the information into the 40-bit packet used by the Acurite 609TXC, creates an array of commands to drive the transmitter and delay the appropriate times, and then invokes AR609.playback() to actually drive the transmitter.  It then delays for the length of time defined by `#define DELAY`, in microseconds (must be less than 64K µs!) before repeating the loop.

## Creating Other Devices
To create another ISM device, you'll need to:
*  Enumerate the types of signals and their delay durations:
	*  Examine the device description in the `rtl_433` distribution under the `/src/devices` directory
	*  Examine the timings that can be analyzed by `rtl_433 -A -r <filename>` from the `rtl_433_tests` directory
	*  Examine the waveforms that can be visualized from the `http` link that `rtl_433 -A -r` provides.
*  Define those signal timings in your device's equivalent of `AR609_signals[]`; 
*  Write a function to reformat the data as it comes from the sensor object into the format expected by the device: the AR609 code `pack_AR609()` demonstrates how to do that; be sure to set the expected message data length, too;
*  Write the procedure `.make_wave()` to create the array of signaling commands: again, `AR609()` demonstrates how to do that.

## Release History

*  V1.0: First operational version, 2024.12.29; added WS7000 2025.01.14.

## Author
Written by David Todd, hdtodd@gmail.com, v1.0.0 2024.12.30.

