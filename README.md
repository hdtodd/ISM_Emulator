# ISM_Emulator
## Emulate ISM remote sensors

This program models an ISM-band remote sensor on an Arduinio Uno or Sparkfun SAMD21.  This version specifically emulates an Acurite 609TXC temperature/humidity sensor, but the program provides the class definitions for emulating other devices.  The transmissions from this program are recognized by `rtl_433` as an Acurite 609TXC remote temperature/humidity sensor, and monitoring of the MQTT publications from `rtl_433` can be displayed by `DNT` or monitored via `rtl_watch`.

## Requirements
Implementation of an Acurite 609TXC emulator with this program requires:
* An Arduino or similar microcontroller that can be programmed through the Arduino IDE;
* An installed Arduino IDE on your host computer system;
* A 433MHZ transmitter (or other frequency as permitted in your locale), available from Amazon for ~$2US each but frequently sold as transmitter and receiver pairs packaged 5 or 6 pairs per package; 
* A temperature/humidity sensor.  This program uses a Bosch BME688 (available from Adafruit or Sparkfun for ~$19US), but other sensors such as the Silicon Labs Si7021, available from Adafruit or Sparkfun for ~$10US, would work equally well.

If your host microcontroller supports it, you might find the BME688 devices with Qwiic to be easiest to wire up.

## Use

* Connect your components:
	* Arduino I2C connection to the BME688 or other sensor
	* Transmitter to VCC and ground and data pin to an Arduino GPIO pin (pin 4 in this example)
	* USB connection from host to microcontroller for power and programming.
* Download `ISM_Emulator` from http://github.com/hdtodd/ISM_Emulator
* Compile ISM_Emulator.ino and download into your Arduino
* Monitor using `rtl_433`; enable MQTT publishing on the `rtl_433` host and monitor with an MQTT client or other tools such as `DNT` (http://github.com/hdtodd/DNT) or `rtl_watch` (http://github.com/hdtodd/rtl_watch).

## The `ISM_Emulator` Program

### Device Class

The `ISM_Emulator` code provides a base CLASS containing structure definitions, variables, and procedures that can be inherited and expanded to emulate specific devices and uses that base CLASS to implement the Acurite 609.

This program drives  a 433MHz transmitter (or local ISM band transmitter) to send temperature/humidity readings using the Acurite 609TXC protocol.  See the device/acurite.c file in the rtl_433 distribution (https://github.com/merbanan/rtl_433) for details about the packet format.  The data packet format created here matches the format recognized by rtl_433 for the Acurite 609TXC.

The transmitted waveform is a series of up/down voltages (pulses) that turn the ISM transmitter on/off (OOK) followed by timing gaps of various durations.  The duration indicates the type of signal (PWM -- pulse-width modulation).  The message is on-off keying/pulse-width modulation..

### The Acurite 609TXC

Transmission format is:
* 2 pulses followed by sync-gap delays;
* 1 pulse followed by a sync delay; 
* A 40-bit message of pulses followed by a short or long delay (short = 0, long = 1); and
* A final pulse follow by interval gap.

The device class function ISM_Device.make_wave() shows how the waveform for any OOK/PWM device can be quickly specified for emulation, and AR609.make_wave() implements the Acurite 609TXC format described above.

Most ISM devices REPEAT the message 2-5 times per transmission to increase the possibility of correct reception (since this is a simplex communication system -- no indication that the information was correctly received).  This AR609 repeats the message 3 times per transmission.

## Program Structure

When asserting/deasserting voltage to the signal pin, timing is critical.  The strategy of this program is to have the "playback" -- the setting of voltages at specific times to convey information -- be as simple as possible to minimize computer processing delays in the signal-setting timings.  So the program generates a "waveform" as a series of program commands in an array to tell when to assert/deassert voltages and to delay the specified times to communicate information.  Those very simple commands in the array represent the waveform.

The playback, then, just retrieves the commands to assert/deassert voltages or delay specific length of time, and then executes them, with minimal processing overhead.  The result is that the various timings, as reported by "rtl_433 -A", have very little variation within a transmission.

The program provides extensive feedback about its operation by printing to the serial or usbserial monitor if `#define DEBUG` is enabled (by default).

This program was modeled, somewhat, on Joans pigpiod (Pi GPIO daemon) code for waveform generation.  But because the microcontroller devices like Arduinos are single-program rather than multitasking OSes, the code here does not need to provide for contingencies as those in multi-tasking environments -- it just generates the waveform description which a subsequent code module uses to drive the transmitter voltages.

### Setup
The setup() procedure creates the BME688 object and sets its operational parameters.

The BME688 code for reading temp/press/hum/VOC was adapted from the Adafruit demo program http://www.adafruit.com/products/3660
The BME688 temperature reading may need calibration against an external thermometer.  The DEFINEd parameter 'BME_TEMP_OFFSET' can be used to perform an adjustment, if needed.

### Loop
The loop() procedure samples the sensor (BME688), reformats the information into the 40-bit packet used by the Acurite 609TXC, creates an array of commands to drive the transmitter and delay the appropriate times, and then invokes AR609.playback() to actually drive the transmitter.  It then delays for the length of time defined by `#define DELAY`, in microseconds (must be less than 64K us!) before repeating the loop.

## Release History

*  V1.0: First operational version, 2024.12.29

## Author
Written by David Todd, hdtodd@gmail.com, v1.0.0 2024.12.30.

