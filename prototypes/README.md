# ISM_Emulator Prototypes
## Generate waveforms compatible with ISM remote sensors without Arduino IDE

These programs model ISM-band remote sensor waveform encoding in C++ and enable testing the waveform generation and playback algorithms without needing an attached Arduinio Uno device or sensors.

## Use

Simply compile the code of choice with the command `g++ -std=c++11` and execute the resulting `a.out` program.

These programs model how code to generate waveforms compatible with other remote-sensing devices might be developed.

## Program Structure

When asserting/deasserting voltage to the signal pin, timing is critical.  The strategy of this program is to have the "playback" -- the setting of voltages at specific times to convey information -- be as simple as possible to minimize computer processing delays in the signal-setting timings.  So the program generates a "waveform" as a series of program commands in an array to tell when to assert/deassert voltages and to delay the specified times to communicate information.  Those very simple commands in the array represent the waveform.

The playback, then, just retrieves the commands to assert/deassert voltages or delay specific length of time, and then executes them, with minimal processing overhead.  The program's `playback` procedure lists the pulse and gap durations that would asserted if the program were executed on an Arduino to drive a 433MHz transmitter.

This program was modeled, somewhat, on Joans `pigpiod` (Pi GPIO daemon) code for waveform generation.  But because the microcontroller devices such as Arduinos are single-program rather than multitasking OSes, the code here does not need to provide for contingencies as those in multi-tasking environments -- it just generates the waveform description which a subsequent code module uses to drive the transmitter voltages.

## Creating Other Devices
To create another ISM device, you'll need to:
*  Enumerate the types of signals and their delay durations: define those in `enum SIGNAL_T` and your device's equivalent of `AR609_signals[]`; 
*  Write a function to reformat the data as it comes from the sensor object into the format expected by the device: the AR609 code `pack_AR609()` demonstrates how to do that; be sure to set the expected message data length, too;
*  Write the procedure `.make_wave()` to create the array of signaling commands: again, `AR609()` demonstrates how to do that.

## Release History

*  V1.0: First operational version, 2024.12.29

## Author
Written by David Todd, hdtodd@gmail.com, v1.0.0 2025.01.14


