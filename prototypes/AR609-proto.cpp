/* g++ -std=c++11
  ISM_proto.cpp: Prototype ISM-device emulator for Arduino

  This program models and tests code to describe and generate
  waveform specifications on Arduino/Sparkfun devices.  The
  waveform specifications can be adapted to modulate ISM-band
  (e.g, 433MHz) on-off keying and pulse-width modulation (OOK/PWM) 
  transmitters to emulate ISM-band remote sensing devices.

  The waveform is a series of up/down voltages (cycles) that turn
  the ISM transmitter on/off (OOK) followed by timing gaps of
  various durations.  The duration indicates the type of signal
  (PWM -- pulse-width modulation).

  For the Acurite 609 remote sensor that this code was initially
  designed to emulate, the cycle begins with a "high" pulse
  of specific duration (500usec) followed by a "low" gap of
  specific duration.  The length of the gap indicates the type of
  information being transmitted (synching signals, or data bits
  such as 520usec ==> "0", 980usec ==> "1", etc).

  Most ISM devices REPEAT the message 2-5 times to increase the
  possibility of correct reception (since this is a simplex
  communication system -- no indication that the information
  was correctly received).

  When asserting/deasserting voltage to the signal pin, timing
  is critical.  The strategy of this program is to have the
  "playback" -- the setting of voltages at specific times to
  convey information -- be as simple as possible to minimize
  computer processing delays in the signal-setting timings.
  So the program generates a "waveform" as a series of commands
  to assert/deassert voltages and to delay the specified times
  to communicate information.  Those commands are entered into
  an array to represent the waveform.

  The playback, then, just retrieves the commands to assert/deassert
  voltages and delay specific length of time and executes them, with
  minimal processing overhead.
  
  This program was modeled, somewhat, on Joan's pigpiod (Pi GPIO daemon)
  code for waveform generation.  But because the Arduino-like devices
  are single-process/single-core rather than multitasking OSes, the code
  here does not need to provide for contingencies in that multi-tasking
  environment -- it just generates the waveform description which a subsequent
  code module uses to drive the transmitter voltages.
  
  The code here provides a base CLASS containing structure definitions,
  variables, and procedures that can be inherited and expanded to
  emulate specific devices.

  The list of commands is generated in .make_wave() by a sequence 
  of procedure calls to insert the appropriate commands for timings
  ("sync", "0", "1", etc) into the command list.
  
  hdtodd@gmail.com, 2024.12.20
*/

#include <stdint.h>
#include <string>
#include <iostream>
#include <iomanip>
using namespace std;

#undef HAVE_SENSOR

#ifdef HAVE_SENSOR
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#else
class SENSOR {
public:

  float temperature, humidity, pressure, gas_resistance;
  
  SENSOR() {
  };

  bool begin() {
    return true;
  };

  void setTemperatureOversampling() {
  };
  void setHumidityOversampling() {
  };
  void setPressureOversampling() {
  };
  void setIIRFilterSize() {
  };
  void setGasHeater() {
  };
  
  bool performReading() {
    temperature =       20.0;
    humidity    =       50.0;
    pressure    =     1000.0;
    gas_resistance  =   78.0;
    return true;
  };
}; // end SENSOR
#endif

/*  "SIGNAL_T" are the possible signal durations..  This list may be
    extended with additional types in the future, but for the Acurite 609
    for which this code is the prototype, we use:
      SIG_PULSE:     the duration of the "pulse" assert voltage level
      SIG_SYNC:      the duration of the "sync" deassert voltage level
      SIG_SYNC_GAP:  the duration of the "sync_gap" deassert before data bits
      SIG_ZERO:      the duration of the deassert gap after a pulse
                     that signifies a "0" data bit
      SIG_ONE:       the duration of the deassert gap after a pulse
                     that signifies a "1" data bit
      SIG_IM_GAP:    the duration of the deassert for the
                     inter-message gap, between transmissions
*/

// Enumerate the possible signal duration types here for use as indices
// Append additional signal timings here and then initialize them
// in the device initiation of the "signals" array
// Maintain the order here in the initialization code in the device class
enum SIGNAL_T {NONE=-1,SIG_PULSE=0, SIG_SYNC, SIG_SYNC_GAP,
	       SIG_ZERO, SIG_ONE, SIG_IM_GAP};

// Structure of the table of timing durations used to signal
typedef struct {
  SIGNAL_T   sig_type ;   // Index the type of signal
  string     sig_name;    // Provide a brief descriptive name
  uint16_t   delay_us;    // The formally-specified duration in microseconds
} SIGNAL;

// Types of commands in the device action list
// WAIT: delay time before next action
// ASSERT: set voltage to 'assert' level
// DEASSERT: set voltage to 'deassert' level
// DONE: no more commands in the list
enum CMD_T {DONE=-1, WAIT=0, ASSERT, DEASSERT};

// "CMD" is the command structure itself: type of command,
//  and if it is a signal, which type of signal
typedef struct {
  CMD_T      cmd;
  SIGNAL_T   signal;
} CMD;

/* ISM_Device is the base class descriptor for various specific OOK-PWM
   emulators.  It contains the command list for the transmitter driver,
   the variables needed to translate procedure calls into the command list,
   and the procedures needed to insert the corresponding commands into the list.
*/
class ISM_Device {
public:
    
  // These are used by the device object procedures
  //   to process the waveform description into a command list
  CMD      cmdList[640];
  uint16_t listEnd=0;
  string   Device_Name="ISM Device";
  uint8_t  Lo=0;
  uint8_t  Hi=1;
  SIGNAL*  signals;
  
  ISM_Device() {
  };

  // Inserts a signal into the commmand list
  // Assert for "on" time, then deassert for "off" time
  void insert(SIGNAL_T on, SIGNAL_T off) {
    cmdList[listEnd++] = {ASSERT,   on};
    cmdList[listEnd++] = {DEASSERT, off};
    return;
  };

  // Plays back the signaling commands in the device's cmdList[]
  void playback() {
    SIGNAL_T this_sig;
    cout << "\nFor device " << Device_Name << " the wave generated is:" << endl;
    for (int i = 0; i<listEnd; i++) {
      this_sig = cmdList[i].signal;
      cout << setw(3) << right << i << ": Cmd= "
           << setw(2) << right << (int) cmdList[i].cmd;
      switch (cmdList[i].cmd) {
        case WAIT:
	  cout << setw(9) << "WAIT" << setw(6) << right << signals[this_sig].delay_us
	       << "\t" << signals[this_sig].sig_name;
	  break;
	case ASSERT:
	  cout << setw(9) << "ASSERT";
	  cout << "\tWAIT" << setw(6) << right << signals[this_sig].delay_us
	       << "\t" << signals[this_sig].sig_name;
	  break;
	case DEASSERT:
	  cout << setw(9) << "DEASSERT";
	  cout << "\tWAIT" << setw(6) << right << signals[this_sig].delay_us
	       << "\t" << signals[this_sig].sig_name;
	  break;
	default:
        case DONE:   // Terminates list but should never be executed
	  cout << " \tERROR -- invalid command: " << (int) cmdList[i].cmd
	       << ( (cmdList[i].cmd == DONE) ? " (DONE)" : "");
	  break;
      };
      cout << endl;
    };
  };
};
    
class AR609 : public ISM_Device {

public:
// Routines to create 40-bit AR609 datagrams from sensor data
// Pack <ID, Status, Temp, Humidity> into a 5-byte AR609 message
//  with 1 checksum byte
void pack_msg(uint8_t id, uint8_t status, int16_t temp,
	      uint8_t hum, uint8_t *msg) {
  msg[0] = ( id&0xff );
  msg[1] = ( (status&0x0f)<<4 | (temp>>8)&0x0f );
  msg[2] = ( temp&0xff );
  msg[3] = ( hum&0xff ); 
  msg[4] = ( msg[0] + msg[1] + msg[2] + msg[3] ) & 0xff;
  return;
};

// Unpack <ID, Status, Temp, Humidity> from a 5-byte AR609 message with 1 checksum byte
void unpack_msg(uint8_t *msg, uint8_t &I, uint8_t &S, int16_t &T, uint8_t &H) {
  if (msg[4] != ( (msg[0]+msg[1]+msg[2]+msg[3])&0xff) ) {
    cout << "Invalid message packet: Checksum error" << endl;
    I = 0;
    S = 0;
    T = 0;
    H = 0;
    } 
  else {
    I = msg[0];
    S = (msg[1]&0xf0) >> 4;
    // Contortions needed to create signed 16-bit from unsigned 4-bit | 8-bit fields
    T =  ( (int16_t) ( ( msg[1]&0x0f ) << 12 | ( msg[2]) << 4 ) ) >> 4; 
    H = msg[3];
    };
  return;
};

  
// AR609 timing durations
// Name, description, spec'd delay in us, place for adjusted duration
int sigLen = 6;
SIGNAL AR609_signals[6] = {
  {SIG_PULSE,    "Pulse",      512},
  {SIG_SYNC,     "Sync",      8912},
  {SIG_SYNC_GAP, "Sync-gap",   480},
  {SIG_ZERO,     "Zero",       976},
  {SIG_ONE,      "One",       1968},
  {SIG_IM_GAP,   "IM_gap",   10200}
};

// Instantiate the device by linking 'signals' to our device timing
AR609() {
  Device_Name = "Acurite 609";
  signals = AR609_signals;
  listEnd = 0;
  cout << "Created device " << Device_Name << endl;
};

void make_wave(uint8_t *msg, uint8_t msgLen) {
  listEnd = 0;
  // Preamble
  insert(SIG_PULSE,SIG_SYNC_GAP);
  insert(SIG_PULSE,SIG_SYNC_GAP);
  insert(SIG_PULSE,SIG_SYNC);
  cout << "The msg packet, length=" << (int) msgLen << ", as a series of bits: ";
  // The data packet
  for (int i=0; i<msgLen; i++) {
    insert(SIG_PULSE,
	   ( (uint8_t) ((msg[i/8]>>(7-(i%8))) & 0x01 ) ) == 0 ? SIG_ZERO : SIG_ONE );
    
    cout << setw(1) << ((msg[i/8]>>(7-(i%8)))&0x01);
  };
  cout << endl;
  // Postamble and terminal marker for safety
  insert(SIG_PULSE,
	 SIG_IM_GAP);
  cmdList[listEnd].cmd = DONE;
};
};
 
int main() {
  // In the absence of a real sensor provide values needed
  //   to emulate the Acurite 609TXC
  uint8_t id =     199;
  uint8_t status =   1;
  int16_t temp =    20;
  uint8_t hum =     50;
  uint8_t i, s, h;
  int16_t t;

  // AR609 messages are 40 bits = 5 bytes
  uint8_t msg[5];
  uint8_t msgLen = 40;

  cout << "\nGenerate an ISM-device waveform and play back as pulse cycles\n" << endl;
  cout << "Create 'ar' as an AR609" << endl;
  AR609 ar;

  cout << "\nNow create a wave for 'ar'" << endl;
  ar.pack_msg(id, status, temp, hum, msg);
  cout << "Device ar message in hex: 0x ";
  ar.unpack_msg(msg, i, s, t, h);
  for (int i=0; i<5; i++)
    cout << setw(2) << setfill('0') << hex << (int) msg[i] << " ";
  cout << dec << setfill(' ') << endl;
  cout << "Message values: id = " << (int) i << "; status = " << (int) s
       << "; temp = " << (int) t << "; humidity = " << (int) h << endl;
  ar.make_wave(msg, msgLen);

  temp = 25;
  hum  = 70;

  cout << "\nCreate another wave for 'ar' and play it back as pulse cycles" << endl;
  ar.pack_msg(id, status, temp, hum, msg);
  cout << "Device ar message in hex: 0x ";
  ar.unpack_msg(msg, i, s, t, h);
  for (int i=0; i<5; i++)
    cout << setw(2) << setfill('0') << hex << (int) msg[i] << " ";
  cout << dec << setfill(' ') << endl;
  cout << "Message values: id = " << (int) i << "; status = " << (int) s
       << "; temp = " << (int) t << "; humidity = " << (int) h << endl;
  ar.make_wave(msg, msgLen);
  
  cout << "\nPlay back ar" << endl;
  ar.playback();

  return 0;
};
