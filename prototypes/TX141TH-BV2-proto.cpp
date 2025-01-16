/* g++ -std=c++11
  TX141TH-BV2-proto.cpp: Generate TX141-compatible waveforms

  This program models and tests code to describe and generate
  waveform specifications on Arduino/Sparkfun devices.  The
  waveform specifications can be adapted to modulate ISM-band
  (e.g, 433MHz) on-off keying and pulse-width modulation (OOK/PWM) 
  transmitters to emulate the transmission protocols used by
  ISM-band remote sensing devices.

  The waveform is a series of up/down voltages (cycles) that turn
  the ISM transmitter on/off (OOK) followed by timing gaps of
  various durations.  The duration indicates the type of signal
  (PWM -- pulse-width modulation).

  For the Lacrosse TX141TH-BV2, the data message is an encoded
  message containing the device id, device status, temperature
  (as an integer to 10ths of a degree), humidity (as integer),
  and LFSR checksum byte.  Tthe transmission begins with
  4 sync pulses followed by 40 data bits representing the
  sensor data and checksum, followed by  2 sync pulses
  before repeating the data message.  The end of transmission is
  signaled by 2 sync pulses and an inter-message gap.
  Syncs are 712us high followed by a 936us gap.
  "0" is 348us high followed by 348us gap.
  "1" is 136us high followed by 936us gap.
  IM gap is 10000us.
  The message bits are inverted before sending (and re-inverted
  by rtl_433).
  
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
  
  The code here provides a base class containing structure definitions,
  variables, and procedures that can be inherited and expanded to
  create waveforms compatible with other specific devices, followed
  by code to implement the TX141.  
  
  hdtodd@gmail.com, 2025.01.15
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
      SIG_SYNC:      the duration of the "sync" deassert voltage level
      SIG_SYNC_GAP:  the duration of the "sync_gap" deassert before data bits
      SIG_ZERO:      the duration of the deassert gap after a pulse
                     that signifies a "0" data bit
      SIG_ONE:       the duration of the deassert gap after a pulse
                     that signifies a "1" data bit
      SIG_IM_GAP:    the duration of the deassert for the
                     inter-message gap, between transmissions
      SIG_PULSE:     the duration of the "pulse" assert voltage level
*/

// Enumerate the possible signal duration types here for use as indices
// Append additional signal timings here and then initialize them
// in the device initiation of the "signals" array
// Maintain the order here in the initialization code in the device class
enum SIGNAL_T {NONE=-1, SIG_SYNC=0, SIG_SYNC_GAP, SIG_ZERO, SIG_ONE,
	       SIG_IM_GAP, SIG_PULSE};

// Structure of the table of timing durations used to signal
typedef struct {
  SIGNAL_T   sig_type;    // Index the type of signal
  string     sig_name;    // Provide a brief descriptive name
  uint16_t   up_time;     // duration for pulse with voltage up
  uint16_t   delay_time;  // delay with voltage down before next signal
} SIGNAL;

/* ISM_Device is the base class descriptor for creating transmissions
   compatible with various other specific OOK-PWM devices.  It contains
   the list of signals for the transmitter driver, the procedure needed
   to insert signals into the list, and the procedure to play the signals
   through the transmitter.
*/
class ISM_Device {

public:
    
  // These are used by the device object procedures
  //   to process the waveform description into a command list
  SIGNAL_T cmdList[640];
  uint16_t listEnd=0;
  string   Device_Name="ISM Device";
  uint8_t  Lo=0;
  uint8_t  Hi=1;
  SIGNAL*  signals;

  ISM_Device() {
  };

  // Inserts a signal into the commmand list
  void insert(SIGNAL_T signal) {
      cmdList[listEnd++] = signal;
      return;
  };

  // Plays back the signaling commands in the device's cmdList[]
  void playback() {
    cout << "\nFor device " << Device_Name << " the wave generated is:" << endl;
    for (int i = 0; i<listEnd; i++) {
      if (cmdList[i] == NONE) { // Terminates list but should never be executed
	  cout << " \tERROR -- invalid NULL command at cmdList index i=" << i << endl;
	  return;
      };
      cout << setw(3) << right << i << ": Cmd= " << setw(2) << right << (int) cmdList[i];
      cout << setw(9) << "\tASSERT"  << setw(6) << right << signals[cmdList[i]].up_time;
      cout << setw(9) << "\tDEASSERT"<< setw(6) << right << signals[cmdList[i]].delay_time;
      cout << "\t" << signals[cmdList[i]].sig_name << endl;
    };
  };
};

class TX141TH : public ISM_Device {
  public:

  // TX141 timing durations
  // Taken from rtl_433 distribution, lacrosse/04/gfile1.cu8
  // Processed with `rtl_433 -A -r` and viewed as http 
  int sigLen = 6;
  SIGNAL TX141TH_signals[6] = {
    {SIG_SYNC,     "Sync",       712,    936},
    {SIG_SYNC_GAP, "Sync-gap",     0,    168},
    {SIG_ZERO,     "Zero",       348,    348},
    {SIG_ONE,      "One",        136,    544},
    {SIG_IM_GAP,   "IM_gap",       0,  10004},
    {SIG_PULSE,    "Pulse",      512,    512}   //spare
  };

  // Instantiate the device by linking 'signals' to our device timing
  TX141TH() {
    Device_Name = "Lacrosse TX141TH-BV2";
    signals = TX141TH_signals;
    cout << "Created device " << Device_Name << endl;
  };

  // From rtl_433/src/bit_utils.c
  uint8_t lfsr_digest8(uint8_t const message[], unsigned bytes, uint8_t gen, uint8_t key)
  {
      uint8_t sum = 0;
      for (unsigned k = 0; k < bytes; ++k) {
	  uint8_t data = message[k];
	  for (int i = 7; i >= 0; --i) {
	      // fprintf(stderr, "key is %02x\n", key);
	      // XOR key into sum if data bit is set
	      if ((data >> i) & 1)
		  sum ^= key;

	      // roll the key right (actually the lsb is dropped here)
	      // and apply the gen (needs to include the dropped lsb as msb)
	      if (key & 1)
		  key = (key >> 1) ^ gen;
	      else
		  key = (key >> 1);
	  }
      }
      return sum;
  }

  uint8_t lfsr_digest8_reflect(uint8_t message[], int bytes, uint8_t gen, uint8_t key)
  {
    uint8_t sum = 0;
    // Process message from last byte to first byte (reflected)
    for (int k = bytes - 1; k >= 0; --k) {
        uint8_t data = message[k];
        // Process individual bits of each byte (reflected)
        for (int i = 0; i < 8; ++i) {
            if ((data >> i) & 1) {
                sum ^= key;
            }

            // roll the key left (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 0x80)
                key = (key << 1) ^ gen;
            else
                key = (key << 1);
        }
    }
    return sum;
}
  
uint8_t crc8(uint8_t message[], uint8_t nBytes, uint8_t polynomial, uint8_t init) {
    uint8_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte) {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ polynomial;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

  // Routine to create 40-bit TX141TH datagrams
  // Pack <ID, Status, Temp, Humidity> into a 5-byte TX141TH message with 1 checksum byte
  void pack_msg(uint8_t I, uint8_t S, int16_t T, uint8_t H, uint8_t *msg) {
    uint8_t tmsg[5];
    msg[0] = ( I&0xff );
    uint16_t TT = T+500;
    msg[1] = ( (S&0x0f)<<4 | (TT>>8)&0x0f );
    msg[2] = ( TT&0xff );
    msg[3] = ( H&0xff );
    msg[4] = lfsr_digest8_reflect(msg, 4, 0x31, 0xf4);
    // Finally, invert the message bits
    for (int i=0; i<5; i++) msg[i] = ~msg[i];
    return;
    };  // end pack_msg()

  // unpack_msg <ID, Status, Temp, Humidity> from a 5-byte AR609 message with 1 checksum byte
  void unpack_msg(uint8_t *msg, uint8_t &I, uint8_t &S, int16_t &T, uint8_t &H) {
    if (msg[4] != lfsr_digest8_reflect(msg, 4, 0x31, 0xf4) ) {
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
      T =  ( ( (int16_t) ( ( msg[1]&0x0f ) << 12 | ( msg[2]) << 4 ) ) >> 4 ) - 500; 
      H = msg[3];
      };
    return;
    };  // end unpack_msg()

  // This creates the TH141-BV2 waveform description
  void make_wave(uint8_t *msg, uint8_t msgLen) {
    listEnd = 0;

    // Repeat preamble, data, and sig_sync 12 times
    // Preamble
    insert(SIG_SYNC);
    insert(SIG_SYNC);
    insert(SIG_SYNC);
    insert(SIG_SYNC);

    // The data packet
    cout << "The msg packet, length=" << (int) msgLen << ", as a series of bits: ";
    for (int i=0; i<msgLen; i++) {
      insert( ( (uint8_t) ((msg[i/8]>>(7-(i%8))) & 0x01 ) ) == 0 ? SIG_ZERO : SIG_ONE );
      cout << setw(1) << ((msg[i/8]>>(7-(i%8)))&0x01);
    };
    // May need to insert additional gap (no pulse) before next repetition
    // insert(SIG_SYNC_GAP);
    cout << endl;

    // Postamble of two SIG_SYNCs and IM_GAP
    // and terminal marker for safety
    cmdList[listEnd++] = SIG_SYNC;
    cmdList[listEnd++] = SIG_SYNC;
    cmdList[listEnd++] = SIG_IM_GAP;
    cmdList[listEnd]   = NONE;
  };
};


int main() {
  // In the absence of a real sensor provide values needed
  uint8_t id =     199;
  uint8_t status =   1;
  int16_t temp =    20;
  uint8_t hum =     50;
  uint8_t i, s, h;
  int16_t t;

  // AR609 and TX141TH-BV2 messages are 40 bits = 5 bytes
  uint8_t msg[8] = {0};;
  uint8_t msgLen = 40;
  cout << "\nGenerate an ISM-device waveform and play back as pulse cycles\n" << endl;

  cout << "Create 'tx' as a TX141TH-BV2" << endl;
  TX141TH tx;
  
  id     = 67;
  status = 0;
  temp   = (int16_t) (9.3*10.0);
  hum    = 73;

  cout << "\nNow create a wave for 'tx'" << endl;
  tx.pack_msg(id, status, temp, hum, msg);
  cout << "Device ar message in hex: 0x ";
  tx.unpack_msg(msg, i, s, t, h);
  for (int i=0; i<5; i++)
    cout << setw(2) << setfill('0') << hex << (int) msg[i] << " ";
  cout << dec << setfill(' ') << endl;
  cout << "raw t = " << (int) t << endl;
  cout << "Message values: id = " << (int) i << "; status = " << (int) s
       << "; temp = " << setprecision(3) << ((float) t)/10.0 << "; humidity = " << (int) h << endl;
  tx.make_wave(msg, msgLen);

  cout << "\nPlay back tx" << endl;
  tx.playback();

  return 0;
};
