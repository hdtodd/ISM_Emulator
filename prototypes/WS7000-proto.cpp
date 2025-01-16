/* g++ -std=c++11
  WS7000_proto.cpp: Prototype for generating WS7000-compatible waveforms

  This program models and tests code to describe and generate
  waveform specifications on Arduino/Sparkfun devices.  The
  waveform specifications can be adapted to modulate ISM-band
  (e.g, 433MHz) on-off keying and pulse-width modulation (OOK/PWM) 
  transmitters to generate waveforms compatible with ISM-band
  remote sensing devices.

  The waveform is a series of up/down voltages (cycles) that turn
  the ISM transmitter on/off (OOK) followed by timing gaps of
  various durations.  The duration indicates the type of signal
  (PWM -- pulse-width modulation).

  The WS7000 transmission frame is 81 bits long (for the -20 model).
  A "0" bit is 800us high followed by 400us low and a "1" bit is
  400us high followed by 800us low.

  The WS7000-20 frame begins with 10 "0" bits followed by 1 "1" bit.
  The data follow that preamble as 14 4-bit nibbles, separated
  from each other by a "1" bit, and each nibble has the
  least-significant bit first.  Soo the data values must have
  their bit patterns reversed as they're inserted into the frame.
  The data are BCD-encoded values representing the decimal digits
  of each of the three readings.
  
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
  
  hdtodd@gmail.com, 2025.01.13
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

class WS7000 : public ISM_Device {

public:

  // WS7000-20 timing durations
  // Name, description, spec'd delay in us, place for adjusted duration
  int sigLen = 6;
  SIGNAL WS7000_signals[6] = {
    {SIG_SYNC,     "Sync",         0,      0},
    {SIG_SYNC_GAP, "Sync-gap",     0,      0},
    {SIG_ZERO,     "Zero",       800,    400},
    {SIG_ONE,      "One",        400,    800},
    {SIG_IM_GAP,   "IM_gap",       0,  10004},
    {SIG_PULSE,    "Pulse",        0,      0}   //spare
  };

  uint8_t reflect4(uint8_t x) {
    x = (x & 0xCC) >> 2 | (x & 0x33) << 2;
    x = (x & 0xAA) >> 1 | (x & 0x55) << 1;
    return x;
  };

  void reflect_nibbles(uint8_t message[], unsigned num_bytes) {
      for (uint8_t i = 0; i < num_bytes; ++i) message[i] = reflect4(message[i]);
  };
  
  /* Routines to create 56-bit WS7000-20 datagrams from sensor data
     Pack <ID, Temp, Humidity, Press> into an 11-nibble
       datagram prepended by a 4-bit nibble with pre-defined datagram
       type (4) identifying this as a WS7000-20 datagram and appended
       with 1 CheckXOR and 1 CheckSum nibble (14 nibbles = 7 bytes total).
       Bits within each nibble are reversed so that they will be 
       transmitted least-significant-bit (lsb) first.

       Inputs:
       <ID> is a 3-bit sensor address
       <temp> is a signed 16-bit value of 10*(temperature reading)
       <hum> is an unsigned 8-bit value representing the relative humidity as integer
       <press> is a 16-bit unsigned integer representing barometric pressure
               in hPa*100
       <msg>   is an array of at least 7 unsigned 8-bit integers

       Output in "msg"(as nibbles, least-significant-bit first))
       1.  Datagram type code = 4 for WS7000-20
       2.  <sign bit for temp> <3-bit ID, or address, code>
       3.  Tenths of a degree from temperature reading, in BCD
       4.  Units of a degree from temperature reading, in BCD
       5.  Tens of degrees from temperature reading, in BCD
       6.  Tenths of a percent of relative humidity, in BCD
       7.  Units of percent of relative humidity, in BCD
       8.  Tens of percent of relative humidity, in BCD
       9.  Units of hPa of barometric pressure, in BCD
       10. Tens of hPa of barometric pressure, in BCD
       11. Hundreds of hPA of barometric pressure, in BCD
       12. Offset in hundreds of hPa for barometric pressure, in BCD
       13. XOR of the 12 data nibbles
       14. Sum of the 12 data nibbles + XOR nibbles + 5, ANDed with 0x0F
  */
  void pack_msg(uint8_t id, int16_t temp, uint16_t hum,
		uint16_t press, uint8_t *msg) {
    uint8_t CONST5 = 5;      // Lacrosse CheckSum factor
    uint8_t p_offset = 200;  // Lacrosse pressure offset
    uint8_t sign = (temp<0) ? 1 : 0;
    uint8_t xcheck = 0;
    uint16_t checksum = 0;
    temp = abs(temp);
    // Store input binary values as BCD digit nibbles in message frame
    msg[0] = (uint8_t) (0x40 | (uint8_t)(sign<<3 | id&0x07)); 
    msg[1] = (temp % 10)<<4 | (temp/10) % 10;
    msg[2] = ((temp/100) % 10)<<4 | (hum % 10);   //(no 10ths of RH)
    msg[3] = ((hum/10) % 10)<<4 | (hum/100) % 10;
    msg[4] = ((press-p_offset) % 10)<<4 | (((press-p_offset)/10) % 10);
    msg[5] = (((press-p_offset)/100) % 10)<<4 | 0x02;
    // CheckXOR nibbles 0-5 and CheckSUM nibbles 0-6 and combine in nibble 7
    for (uint8_t j=0;j<6;j++) {
      xcheck ^= (msg[j]>>4);
      xcheck ^= (msg[j]&0x0F);
      checksum += (msg[j]>>4);
      checksum += (msg[j]&0x0F);
      };
    msg[6] =  (xcheck<<4);
    msg[6] |= (uint8_t) ((checksum + xcheck + CONST5) & 0x0F);
    // And, finally, reflect each nibble to be lsb first
    reflect_nibbles(msg,8);
    return;
  };

  // Unpack <ID, Temp, Humidity, Pressure> from a 7-byte WS7000 message with
  //   CheckXOR and CheckSum nibbles
    void unpack_msg(uint8_t *msg, uint8_t &I, int16_t &T, uint16_t &H,
		    uint16_t &P) {
    uint8_t CONST5 = 5;      // Lacrosse CheckSum factor
    uint8_t TYPE4  = 4;      // Lacrosse WS7000-20 message type
    uint8_t xcheck = 0;
    uint16_t checksum = 0;
    uint8_t sign;
    // Initialize return values in case of errors
    I = 0;
    T = 0;
    H = 0;
    P = 0;
    // Start by reversing bits, so lsb is last in each nibble
    reflect_nibbles(msg,8);
    // Now compute the CheckXOR and CheckSum from the
    //   message frame
    for (uint8_t j=0;j<6;j++) {
      xcheck ^= (msg[j]>>4);
      xcheck ^= (msg[j]&0x0F);
      checksum += (msg[j]>>4);
      checksum += (msg[j]&0x0F);
      };
    if ( (msg[6]>>4) !=  (xcheck&0x0F) ) {
      cout << "CheckXOR failed: invalid message" << endl;
      return;
      };
    if ((msg[6]& 0x0F) != (uint8_t) ((checksum + xcheck + CONST5) & 0x0F) ) {
      cout << "CheckSum failed: invalid message" << endl;
      return;
      };
    if ( (msg[0]>>4) != TYPE4) {
      cout << "Invalid message packet " << (int) (msg[0]>>4) << " not a WS7000-20" << endl;
      return;
      };
    // Capture sign for temperature
    sign = ( (msg[0] & 0x08) != 0 ) ? 1 : 0;
    // Get the device id/address
    I = (uint8_t) (msg[0] & 0x07);
    T = (int16_t) ( ( msg[1]>>4 ) + ( (msg[1]&0x0f)*10 ) + (msg[2]>>4)*100); 
    if (sign != 0) T = -T;
    H = (int16_t) ( ( msg[2]&0x0f ) + ( (msg[3]>>4)*10 ) + (msg[3]&0x0f)*100); 
    P = (int16_t) ( ( msg[4]>>4 ) + ( (msg[4]&0x0f)*10 ) + (msg[5]>>4)*100)
      + (msg[5]&0x0F)*100;; 
    return;
    };

  // Instantiate the device by linking 'signals' to our device timing
  WS7000() {
    Device_Name = "Lacrosse WS7000-20";
    signals = WS7000_signals;
    listEnd = 0;
    cout << "Created device " << Device_Name << endl;
  };

  void make_wave(uint8_t *msg, uint8_t msgLen) {
    listEnd = 0;
    // Preamble
    for (uint8_t i=0; i<10; i++) insert(SIG_ZERO);
    insert(SIG_ONE);
    
    // The data packet
    cout << "The msg packet, length=" << (int) msgLen << ", as a series of bits: ";
    for (uint8_t i=0; i<msgLen; i++) {
      insert( ( (uint8_t) ((msg[i/8]>>(7-(i%8))) & 0x01 ) ) == 0 ? SIG_ZERO : SIG_ONE );
      cout << setw(1) << ((msg[i/8]>>(7-(i%8)))&0x01);
      if ( ((i+1)%4)==0 ) insert(SIG_ONE);
    };
    cout << endl;
    // Postamble and terminal marker for safety
    insert(SIG_IM_GAP);
    cmdList[listEnd] = NONE;
  };
};  // End class WS7000 

int main() {
  // In the absence of a real sensor provide values needed
  uint8_t id =       7;
  int16_t temp =  -254;  // Temp(C) * 10
  uint16_t hum =   479;  // Humidity * 10
  uint16_t press = 995;  // Pressure in hPa
  uint8_t i;
  uint16_t p, h;
  int16_t t;

  // WS7000-20 messages are 7 bytes
  uint8_t msg[7] = {0};
  uint8_t msgLen = 56;
  cout << "\nGenerate an ISM-device waveform and play back as pulse cycles\n" << endl;

  cout << "Create 'ws' as a WS7000-20" << endl;
  WS7000  ws;
  
  cout << "\nNow create a wave for 'ws'" << endl;
  ws.pack_msg(id, temp, hum, press, msg);
  cout << "Device ws message in hex: 0x ";
  ws.unpack_msg(msg, i, t, h, p);
  for (int j=0; j<7; j++)
    cout << setw(2) << setfill('0') << hex << (int) msg[j] << " ";
  cout << dec << setfill(' ') << endl;
  cout << "Message values: id = " << (int) i 
       << "; temp = " << ( (float) t)/10.0 << "C"
       << "; humidity = " << ( (float) h)/10.0 << "%"
       << "; pressure = " << ( (float) p) << "hPa" << endl;
  ws.make_wave(msg, msgLen);

  cout << "\nPlay back ws" << endl;
  ws.playback();
  return 0;
};
