/* -*- c++ -*-
  Lacrosse WS7000-20 temp/humid/press emulator
  
  ISM_Emulator: Emulate an ISM-band remote sensor on an Arduinio Uno
  This version specifically emulates an Acurite 609TXC temperature/humidity
  sensor, but the program provides a class for emulating other devices.

  This program uses a 433MHz transmitter and Arduino (or similar device
  supported on the Arduino IDE) to send temperature/humidity
  readings using the Acurite 609TXC protocol.  See the src/device/acurite.c
  file in the rtl_433 distribution (https://github.com/merbanan/rtl_433)
  for details about the packet format.  The data packet format created
  here matches the format recognized by rtl_433 for the Acurite 609TXC.

  The Lacrosse WS7000-20 remote sensor transmits temperature,
  humidity, and barometric pressure readings. A transmission frame
  is 81 bits long (for the -20 model).  A "0" bit is 800us high
  followed by 400us low and a "1" bit is 400us high followed
  by 800us low.

  The frame begins with 10 "0" bits followed by 1 "1" bit.
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
  
  The BME68x code for reading temp/press/hum/VOC was adapted from
  the Adafruit demo program http://www.adafruit.com/products/3660
  The BME68x temperature reading may need calibration against
  an external thermometer.  The DEFINEd parameter 'BME_TEMP_OFFSET'
  (below) can be used to perform an adjustment, if needed.

  hdtodd@gmail.com, 2025.01.13
*/

#define DEBUG           // SET TO #undef to disable execution trace

//#define DELAY 29350   // Time between messages in ms, less 650ms for overhead on SAMD21
#define DELAY 4350    // Time between messages in ms, less 650ms for overhead on SAMD21

#ifdef DEBUG
#define DBG_begin(...)    SerialUSB.begin(__VA_ARGS__);
#define DBG_print(...)    SerialUSB.print(__VA_ARGS__)
#define DBG_write(...)    SerialUSB.write(__VA_ARGS__)
#define DBG_println(...)  SerialUSB.println(__VA_ARGS__)
#else
#define DBG_begin(...)
#define DBG_print(...)
#define DBG_write(...)
#define DBG_println(...)
#endif

#define SENSOR BME

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// 433MHz transmitter settings
#define TX           4      // transmit data line connected to SAMD21 GPIO 4
#define LED         13      // LED active on GPIO 13 when transmitting
#define REPEATS      1      // Number of times to repeat packet in one transmission

// BME688 settings for Adafruit I2C connection
#define BME_SCK  13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS   10

#define SEALEVELPRESSURE_HPA (1013.25)    // Standard conversion constant
#define BME_TEMP_OFFSET (-1.7/1.8)        // Calibration offset in Fahrenheit

// Print in 2-char hex the message in "buf" of length "len"                                                                        
void hex_print(uint8_t *buf, int len) {
  static const char CH[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                            '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  for (int i = 0; i<len; i++) {
    DBG_print(CH[buf[i]>>4]);
    DBG_print(CH[buf[i]&0x0f]);
    DBG_print(' ');
    };
  return;
};

/*
    "SIGNAL_T" enumerates the possible signal types.  Each signal has a
    type (index), name, duration of asserted signal high), and duration of
    deasserted signal (low).  Durations are in microseconds.  Either or
    both duration may be 0, in which case the signal voltge won't be changed.

    This list may be extended with additional types in the future, but for
    the Acurite 609 for which this code is the prototype, we use:
      SIG_SYNC:      the duration of the "sync" deassert voltage level
      SIG_SYNC_GAP:  the duration of the "sync_gap" deassert before data bits
      SIG_ZERO:      the duration of the deassert gap after a pulse
                     that signifies a "0" data bit
      SIG_ONE:       the duration of the deassert gap after a pulse
                     that signifies a "1" data bit
      SIG_IM_GAP:    the duration of the deassert for the
                     inter-message gap, between transmissions
      SIG_PULSE:     the duration of the "pulse" assert voltage level
                     (spare, as a future contingency)

    Enumerate the possible signal duration types here for use as indices
    Append additional signal timings here and then initialize them
    in the device initiation of the "signals" array
    Maintain the order here in the initialization code in the device class
*/
enum SIGNAL_T {NONE=-1, SIG_SYNC=0, SIG_SYNC_GAP, SIG_ZERO, SIG_ONE,
	       SIG_IM_GAP, SIG_PULSE};

// Structure of the table of timing durations used to signal
typedef struct {
  SIGNAL_T   sig_type;    // Index the type of signal
  String     sig_name;    // Provide a brief descriptive name
  uint16_t   up_time;     // duration for pulse with voltage up
  uint16_t   delay_time;  // delay with voltage down before next signal
} SIGNAL;

bool INVERT=false;        //Pulse direction: INVERT==false ==> Hi = 3v3 or 5v
int Hi, Lo;		  // voltages for pulses (may be inverted)

/* ISM_Device is the base class descriptor for various specific OOK-PWM
   emulators.  It contains the list of signals for the transmitter driver,
   the procedure needed to insert signals into the list, and the procedure
   to play the list of signals 
*/
class ISM_Device {
public:
    
  // These are used by the device object procedures
  //   to process the waveform description into a command list
  SIGNAL_T cmdList[640];
  uint16_t listEnd=0;
  String   Device_Name="ISM Device";
  SIGNAL*  signals;
  
  ISM_Device() {
  };

  // Inserts a signal into the commmand list
  void insert(SIGNAL_T signal) {
      cmdList[listEnd++] = signal;
      return;
  };

  void playback() {
    SIGNAL_T sig;
    for (int i = 0; i<listEnd; i++) {
      sig = cmdList[i];
      if (sig == NONE) { // Terminates list but should never be executed
	  DBG_print(" \tERROR -- invalid signal: "); DBG_print( (int) cmdList[i] );
	  DBG_print( (cmdList[i] == NONE) ? " (NONE)" : "" );
	  return;
      };
      if (signals[sig].up_time > 0) {
        digitalWrite(TX,Hi);
	delayMicroseconds(signals[sig].up_time);
      };
    if (signals[sig].delay_time > 0) {
        digitalWrite(TX,Lo);
	delayMicroseconds(signals[sig].delay_time);
      };
    };
  };  // end playback()
};    // end class ISM_device
    
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
    uint8_t xcheck = 0;
    uint16_t checksum = 0;
    uint8_t sign = (temp<0) ? 1 : 0;
    temp = (temp < 0) ? -temp : temp;
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
      DBG_println("CheckXOR failed: invalid message");
      return;
      };
    if ((msg[6]& 0x0F) != (uint8_t) ((checksum + xcheck + CONST5) & 0x0F) ) {
      DBG_println("CheckSum failed: invalid message");
      return;
      };
    if ( (msg[0]>>4) != TYPE4) {
      DBG_print("Invalid message packet type ");
      DBG_print( (int) (msg[0]>>4) );
      DBG_println(" is not a WS7000-20 message");
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
    DBG_print("Created device "); DBG_println(Device_Name);
  };

  void make_wave(uint8_t *msg, uint8_t msgLen) {
    listEnd = 0;
    // Preamble
    for (uint8_t i=0; i<10; i++) insert(SIG_ZERO);
    insert(SIG_ONE);
    
    // The data packet
    DBG_print("The msg packet, length="); DBG_print( (int)msgLen);
    DBG_print(", as a series of bits: ");
    for (uint8_t i=0; i<msgLen; i++) {
      insert( ( (uint8_t) ((msg[i/8]>>(7-(i%8))) & 0x01 ) ) == 0 ? SIG_ZERO : SIG_ONE );
      DBG_print(((msg[i/8]>>(7-(i%8)))&0x01));
      if ( ((i+1)%4)==0 ) insert(SIG_ONE);
    };
    DBG_println();
    // Postamble and terminal marker for safety
    insert(SIG_IM_GAP);
    cmdList[listEnd] = NONE;
  };
};  // End class WS7000 

class AR609 : public ISM_Device {
  public:

  // AR609 timing durations
  int sigLen = 6;
  SIGNAL AR609_signals[6] = {
    {SIG_SYNC,     "Sync",       512,    480},
    {SIG_SYNC_GAP, "Sync-gap",   512,   8912},
    {SIG_ZERO,     "Zero",       512,    976},
    {SIG_ONE,      "One",        512,   1968},
    {SIG_IM_GAP,   "IM_gap",     512,  10200},
    {SIG_PULSE,    "Pulse",      512,    512}   //spare
  };

  // Instantiate the device by linking 'signals' to our device timing
  AR609() {
    Device_Name = "Acurite 609TXC";
    signals = AR609_signals;
    DBG_print("Created device "); DBG_println(Device_Name);
  };

  // Routine to create 40-bit AR609 datagrams
  // Pack <ID, Status, Temp, Humidity> into a 5-byte AR609 message with 1 checksum byte
  void pack_msg(uint8_t I, uint8_t S, int16_t T, uint8_t H, uint8_t *msg) {
    msg[0] = ( I&0xff );
    msg[1] = ( (S&0x0f)<<4 | (T>>8)&0x0f );
    msg[2] = ( T&0xff );
    msg[3] = ( H&0xff ); 
    msg[4] = ( msg[0] + msg[1] + msg[2] + msg[3] ) & 0xff;
    return;
    };  // end pack_msg()

  // unpack_msg <ID, Status, Temp, Humidity> from a 5-byte AR609 message with 1 checksum byte
  void unpack_msg(uint8_t *msg, uint8_t &I, uint8_t &S, int16_t &T, uint8_t &H) {
    if (msg[4] != ( (msg[0]+msg[1]+msg[2]+msg[3])&0xff) ) {
      DBG_println("Invalid message packet: Checksum error");
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
  };  // end unpack_msg()

  // This creates the Acurite 609THC waveform description
  void make_wave(uint8_t *msg, uint8_t msgLen) {
    listEnd = 0;

    // Preamble
    insert(SIG_SYNC);
    insert(SIG_SYNC);
    insert(SIG_SYNC_GAP);

    // The data packet
    DBG_print("The msg packet, length="); DBG_print( (int)msgLen);
    DBG_print(", as a series of bits: ");
    for (int i=0; i<msgLen; i++) {
      insert( ( (uint8_t) ((msg[i/8]>>(7-(i%8))) & 0x01 ) ) == 0 ? SIG_ZERO : SIG_ONE );
      DBG_print(((msg[i/8]>>(7-(i%8)))&0x01));
    };
    DBG_println();

    // Postamble and terminal marker for safety
    insert(SIG_IM_GAP);
    cmdList[listEnd] = NONE;
  };
};

// Create BME object
Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

uint8_t msg[20];	  // should not have messages > 20 bytes = 160 bits
int temp, hum;		  // temperature & humidity values
int WS7000Len = 56; 	  // WS7000-20 messages are 56 bits long
int count=0;  	       	  // Count the packets sent

void setup(void) {
  DBG_begin(9600);
  delay(2000);
  DBG_println("\nAR609 Starting test transmission");
  if (!INVERT) {Hi = HIGH; Lo = LOW; }
  else         {Hi = LOW;  Lo = HIGH;};
  pinMode(TX, OUTPUT);
  digitalWrite(TX,LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  if (!bme.begin()) {
    DBG_println("Could not find a valid BME680 sensor, check wiring!");
    while (!bme.begin());
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
};  // End setup()

void loop(void) {
  uint8_t id =       3;
  int16_t temp =  -254;  // Temp(C) * 10
  uint16_t hum =   479;  // Humidity * 10
  uint16_t press = 9950;  // Pressure in hPa * 10
  uint16_t voc;
  uint8_t i;
  uint16_t p, h;
  int16_t t;

  // WS7000-20 messages are 7 bytes
  uint8_t msg[7] = {0};
  uint8_t msgLen = 56;
  WS7000  ws;

  // Get the readings from the BME688
  if (! bme.performReading()) {
    DBG_println("BME688 failed to perform reading :-(");
    return;
  }

  // Unpack BME readings, repack for ISM transmission, and create the waveform
  temp = (uint16_t) ((bme.temperature+0.05 + BME_TEMP_OFFSET) * 10); // adjust & round
  hum = (uint16_t) ((bme.humidity+0.5)*10.0);              // round 
  press = (uint16_t) (bme.pressure / 10.0);        // hPa * 10
  voc = (uint16_t) (bme.gas_resistance/1000.0) ;   //KOhms
  ws.pack_msg(id, temp, hum, press, msg);
  ws.make_wave(msg,WS7000Len);
  
  // Set up for transmission
  digitalWrite(TX,Lo);

  DBG_print("Transmit msg "); DBG_print(++count); 
  DBG_print("\tT=");          DBG_print(bme.temperature);     DBG_print("˚C");
  DBG_print(", H=");          DBG_print(bme.humidity);        DBG_print("%");
  DBG_print(", P=");          DBG_print(bme.pressure/100.0);  DBG_print("hPa");
  DBG_print(", V=");          DBG_print(bme.gas_resistance/1000.0);
  DBG_print(": 0x ");
  hex_print(msg,7);
  DBG_println();

  // Messages must be sent as a continual stream, with no
  //   program breaks once transmission begins
  // So the data-transmit loop is a single loop 
  // Repeat packet 'REPEATS' times in this transmission

  digitalWrite(LED,Hi);
  for (int j=0; j<REPEATS;j++)  ws.playback();

  // Done transmitting; turn everything off & delete object
  digitalWrite(LED,LOW);
  digitalWrite(TX,LOW);
  ws.~WS7000();
  delay(DELAY);
}; // end loop()
