/* -*- c++ -*-
  omni.ino:  omni multiprotocol ISM-band sensor transmission 

  This program uses a 433MHz transmitter and Arduino (or similar device
  supported on the Arduino IDE) to send temperature/humidity
  readings in a format compatible with the Acurite 609TXC protocol.
  See the src/device/lacrosse_ws7000.c file in the rtl_433 distribution
  (https://github.com/merbanan/rtl_433) for details about the packet
  format.  The data packet format created here matches the format
  recognized by rtl_433 for the Lacrosse WS7000-20.

  This program executes on the Arduino Uno R3 but is constrained by the
  limited 2K memory limit for variables.  Take care in modifying the
  program as changes in memory requirements may cause execution errors.

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
  was correctly received).  This program sends just one message
  per transmission.

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

//#define DELAY 29165   // Time between messages in ms, less 835ms for overhead on SAMD21
#define DELAY 4165 // Time between messages in ms, less 835ms for overhead on SAMD21

// Couldn't find the IDE macro that says if serial port is Serial or SerialUSB
// So try this; if it doesn't work, specify which
#ifdef ARDUINO_ARCH_SAMD
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif

#define DEBUG // SET TO #undef to disable execution trace

#ifdef DEBUG
#define DBG_begin(...)   SERIAL.begin(__VA_ARGS__);
#define DBG_print(...)   SERIAL.print(__VA_ARGS__)
#define DBG_write(...)   SERIAL.write(__VA_ARGS__)
#define DBG_println(...) SERIAL.println(__VA_ARGS__)
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
/*  libcrc8.c
    From http://github.com/hdtodd/CRC8-Library
    HD Todd, February, 2022

    Predefined table of CRC-8 lookup bytes computed using
    the polynomial 0x97, also know as "C2". Per Koopman,
    "arguably the best" for messages up to 119 bits long

   This table can be recomputed for a different polynomial
   See original library distribution on github
*/
uint8_t CRC8POLY = 0x97;
uint8_t CRC8Table[256] = {
	0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb, 
	0x5d, 0xca, 0xe4, 0x73, 0xb8, 0x2f, 0x01, 0x96, 
	0xba, 0x2d, 0x03, 0x94, 0x5f, 0xc8, 0xe6, 0x71, 
	0xe7, 0x70, 0x5e, 0xc9, 0x02, 0x95, 0xbb, 0x2c, 
	0xe3, 0x74, 0x5a, 0xcd, 0x06, 0x91, 0xbf, 0x28, 
	0xbe, 0x29, 0x07, 0x90, 0x5b, 0xcc, 0xe2, 0x75, 
	0x59, 0xce, 0xe0, 0x77, 0xbc, 0x2b, 0x05, 0x92, 
	0x04, 0x93, 0xbd, 0x2a, 0xe1, 0x76, 0x58, 0xcf, 
	0x51, 0xc6, 0xe8, 0x7f, 0xb4, 0x23, 0x0d, 0x9a, 
	0x0c, 0x9b, 0xb5, 0x22, 0xe9, 0x7e, 0x50, 0xc7, 
	0xeb, 0x7c, 0x52, 0xc5, 0x0e, 0x99, 0xb7, 0x20, 
	0xb6, 0x21, 0x0f, 0x98, 0x53, 0xc4, 0xea, 0x7d, 
	0xb2, 0x25, 0x0b, 0x9c, 0x57, 0xc0, 0xee, 0x79, 
	0xef, 0x78, 0x56, 0xc1, 0x0a, 0x9d, 0xb3, 0x24, 
	0x08, 0x9f, 0xb1, 0x26, 0xed, 0x7a, 0x54, 0xc3, 
	0x55, 0xc2, 0xec, 0x7b, 0xb0, 0x27, 0x09, 0x9e, 
	0xa2, 0x35, 0x1b, 0x8c, 0x47, 0xd0, 0xfe, 0x69, 
	0xff, 0x68, 0x46, 0xd1, 0x1a, 0x8d, 0xa3, 0x34, 
	0x18, 0x8f, 0xa1, 0x36, 0xfd, 0x6a, 0x44, 0xd3, 
	0x45, 0xd2, 0xfc, 0x6b, 0xa0, 0x37, 0x19, 0x8e, 
	0x41, 0xd6, 0xf8, 0x6f, 0xa4, 0x33, 0x1d, 0x8a, 
	0x1c, 0x8b, 0xa5, 0x32, 0xf9, 0x6e, 0x40, 0xd7, 
	0xfb, 0x6c, 0x42, 0xd5, 0x1e, 0x89, 0xa7, 0x30, 
	0xa6, 0x31, 0x1f, 0x88, 0x43, 0xd4, 0xfa, 0x6d, 
	0xf3, 0x64, 0x4a, 0xdd, 0x16, 0x81, 0xaf, 0x38, 
	0xae, 0x39, 0x17, 0x80, 0x4b, 0xdc, 0xf2, 0x65, 
	0x49, 0xde, 0xf0, 0x67, 0xac, 0x3b, 0x15, 0x82, 
	0x14, 0x83, 0xad, 0x3a, 0xf1, 0x66, 0x48, 0xdf, 
	0x10, 0x87, 0xa9, 0x3e, 0xf5, 0x62, 0x4c, 0xdb, 
	0x4d, 0xda, 0xf4, 0x63, 0xa8, 0x3f, 0x11, 0x86, 
	0xaa, 0x3d, 0x13, 0x84, 0x4f, 0xd8, 0xf6, 0x61, 
	0xf7, 0x60, 0x4e, 0xd9, 0x12, 0x85, 0xab, 0x3c
};

uint8_t crc8(uint8_t *msg, int lengthOfMsg, uint8_t init) {
  while (lengthOfMsg-- > 0) init = CRC8Table[ (init ^ *msg++)];
  return init;
};

// 433MHz transmitter settings
#ifdef PICO_RP2350
#define TX  3           // transmit data line connected to Pico 2 GPIO 3
#define LED LED_BUILTIN // LED active on GPIO 25 when transmitting
#else
#define TX  4  // transmit data line connected to SAMD21 GPIO 4
#define LED 13 // LED active on GPIO 13 when transmitting
#endif
#define REPEATS 1 // Number of times to repeat packet in one transmission

// BME688 settings for Adafruit I2C connection
#define BME_SCK              13
#define BME_MISO             12
#define BME_MOSI             11
#define BME_CS               10

#define SEALEVELPRESSURE_HPA (1013.25)    // Standard conversion constant
#define BME_TEMP_OFFSET      (-1.7 / 1.8) // Calibration offset in Fahrenheit

/* "SIGNAL_T" are the possible signal types.  Each signal has a
    type (index), name, duration of asserted signal high), and duration of
    deasserted signal (low).  Durations are in microseconds.  Either or
    both duration may be 0, in which case the signal voltage won't be changed.

    For the Lacrosse WS7000-20:
      SIG_SYNC:      the durations of the pulse and "sync" deassert voltage level
      SIG_SYNC_GAP:  the duration of the pulse and"sync_gap" deassert before data bits
      SIG_ZERO:      the duration of the pulse and deassert gap that signify a "0" data bit
      SIG_ONE:       the duration of the pulse and deassert gap that signify a "1" data bit
      SIG_IM_GAP:    the duration of the pulse and deassert gap that signify an
                     inter-message gap, between transmissions
      SIG_PULSE:     the duration of the "pulse" assert voltage level
                     (spare, as a future contingency)

    Enumerate the possible signal duration types here for use as indices
    Append additional signal timings here and then initialize them
    in the device initiation of the "signals" array
    Maintain the order here in the initialization code in the device class
*/
enum SIGNAL_T { NONE = -1,
    SIG_SYNC         = 0,
    SIG_SYNC_GAP,
    SIG_ZERO,
    SIG_ONE,
    SIG_IM_GAP,
    SIG_PULSE };

// Structure of the table of timing durations used to signal
typedef struct {
    SIGNAL_T sig_type;   // Index the type of signal
    String sig_name;     // Provide a brief descriptive name
    uint16_t up_time;    // duration for pulse with voltage up
    uint16_t delay_time; // delay with voltage down before next signal
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
    SIGNAL_T cmdList[500];
    uint16_t listEnd   = 0;
    String Device_Name = "ISM Device";
    SIGNAL *signals;

    ISM_Device(){};

    // Inserts a signal into the commmand list
    void insert(SIGNAL_T signal)
    {
        cmdList[listEnd++] = signal;
        return;
    };

    // Drive the transmitter voltages per the timings of the signal list
    void playback()
    {
        SIGNAL_T sig;
        for (int i = 0; i < listEnd; i++) {
            sig = cmdList[i];
            if (sig == NONE) { // Terminates list but should never be executed
                DBG_print(F(" \tERROR -- invalid signal: "));
                DBG_print((int)cmdList[i]);
                DBG_print((cmdList[i] == NONE) ? " (NONE)" : "");
                return;
            };
            if (signals[sig].up_time > 0) {
                digitalWrite(TX, HIGH);
                delayMicroseconds(signals[sig].up_time);
            };
            if (signals[sig].delay_time > 0) {
                digitalWrite(TX, LOW);
                delayMicroseconds(signals[sig].delay_time);
            };
        }; // end loop
    };     // end playback()
};         // end class ISM_device

class omni : public ISM_Device {

  public:
    // omni timing durations
    int sigLen               = 6;
    SIGNAL omni_signals[6] = {
            {SIG_SYNC, "Sync", 600, 600},
            {SIG_SYNC_GAP, "Sync-gap", 0, 0},
            {SIG_ZERO, "Zero", 200, 400},
            {SIG_ONE, "One", 400, 200},
            {SIG_IM_GAP, "IM_gap", 0, 1250},
            {SIG_PULSE, "Pulse", 0, 0} // spare
    };

    // Instantiate the device by linking 'signals' to our device timing
    omni()
    {
        Device_Name = F("omni");
        signals     = omni_signals; 
    };

    /* Routines to create 80-bit omni datagrams from sensor data
       Pack <type, id, iTemp, oTemp, iHum, oHum, press, volts> into a 72-bit
         datagram appended with a 1-byte CRC8 checksum (10 bytes total).
	 Bit fields are binary-encoded, most-significant-bit first.
	 
         Inputs:
	 <type> is a 4-bit datagram type identifier, uint8_t
         <id> is a 4-bit sensor ID, uint8_t
         <temp> is a signed 16-bit value of 10*(temperature reading), int16_t
         <hum> is an unsigned 8-bit value representing the relative humidity as integer, uint8_t
         <press> is a 16-bit unsigned integer representing barometric pressure, uint16_t
                 in hPa*100
         <msg>   is an array of at least 10 unsigned 8-bit uint8_t integers

         Output in "msg" as nibbles:

	     yi 11 12 22 hh gg pp pp vv cc

	     y: type of datagram, 1-15 (0 not allowed)
	     i: id of device, 1-15 (0 not allowed)
	     1: sensor 1 temp reading (e.g, indoor),  °C *10, 2's complement
	     2: sensor 2 temp reading (e.g, outdoor), °C *10, 2's complement
	     h: sensor 1 humidity reading (e.g., indoor),  %RH as integer
	     g: sensor 2 humidity reading (e.g., outdoor), %RH as integer
	     p: barometric pressure * 10, in hPa, 0..1628.4 hPa
	     v: (VCC-2.5)*100, in volts, 2.50..5.06 volts
	     c: CRC8 checksum of bytes 1..9, initial remainder 0x00,
	            divisor polynomial 0x97, no reflections or inversions
	 
    */
    void pack_msg(uint8_t id, int16_t temp, uint16_t hum,
            uint16_t press, uint8_t *msg)
    {
        uint8_t CONST5    = 5;   // Lacrosse CheckSum factor
        uint8_t p_offset  = 200; // Lacrosse pressure offset
        uint8_t xcheck    = 0;
        uint16_t checksum = 0;
        uint8_t sign      = (temp < 0) ? 1 : 0;
        temp              = (temp < 0) ? -temp : temp;
        // Store input binary values as BCD digit nibbles in message frame
        msg[0] = (uint8_t)(0x40 | ((uint8_t)(sign << 3 | id & 0x07)) & 0x0F);
        msg[1] = (temp % 10) << 4 | (temp / 10) % 10;
        msg[2] = ((temp / 100) % 10) << 4 | (hum % 10); //(no 10ths of RH)
        msg[3] = ((hum / 10) % 10) << 4 | (hum / 100) % 10;
        msg[4] = ((press - p_offset) % 10) << 4 | (((press - p_offset) / 10) % 10);
        msg[5] = (((press - p_offset) / 100) % 10) << 4 | 0x02;
        // CheckXOR nibbles 0-5 and CheckSUM nibbles 0-6 and combine in nibble 7
        for (uint8_t j = 0; j < 6; j++) {
            xcheck ^= (msg[j] >> 4);
            xcheck ^= (msg[j] & 0x0F);
            checksum += (msg[j] >> 4);
            checksum += (msg[j] & 0x0F);
        };
        msg[6] = (xcheck << 4);
        msg[6] |= (uint8_t)((checksum + xcheck + CONST5) & 0x0F);
        // And, finally, reflect each nibble to be lsb first
        reflect_nibbles(msg, 8);
        return;
    };

    // Unpack <ID, Temp, Humidity, Pressure> from a 7-byte WS7000 message with
    //   CheckXOR and CheckSum nibbles
    void unpack_msg(uint8_t *msg, uint8_t &I, int16_t &T, uint16_t &H, uint16_t &P)
    {
        uint8_t CONST5    = 5; // Lacrosse CheckSum factor
        uint8_t TYPE4     = 4; // Lacrosse WS7000-20 message type
        uint8_t xcheck    = 0;
        uint16_t checksum = 0;
        uint8_t sign;
        // Initialize return values in case of errors
        I = 0;
        T = 0;
        H = 0;
        P = 0;
        // Start by reversing bits, so lsb is last in each nibble
        reflect_nibbles(msg, 8);
        // Now compute the CheckXOR and CheckSum from the
        //   message frame
        for (uint8_t j = 0; j < 6; j++) {
            xcheck ^= (msg[j] >> 4);
            xcheck ^= (msg[j] & 0x0F);
            checksum += (msg[j] >> 4);
            checksum += (msg[j] & 0x0F);
        };
        if ((msg[6] >> 4) != (xcheck & 0x0F)) {
            DBG_println(F("CheckXOR failed: invalid message"));
            return;
        };
        if ((msg[6] & 0x0F) != (uint8_t)((checksum + xcheck + CONST5) & 0x0F)) {
            DBG_println(F("CheckSum failed: invalid message"));
            return;
        };
        if ((msg[0] >> 4) != TYPE4) {
            DBG_print(F("Invalid message packet type "));
            DBG_print((int)(msg[0] >> 4));
            DBG_println(F(" is not a WS7000-20 message"));
            return;
        };
        // Capture sign for temperature
        sign = ((msg[0] & 0x08) != 0) ? 1 : 0;
        // Get the device id/address
        I = (uint8_t)(msg[0] & 0x07);
        T = (int16_t)((msg[1] >> 4) + ((msg[1] & 0x0f) * 10) + (msg[2] >> 4) * 100);
        if (sign != 0)
            T = -T;
        H = (int16_t)((msg[2] & 0x0f) + ((msg[3] >> 4) * 10) + (msg[3] & 0x0f) * 100);
        P = (int16_t)((msg[4] >> 4) + ((msg[4] & 0x0f) * 10) + (msg[5] >> 4) * 100) + (msg[5] & 0x0F) * 100;
        ;
        return;
    }; // end unpack_msg()

    // This creates the WS7000-20 waveform description
    void make_wave(uint8_t *msg, uint8_t msgLen)
    {
        listEnd    = 0;
        bool first = true;

        // Preamble
        for (uint8_t i = 0; i < 10; i++)
            insert(SIG_ZERO);
        insert(SIG_ONE);

        // The data packet
        if (first) {
            DBG_print(F("The msg packet, length="));
            DBG_print((int)msgLen);
            DBG_print(F(", as a series of bits: "));
        };
        for (uint8_t i = 0; i < msgLen; i++) {
            insert(((uint8_t)((msg[i / 8] >> (7 - (i % 8))) & 0x01)) == 0 ? SIG_ZERO : SIG_ONE);
            if (((i + 1) % 4) == 0)
                insert(SIG_ONE);
            if (first)
                DBG_print(((msg[i / 8] >> (7 - (i % 8))) & 0x01));
        };
        if (first)
            DBG_println();
        first = false;

        // Postamble and terminal marker for safety
        insert(SIG_IM_GAP);
        cmdList[listEnd] = NONE;
    }; // End .make_wave()
};     // End class WS7000

// Global variables

// Create BME object
// We use the I2C version but SPI is an option
Adafruit_BME680 bme; // I2C
// Adafruit_BME680 bme(BME_CS); // hardware SPI
// Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// The WS7000 object as a global
WS7000 ws;
// And count the number of messages as they are sent
int count = 0; // Count the packets sent

void setup(void)
{
    DBG_begin(9600);
    delay(2000);
    DBG_println(F("\nStarting WS7000-20 test transmission"));
    pinMode(TX, OUTPUT);
    digitalWrite(TX, LOW);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    if (!bme.begin()) {
        DBG_println(F("Could not find a valid BME680 sensor, check wiring!"));
        while (!bme.begin())
            ;
    }

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
};                              // End setup()

void loop(void)
{
    uint8_t msg[7]    = {0}; // WS7000-20 messages are 7 bytes
    uint8_t WS7000Len = 56;  // WS7000-20 messages are 56 bits long
    // Initiate with values to compare with .cu8 file
    // Keep id, but replace others with BME readings
    uint8_t id     = 5;
    int16_t temp   = -254; // Temp(C) * 10
    uint16_t hum   = 479;  // Humidity * 10
    uint16_t press = 9950; // Pressure in hPa * 10
    uint16_t voc;

    // Get the readings from the BME688
    if (!bme.performReading()) {
        DBG_println(F("BME688 failed to perform reading :-("));
        return;
    }

    // Unpack BME readings, repack for ISM transmission, and create the waveform
    temp  = (uint16_t)((bme.temperature + 0.05 + BME_TEMP_OFFSET) * 10); // adjust & round
    hum   = (uint16_t)((bme.humidity + 0.5) * 10.0);                     // round
    press = (uint16_t)(bme.pressure / 10.0);                             // hPa * 10
    voc   = (uint16_t)(bme.gas_resistance / 1000.0);                     // KOhms
    ws.pack_msg(id, temp, hum, press, msg);
    ws.make_wave(msg, WS7000Len);

    // Set up for transmission
    digitalWrite(TX, LOW);

    DBG_print(F("Transmit msg "));
    DBG_print(++count);
    DBG_print(F("\tT="));
    DBG_print(bme.temperature);
    DBG_print(F("˚C"));
    DBG_print(F(", H="));
    DBG_print(bme.humidity);
    DBG_print(F("%"));
    DBG_print(F(", P="));
    DBG_print(bme.pressure / 100.0);
    DBG_print(F("hPa"));
    DBG_print(F(", V="));
    DBG_print(bme.gas_resistance / 1000.0);
    DBG_print(F(": 0x "));
    for (uint8_t j = 0; j < 7; j++) {
        DBG_print(msg[j], HEX);
        DBG_print(' ');
    };
    DBG_println();

    // Messages are sent as a continual stream
    digitalWrite(LED, HIGH);
    ws.playback();

    // Done transmitting; turn everything off & delete object
    digitalWrite(LED, LOW);
    digitalWrite(TX, LOW);
    delay(DELAY);
}; // end loop()
