/* -*- c++ -*-
  AR609.ino: 609TXC transmission protocol emulator

  This program uses a 433MHz transmitter and Arduino (or similar device
  supported on the Arduino IDE) to send temperature/humidity
  readings in a format compatible with the Acurite 609TXC protocol.
  See the src/device/acurite.c file in the rtl_433 distribution
  (https://github.com/merbanan/rtl_433) for details about the packet
  format.  The data packet format created here matches the format
  recognized by rtl_433 for the Acurite 609TXC.

  This program executes on the Arduino Uno R3 but is constrained by the
  limited 2K memory limit for variables.  Take care in modifying the
  program as changes in memory requirements may cause execution errors.

  The message is pulse-width modulated, on-off-keying at 433.92MHz.
  Waveform format is 2 sync pulses followed by a sync-gap pulse,
  40-bit string of message 0/1 data bits, and final pulse followed
  by aninterval gap.

  The transmitted waveform is a series of up/down voltages (cycles) that
  turn the ISM transmitter on/off (OOK) followed by timing gaps of
  various durations.  The duration indicates the type of signal
  (PWM -- pulse-width modulation).

  For the Acurite 609 remote sensor transmission protocol that this
  program sends, the cycle begins with a "high" pulse of specific
  duration (500usec) followed by a "low" gap of specific duration.
  The length of the gap indicates the type of information being
  transmitted (synching signals or data bits such as
  520usec ==> "0", 980usec ==> "1", etc).

  Most ISM devices REPEAT the message 2-5 times to increase the
  possibility of correct reception (since this is a simplex
  communication system -- no indication that the information
  was correctly received).  This version repeats the packet
  3 times per transmission.

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
  voltages and delay specific length of time, and then executes them,
  with minimal processing overhead.

  This program was modeled, somewhat, on Joans pigpiod (Pi GPIO daemon)
  code for waveform generation.  But because the microcontroller devices
  like Arduinos are single-program rather than multitasking OSes, the code
  here does not need to provide for contingencies as those in multi-tasking
  environments -- it just generates the waveform description which a subsequent
  code module uses to drive the transmitter voltages.

  The code here provides a base class containing structure definitions,
  variables, and procedures that can be inherited and expanded to
  create transmission protocols compatible with other specific devices.

  The BME68x code for reading temp/press/hum/VOC was adapted from
  the Adafruit demo program http://www.adafruit.com/products/3660
  The BME68x temperature reading may need calibration against
  an external thermometer.  The DEFINEd parameter 'BME_TEMP_OFFSET'
  (below) can be used to perform an adjustment, if needed.

  hdtodd@gmail.com, 2024.12.29
*/

//#define DELAY 29350   // Time between messages in ms, less 650ms for overhead on SAMD21
#define DELAY 4350 // Time between messages in ms, less 650ms for overhead on SAMD21

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

// 433MHz transmitter settings
#ifdef PICO_RP2350
#define TX  3           // transmit data line connected to Pico 2 GPIO 3
#define LED LED_BUILTIN // LED active on GPIO 25 when transmitting
#else
#define TX  4  // transmit data line connected to SAMD21 GPIO 4
#define LED 13 // LED active on GPIO 13 when transmitting
#endif
#define REPEATS 3 // Number of times to repeat packet in one transmission

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

    For the TX141TH:
      SIG_SYNC:      the durations of the pulse and "sync" deassert voltage level
      SIG_SYNC_GAP:  the duration of the pulse and"sync_gap" deassert before data bits
      SIG_ZERO:      the duration of the pulse and deassert gap that signify a "0" data bit
      SIG_ONE:       the duration of the pulse and deassert gap that signify a "1" data bit
      SIG_IM_GAP:    the duration of the pulse and deassert gap that signify an
                     inter-message gap, between transmissions
      SIG_PULSE:     the duration of the "pulse" assert voltage level
                     (spare, as a future contingency)
*/

/*  Enumerate the possible signal duration types here for use as indices
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
    SIGNAL_T cmdList[300];
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

class AR609 : public ISM_Device {
  public:
    // AR609 timing durations
    int sigLen              = 6;
    SIGNAL AR609_signals[6] = {
            {SIG_SYNC, "Sync", 512, 480},
            {SIG_SYNC_GAP, "Sync-gap", 512, 8912},
            {SIG_ZERO, "Zero", 512, 976},
            {SIG_ONE, "One", 512, 1968},
            {SIG_IM_GAP, "IM_gap", 512, 10200},
            {SIG_PULSE, "Pulse", 512, 512} // spare
    };

    // Instantiate the device by linking 'signals' to our device timing
    AR609()
    {
        Device_Name = F("Acurite 609TXC");
        signals     = AR609_signals;
    };

    // Routine to create 40-bit AR609 datagrams
    // Pack <ID, Status, Temp, Humidity> into a 5-byte AR609 message with 1 checksum byte
    void pack_msg(uint8_t I, uint8_t S, int16_t T, uint8_t H, uint8_t *msg)
    {
        msg[0] = (I & 0xff);
        msg[1] = ((S & 0x0f) << 4 | (T >> 8) & 0x0f);
        msg[2] = (T & 0xff);
        msg[3] = (H & 0xff);
        msg[4] = (msg[0] + msg[1] + msg[2] + msg[3]) & 0xff;
        return;
    }; // end pack_msg()

    // unpack_msg <ID, Status, Temp, Humidity> from a 5-byte AR609 message with 1 checksum byte
    void unpack_msg(uint8_t *msg, uint8_t &I, uint8_t &S, int16_t &T, uint8_t &H)
    {
        if (msg[4] != ((msg[0] + msg[1] + msg[2] + msg[3]) & 0xff)) {
            DBG_println(F("Invalid message packet: Checksum error"));
            I = 0;
            S = 0;
            T = 0;
            H = 0;
        }
        else {
            I = msg[0];
            S = (msg[1] & 0xf0) >> 4;
            T = ((int16_t)((msg[1] & 0x0f) << 12 | (msg[2]) << 4)) >> 4;
            H = msg[3];
        };
        return;
    }; // end unpack_msg()

    // This creates the Acurite 609THC waveform description
    void make_wave(uint8_t *msg, uint8_t msgLen)
    {
        listEnd    = 0;
        bool first = true;

        // Preamble
        insert(SIG_SYNC);
        insert(SIG_SYNC);
        insert(SIG_SYNC_GAP);

        // The data packet
        if (first) {
            DBG_print(F("The msg packet, length="));
            DBG_print((int)msgLen);
            DBG_print(F(", as a series of bits: "));
        };
        for (uint8_t i = 0; i < msgLen; i++) {
            insert(((uint8_t)((msg[i / 8] >> (7 - (i % 8))) & 0x01)) == 0 ? SIG_ZERO : SIG_ONE);
            if (first)
                DBG_print(((msg[i / 8] >> (7 - (i % 8))) & 0x01));
        };
        if (first)
            DBG_println();
        first = false;

        // Postamble and terminal marker for safety
        insert(SIG_IM_GAP);
        cmdList[listEnd] = NONE;
    }; // end .make_wave()
};     // end class AR609

// Create BME object
// We use the I2C version but SPI is an option
Adafruit_BME680 bme; // I2C
// Adafruit_BME680 bme(BME_CS); // hardware SPI
// Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// Create the AR609 objext as a global
AR609 ar;

// and count the number of messages as they are sent
int count = 0; // Count the packets sent

void setup(void)
{
    DBG_begin(9600);
    delay(2000);
    DBG_println(F("\nAR609 Starting test transmission"));
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
    uint8_t id = 199, st = 2, hum;
    uint16_t temp, press, voc;
    uint8_t msg[20];   // should not have messages > 20 bytes = 160 bits
    int AR609Len = 40; // Acurite 609 has 40-bit message length

    // Get the readings from the BME688
    if (!bme.performReading()) {
        DBG_println(F("BME688 failed to perform reading :-("));
        return;
    }

    // Unpack BME readings, repack for ISM transmission, and create the waveform
    temp  = (uint16_t)((bme.temperature + 0.05 + BME_TEMP_OFFSET) * 10); // adjust & round
    hum   = (uint8_t)(bme.humidity + 0.5);                               // round
    press = (uint16_t)(bme.pressure / 10.0);                             // hPa * 10
    voc   = (uint16_t)(bme.gas_resistance / 1000.0);                     // KOhms
    ar.pack_msg(id, st, temp, hum, msg);
    ar.make_wave(msg, AR609Len);

    // Set up for transmission
    digitalWrite(TX, LOW);

    DBG_print(F("Transmit msg "));
    DBG_print(++count);
    DBG_print(F("\tT="));
    DBG_print(temp / 10.0);
    DBG_print(F("˚C"));
    DBG_print(F(", H="));
    DBG_print(hum);
    DBG_print(F("%"));
    DBG_print(F(", P="));
    DBG_print(press / 10.0);
    DBG_print(F("hPa"));
    DBG_print(F(", V="));
    DBG_print(voc);
    DBG_print(F(": 0x "));
    for (uint8_t j = 0; j < 5; j++) {
        DBG_print(msg[j], HEX);
        DBG_print(' ');
    };
    DBG_println();

    // Messages must be sent as a continual stream, with no
    //   program breaks once transmission begins
    // So the data-transmit loop is a single loop
    // Repeat packet 'REPEATS' times in this transmission

    digitalWrite(LED, HIGH);
    for (int j = 0; j < REPEATS; j++)
        ar.playback();

    // Done transmitting; turn everything off & wait
    digitalWrite(LED, LOW);
    digitalWrite(TX, LOW);
    delay(DELAY);
}; // end loop()
