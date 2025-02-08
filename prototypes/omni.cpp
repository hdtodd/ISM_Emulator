/* g++ -std=c++11
  omni.cpp: Prototype for "omni" sensor  protocol emulator for Arduino

  This program models and tests code to describe and generate
  waveform specifications on Arduino/Sparkfun devices.  The
  waveform specifications can be adapted to modulate ISM-band
  (e.g, 433MHz) on-off keying and pulse-width modulation (OOK/PWM)
  transmitters to generate waveforms compatible with the protocols
  used by ISM-band remote sensing devices.

  The waveform is a series of up/down voltages (cycles) that turn
  the ISM transmitter on/off (OOK) followed by timing gaps of
  various durations.  The duration indicates the type of signal
  (PWM -- pulse-width modulation).

  As a specific example, For the Acurite 609 remote sensor protocol
  for which this program was originally designed, the cycle begins
  with a "high" pulse of specific duration (500usec) followed by a
  "low" gap of specific duration.  The length of the gap indicates
  the type of information being transmitted (synching signals or
  data bits such as 520usec ==> "0", 980usec ==> "1", etc).

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
  create waveforms compatible with specific devices.

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

/*  libcrc8.c
    From http://github.com/hdtodd/CRC8-Library
    HD Todd, February, 2022

    Predefined table of CRC-8 lookup bytes computed using
    the polynomial 0x97, also know as "C2". Per Koopman,
    "arguably the best" for messages up to 119 bits long

   This table can be recomputed for a different polynomial
   See original library distribution on github
*/

#define REPEAT 4

uint8_t CRC8POLY       = 0x97;
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
        0xf7, 0x60, 0x4e, 0xd9, 0x12, 0x85, 0xab, 0x3c};

uint8_t crc8(uint8_t *msg, int lengthOfMsg, uint8_t init)
{
    while (lengthOfMsg-- > 0)
        init = CRC8Table[(init ^ *msg++)];
    return init;
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
enum SIGNAL_T {
    NONE     = -1,
    SIG_SYNC = 0,
    SIG_SYNC_GAP,
    SIG_ZERO,
    SIG_ONE,
    SIG_IM_GAP,
    SIG_PULSE
};

// Structure of the table of timing durations used to signal
typedef struct {
    SIGNAL_T sig_type;   // Index the type of signal
    string sig_name;     // Provide a brief descriptive name
    uint16_t up_time;    // duration for pulse with voltage up
    uint16_t delay_time; // delay with voltage down before next signal
} SIGNAL;

/*  ISM_Device is the base class descriptor used to generate waveforms
    that can be compatible with OOK-PWM transmission protocols of specific
    devices. It contains the list of signals for the transmitter driver,
    the variables needed to translate procedure calls into the signals list,
    and the procedures needed to insert the corresponding signals into the list.
*/
class ISM_Device {
  public:
    // These are used by the device object procedures
    //   to process the waveform description into a list of signals
    SIGNAL_T cmdList[640];
    uint16_t listEnd   = 0;
    string Device_Name = "ISM Device";
    uint8_t Lo         = 0;
    uint8_t Hi         = 1;
    SIGNAL *signals;

    ISM_Device(){};

    // Inserts a signal into the commmand list
    void insert(SIGNAL_T signal)
    {
        cmdList[listEnd++] = signal;
        return;
    };

    // Plays back the signaling commands in the device's cmdList[]
    void playback()
    {
        SIGNAL_T sig;
        cout << "\nFor device " << Device_Name << " the waveform generated is:" << endl;
        for (int i = 0; i < listEnd; i++) {
            sig = cmdList[i];
            cout << setw(3) << right << i << ": Cmd= "
                 << setw(2) << right << (int)sig;
            if (sig == NONE) { // Terminates list but should never be executed
                cout << " \tERROR -- invalid NULL command at cmdList index i=" << i << endl;
                return;
            };
            cout << setw(3) << right << i << ": Cmd= " << setw(2) << right << (int)cmdList[i];
            cout << setw(9) << "\tASSERT" << setw(6) << right << signals[sig].up_time;
            cout << setw(9) << "\tDEASSERT" << setw(6) << right << signals[sig].delay_time;
            cout << "\t" << signals[sig].sig_name << endl;
        };
    }; // end playback()
};     // end class ISM_device

class omni : public ISM_Device {

  public:
    // omni timing durations
    // Name, description, pulse in μs, gap in μs
    int sigLen             = 6;
    SIGNAL omni_signals[6] = {
            {SIG_SYNC, "Sync", 600, 600},
            {SIG_SYNC_GAP, "Sync-gap", 200, 800},
            {SIG_ZERO, "Zero", 200, 400},
            {SIG_ONE, "One", 400, 200},
            {SIG_IM_GAP, "IM_gap", 0, 2000},
            {SIG_PULSE, "Pulse", 0, 0} // spare
    };

    // Instantiate the device by linking 'signals' to our device timing
    omni()
    {
        Device_Name = "omni";
        signals     = omni_signals;
        cout << "Created device " << Device_Name << endl;
    };

    /* Routines to create 80-bit omni datagrams from sensor data
       Pack <type, id, iTemp, oTemp, iHum, oHum, press, volts> into a 72-bit
         datagram appended with a 1-byte CRC8 checksum (10 bytes total).
         Bit fields are binary-encoded, most-significant-bit first.

         Inputs:
         uint8_t  <type>  is a 4-bit unsigned integer datagram type identifier
         uint8_t  <id>    is a 4-bit unsigned integer sensor ID
         uint16_t <temp>  is a 16-bit signed twos-complement integer representing 10*(temperature reading)
         uint8_t  <hum>   is an 8-bit unsigned integer representing the relative humidity as integer
         uint16_t <press> is a 16-bit unsigned integer representing barometric 10*pressure (in hPa)
         uint16_t <volts> is a 16-bit unsigned integer representing 100*(voltage-2.50) volts
         uint8_t  <msg>   is an array of at least 10 unsigned 8-bit uint8_t integers

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
    void pack_msg(uint8_t type, uint8_t id, int16_t iTemp, int16_t oTemp,
            uint8_t iHum, uint8_t oHum, uint16_t press, uint16_t volts, uint8_t msg[])
    {
        msg[0] = (type & 0x0f) << 4 | (id & 0x0f);
        msg[1] = (iTemp >> 4) & 0xff;
        msg[2] = ((iTemp << 4) & 0xf0) | ((oTemp >> 8) & 0x0f);
        msg[3] = oTemp & 0xff;
        msg[4] = iHum & 0xff;
        msg[5] = oHum & 0xff;
        msg[6] = (press >> 8) & 0xff;
        msg[7] = press & 0xff;
        msg[8] = volts & 0xff;
        msg[9] = crc8(msg, 9, 0x00);
        return;
    };

    // Unpack message into data values it represents
    void unpack_msg(uint8_t msg[], uint8_t &type, uint8_t &id, int16_t &iTemp, int16_t &oTemp,
            uint8_t &iHum, uint8_t &oHum, uint16_t &press, uint8_t &volts)
    {
        if (msg[9] != crc8(msg, 9, 0x00)) {
            cout << "Invalid message packet: Checksum error" << endl;
            type  = 0;
            id    = 0;
            iTemp = 0;
            oTemp = 0;
            iHum  = 0;
            oHum  = 0;
            press = 0;
            volts = 0;
        }
        else {
            type  = msg[0] >> 4;
            id    = msg[0] & 0x0f;
            iTemp = ((int16_t)((((uint16_t)msg[1]) << 8) | (uint16_t)msg[2])) >> 4;
            oTemp = ((int16_t)((((uint16_t)msg[2]) << 12) | ((uint16_t)msg[3]) << 4)) >> 4;
            iHum  = msg[4];
            oHum  = msg[5];
            press = ((uint16_t)(msg[6] << 8)) | msg[7];
            volts = msg[8];
        };
        return;
    };

    void make_wave(uint8_t *msg, uint8_t msgLen)
    {
        listEnd    = 0;
        bool first = true;

        // Repeat message "REPEAT" times in one transmission
        for (uint8_t j = 0; j < REPEAT; j++) {

            // Preamble
            for (uint8_t i = 0; i < 4; i++)
                insert(SIG_SYNC);

            // The data packet
            if (first)
                cout << "The msg packet, length=" << (int)msgLen << ", as a series of bits:";

            for (uint8_t i = 0; i < msgLen; i++) {
                insert(((uint8_t)((msg[i / 8] >> (7 - (i % 8))) & 0x01)) == 0 ? SIG_ZERO : SIG_ONE);
                if (first)
                    cout << setw(1) << ((msg[i / 8] >> (7 - (i % 8))) & 0x01);
            };
            if (first)
                cout << endl;
            first = false;
        };

        // Postamble and terminal marker for safety
        insert(SIG_IM_GAP);
        cmdList[listEnd] = NONE;
    }; // end .make_wave()
};     // end class omni

int main()
{
    // In the absence of a real sensor provide values for testing
    // Set the values in float format and convert them to .pack_msg() format
    uint8_t type = 1;
    uint8_t id   = 1;
    float itempf, otempf, ihumf, ohumf, pressf, voltsf;
    int16_t itemp, otemp, press, volts;
    uint8_t ihum, ohum;
    uint8_t y, i, ih, oh, v;
    int16_t it, ot;
    uint16_t p;

    // omni messages are 80 bits = 10 bytes
    uint8_t msg[11];
    uint8_t const msgLen = 80;

    cout << "\nGenerate an omni device waveform and play back as pulse cycles\n"
         << endl;
    cout << "Create 'om' as an omni device" << endl;
    omni om;

    // Set the data values as if they had come from a sensor
    itempf = -95.9; //°C
    otempf = -18.3; //°C
    ihumf  = 34.8;  //%
    ohumf  = 49.3;  //%
    pressf = 999.3; //hPa
    voltsf = 4.936; //volts

    // Convert values to format needed by .pack_msg()
    itemp = (int16_t)(10.0 * itempf + (signbit(itempf) ? -0.5 : +0.5)); // round with the 0.5
    otemp = (int16_t)(10.0 * otempf + (signbit(otempf) ? -0.5 : +0.5)); // round with the 0.5
    ihum  = (uint8_t)(ihumf + 0.5);                                     // round with the 0.5
    ohum  = (uint8_t)(ohumf + 0.5);                                     // round with the 0.5
    press = (uint16_t)(10.0 * pressf + 0.5);                            // round with the 0.5
    volts = (uint8_t)(100.0 * (voltsf - 2.50) + 0.5);

    cout << "Pack the data values into the message packet ..." << endl;
    om.pack_msg(type, id, itemp, otemp, ihum, ohum, press, volts, msg);
    cout << "Message as packed for device 'om' in hex: 0x ";
    for (uint8_t j = 0; j < 10; j++)
        cout << setw(2) << setfill('0') << hex << (int)msg[j] << " ";
    cout << dec << setfill(' ') << endl;

    // Now unpack the message for comparison with original values
    cout << "\nUnpack the message block and retrieve field values" << endl;
    om.unpack_msg(msg, y, i, it, ot, ih, oh, p, v);

    cout << "Message values as retrieved from transmitted packet:\n\tOriginal\tRetrieved" << endl;
    cout << "Type\t" << (int)type << "\t\t" << (int)y << endl;
    cout << "ID\t" << (int)id << "\t\t" << (int)i << endl;
    cout << "iTemp\t" << itempf << "\t\t" << ((float)it) / 10.0 << endl;
    cout << "oTemp\t" << otempf << "\t\t" << ((float)ot) / 10.0 << endl;
    cout << "iHum\t" << ihumf << "\t\t" << (float)ih << endl;
    cout << "oHum\t" << ohumf << "\t\t" << (float)oh << endl;
    cout << "Press\t" << pressf << "\t\t" << ((float)p) / 10.0 << endl;
    cout << "Volts\t" << voltsf << "\t\t" << ((float)v) / 100.0 + 2.50 << endl
         << endl;

    // Now create the waveform and play it back
    cout << "\nNow create a wave for 'om' and play it back as signals" << endl;
    om.make_wave(msg, msgLen);
    om.playback();

    return 0;
};
