/*
 * iDSB Transceiver Copyright (c) 2021 Ian Mitchell, VK7IAN
 *
 * for nano clone with CH340 choose board "arduino duemilanove or diecimila"
 * see: manual install in: http://www.arduino.cc/en/Guide/Libraries
 * (eg, briefly, copy PWM to C:/Users\Ian\Documents\Arduino\libraries and restart arduino IDE)
 */

/*
 * TODO:
 *   status: meter eg, SWR, PWR, SIG (smeter), ATT (rx or tx), TUN, LCK
 *   enable/disable spectrum display
 *   CWL, CWU
 *   change band with frequency - TCA9534A
 *
 *
 *
 *   Meter: SIG, SWR, PWR, TUN
 *
 *   SWR = (Vf+Vr)/(Vf-Vr)
 *   PWR = Vf.Vf/100
 *   PWR = ((fwd+62)*5*8)^2 / 1024 / 1024 = result is *100
 *   (62 is 0.3/5 * 1024 => the diode forward voltage drop correction)
 *
 *   1.  LSB – SSB mode
 *   2.  USB – SSB mode
 *   3.  CWU – CW on upper sideband
 *   4.  CWL – CW on lower sideband
 *   5.  DIG – Digital Mode
 *   6.  DSB – DSB mode
 *   7.  STP – Tuning step
 *   8.  BND – Band
 *   9.  TUN – Antenna Tune
 *   10. ATT – Attenuator
 *   11. RBW – Receive bandwidth
 *   12. TBW – Transmit bandwidth
 *   13. SCP – Band scope
 *   14. LCK - Lock tuning
 *   15. RIT – Receive only fine tuning (Receiver Incremental Tuning)
 *
 */

/*

// In global declarations:
GFXcanvas1 canvas(128, 32); // 128x32 pixel canvas
// In code later:
canvas.println("I like cake");
tft.drawBitmap(x, y, canvas.getBuffer(), 128, 32, foreground, background); // Copy to screen

*/

/*
 * "C:\Program Files (x86)\Arduino\hardware\tools\avr\avr\bin\objdump" -S iDSB0007.ino.elf > test2.txt
 */

//https://github.com/brianlow/Rotary
//https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads
//https://github.com/adafruit/Adafruit-ST7735-Library
//https://github.com/hideakitai/TCA9534 - note: remove Serial.print from writeBytes() function!
//http://gammon.com.au/adc
//https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h
//http://wiki.openmusiclabs.com/wiki/ArduinoFHT
//https://github.com/pilotak/MCP3X21 - MCP3021 - 0x48 (forward) and 0x4D (reverse)


#define LOG_OUT 1 // use the log output function
#define FHT_N 256 // set to 256 point fht

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <TCA9534.h>         // note: remove Serial.print() from writeBytes() function!
#include <SPI.h>
#include <Wire.h>
#include <PWM.h>
#include <FHT.h>

#define MIN_FREQUENCY        500000UL
#define MAX_FREQUENCY        30000000UL
#define DEFAULT_FREQUENCY    7000000UL
#define DEFAULT_TUNING_STEP  1000UL
#define DEFAULT_MIXER        3000UL
#define DEFAULT_MIXER_ACTUAL 3052UL
#define DEFAULT_RX_FILTER    2700UL
#define DEFAULT_TX_FILTER    2500UL
#define PTT_THRESHOLD        50U
#define METER_DECAY_RATE     10U
#define CW_TONE              601UL

#define POS_FREQUENCY_X    80
#define POS_FREQUENCY_Y     0
#define POS_TX_X           10
#define POS_TX_Y            5
#define POS_RX_X           40
#define POS_RX_Y            5
#define POS_MODE_X         10
#define POS_MODE_Y         30
#define POS_OPTION_LSB_X    1
#define POS_OPTION_LSB_Y   56
#define POS_OPTION_USB_X   49
#define POS_OPTION_USB_Y   56
#define POS_OPTION_DIG_X   97
#define POS_OPTION_DIG_Y   56
#define POS_OPTION_STP_X  145
#define POS_OPTION_STP_Y   56
#define POS_OPTION_SCP_X  193
#define POS_OPTION_SCP_Y   56
#define POS_OPTION_CWU_X    1
#define POS_OPTION_CWU_Y   82
#define POS_OPTION_CWL_X   49
#define POS_OPTION_CWL_Y   82
#define POS_OPTION_DSB_X   97
#define POS_OPTION_DSB_Y   82
#define POS_OPTION_BND_X  145
#define POS_OPTION_BND_Y   82
#define POS_OPTION_TUN_X  193
#define POS_OPTION_TUN_Y   82
#define POS_OPTION_ATT_X    1
#define POS_OPTION_ATT_Y  108
#define POS_OPTION_RBW_X   49
#define POS_OPTION_RBW_Y  108
#define POS_OPTION_TBW_X   97
#define POS_OPTION_TBW_Y  108
#define POS_OPTION_RIT_X  145
#define POS_OPTION_RIT_Y  108
#define POS_OPTION_LCK_X  193
#define POS_OPTION_LCK_Y  108
#define POS_METER_X       100
#define POS_METER_Y        25
#define POS_TUNING_STEP_X 140
#define POS_TUNING_STEP_Y  40
#define POS_WATER_X         0
#define POS_WATER_Y        82

#define D0          0
#define D1          1
#define D2          2
#define D3          3
#define D4          4
#define D5          5
#define D6          6
#define D7          7
#define D8          8
#define D9          9
#define D10         10
#define D11         11
#define D12         12
#define D13         13

#define FILTER1_PIN D9 // digital output (PWM)
#define FILTER2_PIN D3 // digital output (PWM)
#define MIXER_PIN   D5 // digital output (PWM)
#define TX_PIN      D2 // digital output
#define ES_PIN      D7 // digital output
#define CW_PIN      D8 // digital output
#define MUTE_PIN    D4 // digital output
#define CW_TONE_PIN D6 // digital output (tone) (also PD6)
#define SENSE_PIN   A7 // analog input (PTT, Paddle)
#define SIGMON_PIN  A6 // analog input (S Meter)
#define SCAN_PIN    A0 // analog input (spectrum)

#define ENCBUT_PIN  D0  // digital input (with pullup)
#define ENCB_PIN    A3  // digital input (with pullup)
#define ENCA_PIN    A2  // digital input (with pullup)
#define SENSE_ADC   0x7 // analog input (PTT, Paddle)
#define SIGMON_ADC  0x6 // analog input (S Meter)
#define SCAN_ADC    0x0 // analog input (spectrum)
#define CW_PORT_PIN (1<<PD6) // used in ADC ISA for generating CW tone

//#define WRITESPI(x) for (SPDR = (x); (!(SPSR & _BV(SPIF)));)
///*
#define WRITERAW(x) SPDR = (x);
#define WRITESPI(x) \
  SPDR = (x); \
  asm volatile("rjmp .+0\n"); \
  asm volatile("rjmp .+0\n"); \
  asm volatile("rjmp .+0\n"); \
  asm volatile("rjmp .+0\n"); \
  asm volatile("rjmp .+0\n"); \
  asm volatile("rjmp .+0\n"); \
  asm volatile("rjmp .+0\n"); \
  asm volatile("rjmp .+0\n"); \
  asm volatile("nop\n");
//*/

// radio state
enum __attribute__ ((__packed__)) state_t
{
  STATE_NO_STATE,
  STATE_SSB_RECEIVE_INIT,
  STATE_SSB_RECEIVE,
  STATE_LSB_TX_INIT,
  STATE_LSB_TX,
  STATE_USB_TX_INIT,
  STATE_USB_TX,
  STATE_CW_RECEIVE_INIT,
  STATE_CW_RECEIVE,
  STATE_CW_TX_INIT,
  STATE_CW_TX,
  STATE_STEP,
  STATE_MODE,
  STATE_BAND,
  STATE_FILTER,
  STATE_SCAN,
  STATE_STEP_INIT,
  STATE_STEP_SELECT,
  STATE_OPTION_INIT,
  STATE_OPTION_WAIT,
  STATE_OPTION_SELECT,
  STATE_OPTION_EXIT
};

#define OPTION_MODE_LSB  0
#define OPTION_MODE_USB  1
#define OPTION_MODE_DIG  2
#define OPTION_STEP      3
#define OPTION_SCP       4
#define OPTION_MODE_CWU  5
#define OPTION_MODE_CWL  6
#define OPTION_DSB       7
#define OPTION_BND       8
#define OPTION_TUN       9
#define OPTION_ATT      10
#define OPTION_RBW      11
#define OPTION_TBW      12
#define OPTION_RIT      13
#define OPTION_LCK      14
#define OPTION_NULL     15

// radio mode
enum __attribute__ ((__packed__)) mode_t {MODE_LSB,MODE_USB,MODE_CWU,MODE_CWL,MODE_DIGITAL};
enum __attribute__ ((__packed__)) band_t {BAND_OTHER,BAND_160,BAND_80,BAND_40,BAND_20,BAND_15,BAND_10};

typedef struct
{
  uint32_t frequency;
  uint32_t tuning_step;
  uint16_t rxfilter1;
  uint16_t rxfilter2;
  uint16_t txfilter1;
  uint16_t txfilter2;
  mode_t mode;
  band_t band;
  uint8_t locked;
  uint8_t tx;
} radio_t;

static radio_t radio =
{
  DEFAULT_FREQUENCY,
  DEFAULT_TUNING_STEP,
  DEFAULT_RX_FILTER,
  DEFAULT_RX_FILTER,
  DEFAULT_TX_FILTER,
  DEFAULT_TX_FILTER,
  MODE_LSB,
  BAND_40,
  false,
  false
};

volatile static uint8_t wp = 0;
static uint8_t water[10][60] = {0};

static const uint16_t color_map[16] =
{
  0x0000, // black
  0x0010, // blue
  0x0018,
  0x001f,
  0x4208,
  0x630c,
  0x8410,
  0xfff0,
  0xffe8,
  0xffe0,
  0xfc00,
  0xfdc8,
  0xfe73,
  0xfb2c,
  0xf986,
  0xf800  // red
};
/*
// Green
static const uint16_t color_map[16] =
{
  0x0000, // black
  0x0040,
  0x0080,
  0x00c0,
  0x0100,
  0x0140,
  0x0180,
  0x01c0,
  0x0200,
  0x0240,
  0x0280,
  0x02c0,
  0x0300,
  0x0340,
  0x0380,
  0xffff
};
*/

/*
// Blue, too dark!
static const uint16_t color_map[16] =
{
  0x0000, // black
  0x0001,
  0x0002,
  0x0004,
  0x0006,
  0x0008,
  0x000a,
  0x000c,
  0x000e,
  0x0010,
  0x0012,
  0x0014,
  0x0016,
  0x0018,
  0x001f,
  0xffff
};
*/

////
volatile static int8_t knob = 0;
volatile static uint16_t sense = 0x3ffU;
volatile static uint16_t sigmon = 0;
volatile static uint8_t scounter = 0;
volatile static uint8_t p = 0;
volatile static uint8_t buffer_full = false;
volatile static uint8_t processing_complete = false;
static uint8_t waterfall_enabled = false;
static uint8_t cw_tone_enable = false;
static uint8_t last_option = 15;
static state_t radio_state = STATE_SSB_RECEIVE_INIT;
static state_t next_state = STATE_NO_STATE;
static int8_t option = 0;

// delay that doesn't use a timer
void delayMS(const uint16_t ms)
{
  for (uint16_t i=0;i<ms;i++)
  {
    delayMicroseconds(1000);
  }
}

/*

General Init:
      Set TX pin to output
      Set MUTE pin to output
      Set ES pin to output
      Set CW pin to output
      Set FILTER1 pin to output
      Set FILTER2 pin to output
      Set AMIX pin to output
      Set SENSE pin to input (analogue)
      Set SIGMON pin to input (analogue)
      Init LCD (does CS, RST need to be configured manually?)***
      Init SI514
      Init TCA9534
      Set default frequency
      Set default mode: LSB
      Set default TX/RX: RX
      Set LPF relays based on default frequency
      Set attenuation relays to disabled
      Set CTX relay to engage
      Set inductor relays to disengaged
      Set capacitor relays to disengaged
      Set State to RECEIVE_SSB_INIT

Receive SSB Init: (state: RECEIVE_SSB_INIT)
      Set TX pin to output
      Set MUTE pin to output
      Set ES pin to output
      Set CW pin to output
      Set FILTER1 pin to output
      Set FILTER2 pin to output
      Set AMIX pin to output
      Set SENSE pin to input (digital)
      Set SIGMON pin to input (analogue)

      Set TX low (receive)
      Set ES low
      Set CW low
      Set AMIX low

      Init LCD (does CS, RST need to be configured manually?)
      Init timers for FILTER1 and FILTER2
      Set FILTER1 frequency
      Set FITLER2 frequency
      Set State to RECEIVE_SSB

Receive (state: RECEIVE_SSB)

 */
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
/* Rotary encoder handler for arduino. v1.1
 *
 * Copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3.
 * Contact: bb@cactii.net
 *
 * A typical mechanical rotary encoder emits a two bit gray code
 * on 3 output pins. Every step in the output (often accompanied
 * by a physical 'click') generates a specific sequence of output
 * codes on the pins.
 *
 * There are 3 pins used for the rotary encoding - one common and
 * two 'bit' pins.
 *
 * The following is the typical sequence of code on the output when
 * moving from one step to the next:
 *
 *   Position   Bit1   Bit2
 *   ----------------------
 *     Step1     0      0
 *      1/4      1      0
 *      1/2      1      1
 *      3/4      0      1
 *     Step2     0      0
 *
 * From this table, we can see that when moving from one 'click' to
 * the next, there are 4 changes in the output code.
 *
 * - From an initial 0 - 0, Bit1 goes high, Bit0 stays low.
 * - Then both bits are high, halfway through the step.
 * - Then Bit1 goes low, but Bit2 stays high.
 * - Finally at the end of the step, both bits return to 0.
 *
 * Detecting the direction is easy - the table simply goes in the other
 * direction (read up instead of down).
 *
 * To decode this, we use a simple state machine. Every time the output
 * code changes, it follows state, until finally a full steps worth of
 * code is received (in the correct order). At the final 0-0, it returns
 * a value indicating a step in one direction or the other.
 *
 * It's also possible to use 'half-step' mode. This just emits an event
 * at both the 0-0 and 1-1 positions. This might be useful for some
 * encoders where you want to detect all positions.
 *
 * If an invalid state happens (for example we go from '0-1' straight
 * to '1-0'), the state machine resets to the start until 0-0 and the
 * next valid codes occur.
 *
 * The biggest advantage of using a state machine over other algorithms
 * is that this has inherent debounce built in. Other algorithms emit spurious
 * output with switch bounce, but this one will simply flip between
 * sub-states until the bounce settles, then continue along the state
 * machine.
 * A side effect of debounce is that fast rotations can cause steps to
 * be skipped. By not requiring debounce, fast rotations can be accurately
 * measured.
 * Another advantage is the ability to properly handle bad state, such
 * as due to EMI, etc.
 * It is also a lot simpler than others - a static state table and less
 * than 10 lines of logic.
 */

/*
 * The below state table has, for each state (row), the new state
 * to set based on the next encoder output. From left to right in,
 * the table, the encoder outputs are 00, 01, 10, 11, and the value
 * in that position is the new state to set.
 */

// Enable weak pullups
#define ENABLE_PULLUPS

// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0x0
#define DIR_CW 0x10  // Clockwise step.
#define DIR_CCW 0x20 // Anti-clockwise step.
#define R_START 0x0

#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5

static const uint8_t ttable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};
#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

static const uint8_t ttable[7][4] =
{
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

/*
 * Constructor. Each arg is the pin number for each encoder contact.
 */
class Rotary
{
  public:
    Rotary(char, char);
    // Process pin(s)
    unsigned char process();
  private:
    unsigned char state;
    unsigned char pin1;
    unsigned char pin2;
};

Rotary::Rotary(char _pin1, char _pin2) {
  // Assign variables.
  pin1 = _pin1;
  pin2 = _pin2;
  // Set pins to input.
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
#ifdef ENABLE_PULLUPS
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, HIGH);
#endif
  // Initialise state.
  state = R_START;
}

unsigned char Rotary::process(void)
{
  // Grab state of input pins.
  unsigned char pinstate = (digitalRead(pin2) << 1) | digitalRead(pin1);
  // Determine new state from the pins and state table.
  state = ttable[state & 0xf][pinstate];
  // Return emit bits, ie the generated event.
  return state & 0x30;
}

//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
#define SI514_OUTPUT_DISABLE_132 0x0
#define SI514_OUTPUT_ENABLE_132 0x4
#define SI514_CALIBRATE_132 0x1

// calibration
// 1. set for 10MHz output signal
// 2. measure the actual output signal, M
// 3. New XOSC = M/10000000*31980000
//#define XOSC 31980000
#define XOSC 31979894

class Si514
{
   public:
      Si514(const uint8_t I2Caddress,const uint32_t default_frequency);
      void setFrequency(const uint32_t f);
   private:
      uint8_t _I2Caddress;
      uint32_t _frequency;
};

Si514::Si514(const uint8_t I2Caddress=0x55,const uint32_t default_frequency=10000000)
{
   _I2Caddress = I2Caddress;
   _frequency = default_frequency;
   Wire.begin();
}

void Si514::setFrequency(const uint32_t f)
{
  static uint32_t centre_frequency = _frequency;
  static uint64_t m = 0;

  // 2080/1022*1e6*2^32
  static const uint64_t ls_magic = 8741225025127202ULL;
  static const uint64_t hs_magic = 8933531975680000000ULL;

  const uint64_t ppm = (uint64_t)(f>centre_frequency?f-centre_frequency:centre_frequency-f)*1000000ULL/centre_frequency;
  if (ppm>1000ULL||m==0ULL)
  {
    // new centre frequency
    centre_frequency = f;

    uint16_t ls_div = 0;
    uint16_t hs_div = 0;
    uint16_t M_int = 0;
    uint32_t M_frac = 0;
    uint16_t LP1 = 0;
    uint16_t LP2 = 0;
    uint16_t r = 0;

    const uint64_t l = ls_magic/f;
    if (l<(1ULL<<32)) {ls_div = 1;r = 0;}
    else if (l<(1ULL<<33)) {ls_div = 2;r = 1;}
    else if (l<(1ULL<<34)) {ls_div = 4;r = 2;}
    else if (l<(1ULL<<35)) {ls_div = 8;r = 3;}
    else if (l<(1ULL<<36)) {ls_div = 16;r = 4;}
    else if (l<(1ULL<<37)) {ls_div = 32;r = 5;}

    const uint64_t h = hs_magic/f/ls_div;
    hs_div = (h>>32)+((h&0xffffffff)?1:0);
    hs_div = (hs_div&1)?hs_div+1:hs_div;
    m = (((uint64_t)f)<<32)/XOSC*ls_div*hs_div;
    M_frac = (m>>3)&0x1fffffff;
    M_int = (m>>32)&0xff;

    // these magic numbers are directly from the datasheet
    if (m<280289480894ULL) {LP1 = 2;LP2 = 2;}
    else if (m<291455464787ULL) {LP1 = 2;LP2 = 3;}
    else if (m<313264713941ULL) {LP1 = 3;LP2 = 3;}
    else if (m<325744342994ULL) {LP1 = 3;LP2 = 4;}
    else {LP1 = 4;LP2 = 4;}
    ls_div = r;

    const uint8_t r0 = (LP1<<4)|LP2;
    const uint8_t r5 = M_frac&0xff;
    const uint8_t r6 = M_frac>>8&0xff;
    const uint8_t r7 = M_frac>>16&0xff;
    const uint8_t r8 = (M_frac>>24&0x1f)|((M_int&0x07)<<5);
    const uint8_t r9 = M_int>>3&0x3f;
    const uint8_t r10 = hs_div&0xff;
    const uint8_t r11 = (hs_div>>8&0x03)|(ls_div<<4&0x70);

    // disable output
    Wire.beginTransmission(_I2Caddress);
    Wire.write(132);
    Wire.write(SI514_OUTPUT_DISABLE_132);
    Wire.endTransmission();

    // write registers
    Wire.beginTransmission(_I2Caddress);
    Wire.write(0x00);
    Wire.write(r0);
    Wire.endTransmission();
    Wire.beginTransmission(_I2Caddress);
    Wire.write(0x05);
    Wire.write(r5);
    Wire.write(r6);
    Wire.write(r7);
    Wire.write(r8);
    Wire.write(r9);
    Wire.write(r10);
    Wire.write(r11);
    Wire.endTransmission();

    // calibrate
    Wire.beginTransmission(_I2Caddress);
    Wire.write(132);
    Wire.write(SI514_CALIBRATE_132);
    Wire.endTransmission();
    delayMS(10);

    // enable output
    Wire.beginTransmission(_I2Caddress);
    Wire.write(132);
    Wire.write(SI514_OUTPUT_ENABLE_132);
    Wire.endTransmission();
  }
  else
  {
    // Mnew = Mcurrent x Fout_new / Fout_current
    const uint64_t n = m*f/centre_frequency;
    const uint32_t M_frac = (n>>3)&0x1fffffff;
    const uint16_t M_int = (n>>32)&0xff;
    const uint8_t r5 = M_frac&0xff;
    const uint8_t r6 = M_frac>>8&0xff;
    const uint8_t r7 = M_frac>>16&0xff;
    const uint8_t r8 = (M_frac>>24&0x1f)|((M_int&0x07)<<5);
    const uint8_t r9 = M_int>>3&0x3f;

    // write new M only
    Wire.beginTransmission(_I2Caddress);
    Wire.write(0x05);
    Wire.write(r5);
    Wire.write(r6);
    Wire.write(r7);
    Wire.write(r8);
    Wire.write(r9);
    Wire.endTransmission();
  }
}


//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
//=======================================================================================================
/*
#ifdef DEFAULT_SPI_FREQ
#undef DEFAULT_SPI_FREQ
#define DEFAULT_SPI_FREQ 16000000
#error got here 2!
#endif
*/

#define TFT_CS        A1
#define TFT_RST       10
#define TFT_DC         1
//#define TFT_MOSI      11
//#define TFT_SCLK      13

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// SI514 silicon oscillator
Si514 so;

TCA9534 LPF;
TCA9534 L;
TCA9534 C;

static Rotary r = Rotary(ENCB_PIN,ENCA_PIN);

// ADC interrupt
ISR(ADC_vect)
{
  const int16_t v = ADC;

  // CW sidetone frequency = 250000/13/2/(CW tone counter)
  // for count of 16 CW tone = 250000/13/2/16 = 601
  if (cw_tone_enable && (p&0x0f)==0)
  {
    PORTD ^= CW_PORT_PIN;
  }

  if (p==254) sense = v;
  else if (p==255) sigmon = v;
////
  else if (!buffer_full) fht_input[p] = (v-512)<<6;
  //else if (!buffer_full) fht_input[p] = v<<6;

  if (p==252) ADMUX = bit(REFS0)|(SENSE_ADC&0x0f);
  else if (p==253) ADMUX = bit(REFS0)|(SIGMON_ADC&0x0f);
  else ADMUX = bit(REFS0)|(SCAN_ADC&0x0f);

  p++;

  switch (r.process())
  {
    case DIR_CW:  knob++; break;
    case DIR_CCW: knob--; break;
  }

  if (p==0)
  {
    if (processing_complete)
    {
      // the processing has completed
      // collect new values and
      // reset the processing flag
      buffer_full = false;
      processing_complete = false;
    }
    else
    {
      // not processing, so indicate the buffer is full
      // and then it can be processed
      buffer_full = true;
    }

    // S meter decay counter
    if (scounter>0) scounter--;
  }
}

////
/*
#define FILTER1_PIN D9 // digital output (PWM)
#define FILTER2_PIN D3 // digital output (PWM)
#define MIXER_PIN   D5 // digital output (PWM)
#define TX_PIN      D2 // digital output
#define ES_PIN      D7 // digital output
#define CW_PIN      D8 // digital output
#define MUTE_PIN    D4 // digital output
#define CW_TONE_PIN D5 // digital output (tone)
#define SENSE_PIN   A7 // analog input (PTT, Paddle)
#define SIGMON_PIN  A6 // analog input (S Meter)
#define ENCBUT_PIN  D0 // digital input (with pullup)
#define ENCB_PIN    A3 // digital input (with pullup)
#define ENCA_PIN    A2 // digital input (with pullup)
#define SCAN_PIN    A0 // analog input (spectrum)
*/
void setup(void)
{
  //Serial.begin(9600);
  //Serial.print(F("Hello! ST77xx TFT Test"));

  pinMode(MUTE_PIN,OUTPUT);
  pinMode(FILTER1_PIN,OUTPUT);
  pinMode(FILTER2_PIN,OUTPUT);
  pinMode(MIXER_PIN,OUTPUT);
  pinMode(TX_PIN,OUTPUT);
  pinMode(ES_PIN,OUTPUT);
  pinMode(CW_PIN,OUTPUT);
  pinMode(CW_TONE_PIN,OUTPUT);
  pinMode(ENCBUT_PIN,INPUT_PULLUP);

  pinMode(SENSE_PIN,INPUT);
  pinMode(SIGMON_PIN,INPUT);
  pinMode(SCAN_PIN,INPUT);
  digitalWrite(MUTE_PIN,HIGH);
  digitalWrite(TX_PIN,LOW);
  digitalWrite(ES_PIN,LOW);
  digitalWrite(CW_PIN,LOW);
  digitalWrite(CW_TONE_PIN,LOW);
  digitalWrite(FILTER1_PIN,LOW);
  digitalWrite(FILTER2_PIN,LOW);
  digitalWrite(MIXER_PIN,LOW);

  // low pass filter and attenuator
  LPF.attach(Wire);
  LPF.setDeviceAddress(0x38);
  LPF.config(TCA9534::Config::OUT);
  LPF.polarity(TCA9534::Polarity::ORIGINAL);
  LPF.output(TCA9534::Level::L);

  // L network
  L.attach(Wire);
  L.setDeviceAddress(0x39);
  L.config(TCA9534::Config::OUT);
  L.polarity(TCA9534::Polarity::ORIGINAL);
  L.output(TCA9534::Level::L);

  // C network
  C.attach(Wire);
  C.setDeviceAddress(0x3A);
  C.config(TCA9534::Config::OUT);
  C.polarity(TCA9534::Polarity::ORIGINAL);
  C.output(TCA9534::Level::L);

////
/*
  InitTimersSafe();
  SetPinFrequency(FILTER1_PIN,radio.filter1*100UL);
  pwmWrite(FILTER1_PIN,128);
  SetPinFrequency(FILTER2_PIN,radio.filter2*100UL);
  pwmWrite(FILTER2_PIN,128);

  delayMS(5000);

  InitTimers();
  SetPinFrequency(FILTER1_PIN,radio.filter1*100UL);
  pwmWrite(FILTER1_PIN,128);
  SetPinFrequency(FILTER2_PIN,radio.filter2*100UL);
  pwmWrite(FILTER2_PIN,128);
  SetPinFrequency(MIXER_PIN,DEFAULT_MIXER);
  pwmWrite(MIXER_PIN,128);

  for (;;);
*/
  // Init ST7789 240x135
  tft.init(135,240);
  tft.fillScreen(ST77XX_BLACK);

  so.setFrequency(radio.frequency);
/*
  for (uint32_t f = 10000000;f<=10010000;f+=1000)
  {
    so.setFrequency(f);
    delayMS(2000);
  }
*/

  tft.setRotation(3);
  init_options();
  show_frequency();
  show_mode();
  show_tuning_step(0);
  show_meter_dial();

  // ADC setup
  // sampling rate will be 250,000/13 = 19,230Hz
  ADCSRA =  bit(ADEN);                      // enable the ADC
  ADCSRA |= bit(ADPS1)|bit(ADPS2);          // prescaler of 64 (250KHz)
  ADMUX  =  bit(REFS0)|(SCAN_ADC&0x0f);     // set ref to AVCC and select input port
  ADCSRA |= bit(ADATE)|bit(ADSC)|bit(ADIE); // enable continuous mode ADC interrupt
/*
    for (uint8_t r=15;r>0;r--)
    {
      for (uint8_t c=0;c<60;c++)
      {
        water[r][c] = water[r-1][c];
      }
    }
    for (uint8_t c=0;c<60;c++)
    {
      water[0][c] = ((random(16)&0x0f)<<4)|(random(16)&0x0f);;
    }

    //setAddrWindow(x, y, 1, 1);
    //SPI_WRITE16(color);
    //    AVR_WRITESPI(w >> 8);
    //    AVR_WRITESPI(w);


    //for (uint8_t r=0,row=135-32;r<16;r++)
    for (uint8_t r=15,row=134;r!=255;r--)
    {
      for (uint8_t c=0,col=0;c<60;c++)
      {
        const uint8_t w = water[r][c];
        const uint16_t p1 = color_map[w&0x0f];
        const uint16_t p2 = color_map[(w>>4)&0x0f];
        tft.drawPixel(col++,row,p1);
        tft.drawPixel(col++,row,p1);
        tft.drawPixel(col++,row,p2);
        tft.drawPixel(col++,row,p2);
      }
      row--;
      for (uint8_t c=0,col=0;c<60;c++)
      {
        const uint8_t w = water[r][c];
        const uint16_t p1 = color_map[w&0x0f];
        const uint16_t p2 = color_map[(w>>4)&0x0f];
        tft.drawPixel(col++,row,p1);
        tft.drawPixel(col++,row,p1);
        tft.drawPixel(col++,row,p2);
        tft.drawPixel(col++,row,p2);
      }
      row--;
    }

    //delayMS(100);
*/
}

/*
  for (uint8_t row=135-32;row<135;row++)
  {
    for (uint8_t col=0;col<240;col++)
    {
      //tft.drawPixel(col,row,ST77XX_GREEN);
      tft.drawPixel(col,row,water[row][col]);
    }
  }
*/

static void show_frequency(void)
{
  tft.setTextSize(3);
  tft.setTextColor(radio.locked?ST77XX_RED:ST77XX_WHITE,ST77XX_BLACK);
  tft.setCursor(POS_FREQUENCY_X,POS_FREQUENCY_Y);
  if (radio.frequency<10000000UL) tft.print(F(" "));
  if (radio.frequency<1000000UL) tft.print(F(" "));
  tft.print(radio.frequency);
}

static void show_meter_dial(void)
{
  tft.setTextSize(1);
  tft.setCursor(POS_METER_X,POS_METER_Y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(F("1 3 5 7 9 +20"));

  tft.setTextSize(1);
  tft.setCursor(POS_TUNING_STEP_X-30,POS_TUNING_STEP_Y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(F("STEP"));

}

static void cw_tone_on(void)
{
  cw_tone_enable = true;
}

static void cw_tone_off(void)
{
  cw_tone_enable = false;
}

static void smeter(void)
{
  // get the value
  noInterrupts();
  uint16_t sv = sigmon;
  interrupts();

  // remove the DC offset and get the absolute value
////
  sv >>= 1;
/*
  if (sv==0)
  {
    sv = 511;
  }
  else
  {
     sv -= 512;
     sv = abs((int16_t)sv);
  }
*/
  // take the log2 of the signal level
  uint8_t l = 0;
  while (sv>>=1) l++;

  // has the level increased?
  static uint8_t peak = 0;
  if (l>peak)
  {
    // level exceeded, reset the decay counter
    peak = l;
    scounter = METER_DECAY_RATE;
  }
  else if (l<peak)
  {
    // level not exceeded peak so decrease level
    // after the decay period
    if (peak>0)
    {
      if (scounter==0)
      {
        peak--;
        scounter = METER_DECAY_RATE;
      }
    }
  }

  // has the value changed?
  static uint8_t v = 0;
  if (peak==v) return;
  v = peak;

  // display peak value as S meter
  tft.startWrite();
  for (uint8_t i=0;i<9;i++)
  {
    uint16_t color = ST77XX_BLACK;
    if (i+1<=peak) color = ST77XX_WHITE;

    tft.setAddrWindow(POS_METER_X+i*10+0,POS_METER_Y+8,4,4);
    for (uint8_t j=0;j<16;j++)
    {
      WRITESPI(color>>8);
      WRITESPI(color&0xff);
    }


////
    // plot a dot (or box) at X+(i*3),Y using “colour”
    //tft.drawFastVLine(POS_METER_X+i*10+0,POS_METER_Y+8,4,color);
    //tft.drawFastVLine(POS_METER_X+i*10+1,POS_METER_Y+8,4,color);
    //tft.drawFastVLine(POS_METER_X+i*10+2,POS_METER_Y+8,4,color);
    //tft.drawFastVLine(POS_METER_X+i*10+3,POS_METER_Y+8,4,color);
  }
  tft.endWrite();
}

static void show_tuning_step(const uint32_t current_tuning_step)
{
  tft.setTextSize(1);
  if (current_tuning_step!=0)
  {
    tft.setCursor(POS_TUNING_STEP_X,POS_TUNING_STEP_Y);
    tft.setTextColor(ST77XX_BLACK);
    tft.print(current_tuning_step);
  }
  tft.setCursor(POS_TUNING_STEP_X,POS_TUNING_STEP_Y);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(radio.tuning_step);
}

static void show_mode(void)
{
  tft.fillRect(POS_MODE_X-5,POS_MODE_Y-5,45,25,ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(POS_MODE_X,POS_MODE_Y);
  switch (radio.mode)
  {
    case MODE_LSB:
    {
      tft.print(F("LSB"));
      break;
    }
    case MODE_USB:
    {
      tft.print(F("USB"));
      break;
    }
    case MODE_CWU:
    {
      tft.print(F("CWU"));
      break;
    }
    case MODE_CWL:
    {
      tft.print(F("CWL"));
      break;
    }
    case MODE_DIGITAL:
    {
      tft.print(F("DIG"));
      break;
    }
  }
}

static void waterfall(void)
{
/*
    tft.startWrite();
    tft.setAddrWindow(POS_WATER_X,POS_WATER_Y-16,16,2);
    for (uint8_t i=0;i<2;i++)
    {
      WRITESPI(ST77XX_WHITE>>8);
      WRITESPI(ST77XX_WHITE&0xff);
      WRITESPI(ST77XX_WHITE>>8);
      WRITESPI(ST77XX_WHITE&0xff);
      WRITESPI(ST77XX_WHITE>>8);
      WRITESPI(ST77XX_WHITE&0xff);
      WRITESPI(ST77XX_WHITE>>8);
      WRITESPI(ST77XX_WHITE&0xff);
      WRITESPI(ST77XX_GREEN>>8);
      WRITESPI(ST77XX_GREEN&0xff);
      WRITESPI(ST77XX_GREEN>>8);
      WRITESPI(ST77XX_GREEN&0xff);
      WRITESPI(ST77XX_GREEN>>8);
      WRITESPI(ST77XX_GREEN&0xff);
      WRITESPI(ST77XX_GREEN>>8);
      WRITESPI(ST77XX_GREEN&0xff);
    }
    for (uint8_t i=0;i<2;i++)
    {
      WRITESPI(ST77XX_WHITE>>8);
      WRITESPI(ST77XX_WHITE&0xff);
      WRITESPI(ST77XX_WHITE>>8);
      WRITESPI(ST77XX_WHITE&0xff);
      WRITESPI(ST77XX_WHITE>>8);
      WRITESPI(ST77XX_WHITE&0xff);
      WRITESPI(ST77XX_WHITE>>8);
      WRITESPI(ST77XX_WHITE&0xff);
      WRITESPI(ST77XX_GREEN>>8);
      WRITESPI(ST77XX_GREEN&0xff);
      WRITESPI(ST77XX_GREEN>>8);
      WRITESPI(ST77XX_GREEN&0xff);
      WRITESPI(ST77XX_GREEN>>8);
      WRITESPI(ST77XX_GREEN&0xff);
      WRITESPI(ST77XX_GREEN>>8);
      WRITESPI(ST77XX_GREEN&0xff);
    }
    tft.endWrite();
*/

/*
    tft.startWrite();
    for (uint8_t r=0;r<120;r++)
    {
      //tft.setAddrWindow(POS_WATER_X+r*2,POS_WATER_Y-16,2,mag);
      tft.setAddrWindow(POS_WATER_X+r*2,POS_WATER_Y-16,2,8);
      WRITESPI(ST77XX_WHITE>>8);
      WRITESPI(ST77XX_WHITE&0xff);
    }
    tft.endWrite();
*/
  if (buffer_full && !processing_complete)
  {
    fht_input[254] = 0;
    fht_input[255] = 0;
    fht_window();
    fht_reorder();
    fht_run();
    fht_mag_log();
    processing_complete = true;

/*
    tft.startWrite();
    for (uint8_t r=0;r<120;r++)
    {
      tft.setAddrWindow(r<<1,POS_WATER_Y,1,31);
      uint8_t mag = fht_log_out[r]>>2;
      if (mag>31) mag = 31;
      for (int8_t i=31;i>0;i--)
      {
        if (i>mag)
        {
          WRITESPI(ST77XX_BLACK>>8);
          WRITESPI(ST77XX_BLACK&0xff);
          continue;
        }
        WRITESPI(ST77XX_WHITE>>8);
        WRITESPI(ST77XX_WHITE&0xff);
      }
    }
    tft.endWrite();
*/
///*
    tft.startWrite();
    for (uint8_t r=0;r<240;r+=2)
    {
      tft.setAddrWindow(r,POS_WATER_Y,1,31);
      uint8_t mag = fht_log_out[r>>1]>>2;
      if (mag>31) mag = 31;
      for (uint8_t i=31;i>0;i--)
      {
        if (i>mag)
        {
          WRITESPI(ST77XX_BLACK>>8);
          //WRITESPI(ST77XX_BLACK&0xff);
          // can knock off 7 cycles
          WRITERAW(ST77XX_BLACK&0xff);
          asm volatile("rjmp .+0\n");
          asm volatile("rjmp .+0\n");
          asm volatile("rjmp .+0\n");
          asm volatile("rjmp .+0\n");
          asm volatile("rjmp .+0\n");
          continue;
        }
        WRITESPI(ST77XX_WHITE>>8);
        //WRITESPI(ST77XX_WHITE&0xff);
        // can knock off 7 cycles
        WRITERAW(ST77XX_WHITE&0xff);
        asm volatile("rjmp .+0\n");
        asm volatile("rjmp .+0\n");
        asm volatile("rjmp .+0\n");
        asm volatile("rjmp .+0\n");
        asm volatile("rjmp .+0\n");
      }
    }
    tft.endWrite();
//*/

/*
    tft.startWrite();
    for (uint8_t r=0;r<240;r+=2)
    {
      //const uint8_t mag = fht_log_out[r]>>4;
      tft.setAddrWindow(r,POS_WATER_Y,1,32);
      uint8_t mag = fht_log_out[r>>1]>>2;
      if (mag>31) mag = 31;
      uint8_t pixels = 31-mag;
      if (pixels)
      {
        for (uint8_t i=0;i<pixels;i++)
        {
          WRITESPI(ST77XX_BLACK>>8);
          WRITESPI(ST77XX_BLACK&0xff);
        }
      }
      pixels = mag;
      for (uint8_t i=0;i<pixels;i++)
      {
        WRITESPI(ST77XX_WHITE>>8);
        WRITESPI(ST77XX_WHITE&0xff);
      }
      //tft.drawFastVLine(POS_WATER_X+r*2,POS_WATER_Y,15-mag,ST77XX_BLACK);
      //tft.drawFastVLine(POS_WATER_X+r*2,POS_WATER_Y+15-mag,mag,ST77XX_WHITE);
      //tft.setAddrWindow(POS_WATER_X+r*2,POS_WATER_Y-16,2,mag);
      //WRITESPI(ST77XX_WHITE>>8);
      //WRITESPI(ST77XX_WHITE&0xff);
    }
    tft.endWrite();
*/

    // display waterfall!
    for (uint8_t r=0,i=0;r<60;r++)
    {
      uint8_t m1 = fht_log_out[i++]>>2;
      uint8_t m2 = fht_log_out[i++]>>2;
      if (m1>15) m1 = 15;
      if (m2>15) m2 = 15;
      water[wp][r] = (m1<<4)|m2;
    }
    int8_t r = wp;
    uint8_t y = POS_WATER_Y+32;
    tft.startWrite();
    for (uint8_t i=0;i<10;i++)
    {
      uint8_t x = POS_WATER_X;
      for (uint8_t j=0;j<60;j++)
      {
        const uint16_t c1 = color_map[water[r][j]>>4];
        const uint16_t c2 = color_map[water[r][j]&0x0f];
        uint16_t hb = c1>>8;
        uint16_t lb = c1&0xff;
        //tft.fillRect(x,y,2,2,c1);
        tft.setAddrWindow(x,y,2,2);
        WRITESPI(hb);
        WRITESPI(lb);
        WRITESPI(hb);
        WRITESPI(lb);
        WRITESPI(hb);
        WRITESPI(lb);
        WRITESPI(hb);
        WRITERAW(lb);
        x += 2;
        //tft.fillRect(x,y,2,2,c2);
        hb = c2>>8;
        lb = c2&0xff;
        tft.setAddrWindow(x,y,2,2);
        WRITESPI(hb);
        WRITESPI(lb);
        WRITESPI(hb);
        WRITESPI(lb);
        WRITESPI(hb);
        WRITESPI(lb);
        WRITESPI(hb);
        WRITERAW(lb);
        x += 2;
      }
      y += 2;
      r--;
      if (r<0) r = 9;
    }
    tft.endWrite();
    wp++;
    if (wp>=10) wp = 0;

/*
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);
  tft.setCursor(0,123);
  tft.print(fht_log_out[8]);
  tft.print("   ");
  tft.drawFastVLine(0,124,10,ST77XX_WHITE);
*/
  }
}

static void show_option(const uint8_t x,const uint8_t y,const __FlashStringHelper *s)
{
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(x+5,y+5);
  tft.print(s);
  tft.drawRect(x,y,46,24,ST77XX_RED);

}

/*
#define POS_OPTION_CWU_X    1
#define POS_OPTION_CWU_Y   82
#define POS_OPTION_CWL_X   49
#define POS_OPTION_CWL_Y   82
#define POS_OPTION_DSB_X   97
#define POS_OPTION_DSB_Y   82
#define POS_OPTION_BND_X  145
#define POS_OPTION_BND_Y   82
#define POS_OPTION_TUN_X  193
#define POS_OPTION_TUN_Y   82

#define POS_OPTION_ATT_X    1
#define POS_OPTION_ATT_Y  108
#define POS_OPTION_RBW_X   49
#define POS_OPTION_RBW_Y  108
#define POS_OPTION_TBW_X   97
#define POS_OPTION_TBW_Y  108
#define POS_OPTION_RIT_X  145
#define POS_OPTION_RIT_Y  108
#define POS_OPTION_LCK_X  193
#define POS_OPTION_LCK_Y  108
*/
static void init_options(void)
{
  show_option(POS_OPTION_LSB_X,POS_OPTION_LSB_Y,F("LSB"));
  show_option(POS_OPTION_USB_X,POS_OPTION_USB_Y,F("USB"));
  show_option(POS_OPTION_DIG_X,POS_OPTION_DIG_Y,F("DIG"));
  show_option(POS_OPTION_STP_X,POS_OPTION_STP_Y,F("STP"));
  show_option(POS_OPTION_SCP_X,POS_OPTION_SCP_Y,F("SCP"));
  show_option(POS_OPTION_CWU_X,POS_OPTION_CWU_Y,F("CWU"));
  show_option(POS_OPTION_CWL_X,POS_OPTION_CWL_Y,F("CWL"));
  show_option(POS_OPTION_DSB_X,POS_OPTION_DSB_Y,F("DSB"));
  show_option(POS_OPTION_BND_X,POS_OPTION_BND_Y,F("BND"));
  show_option(POS_OPTION_TUN_X,POS_OPTION_TUN_Y,F("TUN"));
  show_option(POS_OPTION_ATT_X,POS_OPTION_ATT_Y,F("ATT"));
  show_option(POS_OPTION_RBW_X,POS_OPTION_RBW_Y,F("RBW"));
  show_option(POS_OPTION_TBW_X,POS_OPTION_TBW_Y,F("TBW"));
  show_option(POS_OPTION_RIT_X,POS_OPTION_RIT_Y,F("RIT"));
  show_option(POS_OPTION_LCK_X,POS_OPTION_LCK_Y,F("LCK"));
  last_option = 15;
}

void clear_option(const int8_t option)
{
  switch (option)
  {
    case OPTION_MODE_LSB:
    {
      tft.drawRect(POS_OPTION_LSB_X,POS_OPTION_LSB_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_MODE_USB:
    {
      tft.drawRect(POS_OPTION_USB_X,POS_OPTION_USB_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_MODE_DIG:
    {
      tft.drawRect(POS_OPTION_DIG_X,POS_OPTION_DIG_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_STEP:
    {
      tft.drawRect(POS_OPTION_STP_X,POS_OPTION_STP_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_SCP:
    {
      tft.drawRect(POS_OPTION_SCP_X,POS_OPTION_SCP_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_MODE_CWU:
    {
      if (waterfall_enabled) break;
      tft.drawRect(POS_OPTION_CWU_X,POS_OPTION_CWU_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_MODE_CWL:
    {
      tft.drawRect(POS_OPTION_CWL_X,POS_OPTION_CWL_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_DSB:
    {
      tft.drawRect(POS_OPTION_DSB_X,POS_OPTION_DSB_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_BND:
    {
      tft.drawRect(POS_OPTION_BND_X,POS_OPTION_BND_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_TUN:
    {
      tft.drawRect(POS_OPTION_TUN_X,POS_OPTION_TUN_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_ATT:
    {
      tft.drawRect(POS_OPTION_ATT_X,POS_OPTION_ATT_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_RBW:
    {
      tft.drawRect(POS_OPTION_RBW_X,POS_OPTION_RBW_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_TBW:
    {
      tft.drawRect(POS_OPTION_TBW_X,POS_OPTION_TBW_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_RIT:
    {
      tft.drawRect(POS_OPTION_RIT_X,POS_OPTION_RIT_Y,46,24,ST77XX_RED);
      break;
    }
    case OPTION_LCK:
    {
      tft.drawRect(POS_OPTION_LCK_X,POS_OPTION_LCK_Y,46,24,ST77XX_RED);
      break;
    }
  }
}

static void highlight_option(const int8_t option)
{
  //tft.setTextSize(2);
  //tft.setTextColor(ST77XX_WHITE);
  switch (option)
  {
    case OPTION_MODE_LSB:
    {
      //tft.setCursor(POS_OPTION_LSB_X,POS_OPTION_LSB_Y);
      //tft.print("LSB");
      tft.drawRect(POS_OPTION_LSB_X,POS_OPTION_LSB_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_MODE_USB:
    {
      //tft.setCursor(POS_OPTION_USB_X,POS_OPTION_USB_Y);
      //tft.print("USB");
      tft.drawRect(POS_OPTION_USB_X,POS_OPTION_USB_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_MODE_DIG:
    {
      //tft.setCursor(POS_OPTION_DIG_X,POS_OPTION_DIG_Y);
      //tft.print("DIG");
      tft.drawRect(POS_OPTION_DIG_X,POS_OPTION_DIG_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_STEP:
    {
      //tft.setCursor(POS_OPTION_STP_X,POS_OPTION_STP_Y);
      //tft.print("STP");
      tft.drawRect(POS_OPTION_STP_X,POS_OPTION_STP_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_SCP:
    {
      //tft.setCursor(POS_OPTION_SCP_X,POS_OPTION_SCP_Y);
      //tft.print("STP");
      tft.drawRect(POS_OPTION_SCP_X,POS_OPTION_SCP_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_MODE_CWU:
    {
      if (waterfall_enabled) break;
      tft.drawRect(POS_OPTION_CWU_X,POS_OPTION_CWU_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_MODE_CWL:
    {
      tft.drawRect(POS_OPTION_CWL_X,POS_OPTION_CWL_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_DSB:
    {
      tft.drawRect(POS_OPTION_DSB_X,POS_OPTION_DSB_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_BND:
    {
      tft.drawRect(POS_OPTION_BND_X,POS_OPTION_BND_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_TUN:
    {
      tft.drawRect(POS_OPTION_TUN_X,POS_OPTION_TUN_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_ATT:
    {
      tft.drawRect(POS_OPTION_ATT_X,POS_OPTION_ATT_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_RBW:
    {
      tft.drawRect(POS_OPTION_RBW_X,POS_OPTION_RBW_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_TBW:
    {
      tft.drawRect(POS_OPTION_TBW_X,POS_OPTION_TBW_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_RIT:
    {
      tft.drawRect(POS_OPTION_RIT_X,POS_OPTION_RIT_Y,46,24,ST77XX_WHITE);
      break;
    }
    case OPTION_LCK:
    {
      tft.drawRect(POS_OPTION_LCK_X,POS_OPTION_LCK_Y,46,24,ST77XX_WHITE);
      break;
    }
  }
}

void loop(void)
{
  static int8_t k = 0;
  for (;;)
  {
    k = 0;
    if (knob!=0)
    {
      noInterrupts();
      k = knob;
      knob = 0;
      interrupts();
    }

    smeter();
    if (waterfall_enabled && !radio.tx)
    {
      waterfall();
    }


////
/*
    tft.fillRect(0,90,100,20,ST77XX_BLACK);
    tft.setCursor(0,90);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(peak);
    tft.print(" ");
    tft.print(sigmon);

    tft.fillRect(0,90,239,20,ST77XX_BLACK);
    tft.setCursor(0,90);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(buf[0]);
    tft.print(", ");
    tft.print(buf[1]);
    tft.print(", ");
    tft.print(buf[2]);
    tft.print(", ");
    tft.print(buf[3]);

    tft.fillRect(0,120,239,20,ST77XX_BLACK);
    tft.setCursor(0,120);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(buf[4]);
    tft.print(", ");
    tft.print(buf[5]);
    tft.print(", ");
    tft.print(buf[6]);
    tft.print(", ");
    tft.print(buf[7]);

    tft.print(", ");
    tft.print(sigmon);
    tft.print(", ");
    tft.print(sense);
*/
/*
  #define FILTER1_PIN D9 // digital output (PWM)
  #define FILTER2_PIN D3 // digital output (PWM)
  #define MIXER_PIN   D5 // digital output (PWM)
  #define TX_PIN      D2 // digital output
  #define ES_PIN      D7 // digital output
  #define CW_PIN      D8 // digital output
  #define MUTE_PIN    D4 // digital output
  #define CW_TONE_PIN D5 // digital output (tone)
  #define SENSE_PIN   A7 // analog input (PTT, Paddle)
  #define SIGMON_PIN  A6 // analog input (S Meter)
  #define ENCBUT_PIN  D0 // digital input (with pullup)
  #define ENCB_PIN    A3 // digital input (with pullup)
  #define ENCA_PIN    A2 // digital input (with pullup)
  #define SCAN_PIN    A0 // analog input (spectrum)
*/
    switch (radio_state)
    {
      case STATE_SSB_RECEIVE_INIT:
      {
        radio.tx = false;
        digitalWrite(TX_PIN,LOW);
        //init();
        tft.setTextSize(2);
        tft.setCursor(POS_TX_X,POS_TX_Y);
        tft.setTextColor(ST77XX_RED);
        tft.print(F("TX"));
        tft.setCursor(POS_RX_X,POS_RX_Y);
        tft.setTextColor(ST77XX_WHITE);
        tft.print(F("RX"));
        so.setFrequency(radio.frequency);
        pinMode(MIXER_PIN,OUTPUT);
        digitalWrite(MIXER_PIN,LOW);
        digitalWrite(ES_PIN,LOW);
        digitalWrite(CW_PIN,LOW);
        digitalWrite(CW_TONE_PIN,LOW);
        InitTimersSafe();
        SetPinFrequencySafe(FILTER1_PIN,(uint32_t)radio.rxfilter1*100UL);
        pwmWrite(FILTER1_PIN,128);
        SetPinFrequencySafe(FILTER2_PIN,(uint32_t)radio.rxfilter2*100UL);
        pwmWrite(FILTER2_PIN,128);
        digitalWrite(MUTE_PIN,LOW);
        radio_state = STATE_SSB_RECEIVE;
        delayMS(30); // debounce PTT
        break;
      }
      case STATE_SSB_RECEIVE:
      {
        if (k!=0 && !radio.locked)
        {
          uint32_t new_frequency = radio.frequency+radio.tuning_step*k;
          if (new_frequency<MIN_FREQUENCY||new_frequency>MAX_FREQUENCY)
          {
            if (k<0) new_frequency = MIN_FREQUENCY;
            else new_frequency = MAX_FREQUENCY;
          }
          if (radio.frequency!=new_frequency)
          {
            radio.frequency = new_frequency;
            so.setFrequency(radio.frequency);
            show_frequency();
          }
        }
        if (digitalRead(ENCBUT_PIN)==LOW)
        {
          // pressed encoder button
          radio_state = STATE_OPTION_INIT;
          break;
        }
        if (sense<PTT_THRESHOLD)
        {
          switch (radio.mode)
          {
            case MODE_LSB:
            {
              radio_state = STATE_LSB_TX_INIT;
              break;
            }
            case MODE_USB:
            {
              radio_state = STATE_USB_TX_INIT;
              break;
            }
          }
        }
        break;
      }
      case STATE_CW_RECEIVE_INIT:
      {
        radio.tx = false;
        digitalWrite(TX_PIN,LOW);
        digitalWrite(CW_PIN,LOW);
        //init();
        tft.setTextSize(2);
        tft.setCursor(POS_TX_X,POS_TX_Y);
        tft.setTextColor(ST77XX_RED);
        tft.print(F("TX"));
        tft.setCursor(POS_RX_X,POS_RX_Y);
        tft.setTextColor(ST77XX_WHITE);
        tft.print(F("RX"));
        if (radio.mode==MODE_CWL)
        {
          so.setFrequency(radio.frequency+CW_TONE);
        }
        else
        {
          so.setFrequency(radio.frequency-CW_TONE);
        }
        pinMode(MIXER_PIN,OUTPUT);
        digitalWrite(MIXER_PIN,LOW);
        digitalWrite(ES_PIN,LOW);
        digitalWrite(CW_TONE_PIN,LOW);
        InitTimersSafe();
        SetPinFrequencySafe(FILTER1_PIN,(uint32_t)radio.rxfilter1*100UL);
        pwmWrite(FILTER1_PIN,128);
        SetPinFrequencySafe(FILTER2_PIN,(uint32_t)radio.rxfilter2*100UL);
        pwmWrite(FILTER2_PIN,128);
        digitalWrite(MUTE_PIN,LOW);
        radio_state = STATE_CW_RECEIVE;
        delayMS(30); // debounce key
        break;
      }
      case STATE_CW_RECEIVE:
      {
        if (k!=0 && !radio.locked)
        {
          uint32_t new_frequency = radio.frequency+radio.tuning_step*k;
          if (new_frequency<MIN_FREQUENCY||new_frequency>MAX_FREQUENCY)
          {
            if (k<0) new_frequency = MIN_FREQUENCY;
            else new_frequency = MAX_FREQUENCY;
          }
          if (radio.frequency!=new_frequency)
          {
            radio.frequency = new_frequency;
            if (radio.mode==MODE_CWL)
            {
              so.setFrequency(radio.frequency+CW_TONE);
            }
            else
            {
              so.setFrequency(radio.frequency-CW_TONE);
            }
            show_frequency();
          }
        }
        if (digitalRead(ENCBUT_PIN)==LOW)
        {
          // pressed encoder button
          radio_state = STATE_OPTION_INIT;
          break;
        }
        if (sense<PTT_THRESHOLD)
        {
          radio_state = STATE_CW_TX_INIT;
        }
        break;
      }
      case STATE_LSB_TX_INIT:
      {
        digitalWrite(MUTE_PIN,HIGH);
        so.setFrequency(radio.frequency-DEFAULT_MIXER_ACTUAL);
        tft.setTextSize(2);
        tft.setCursor(POS_TX_X,POS_TX_Y);
        tft.setTextColor(ST77XX_WHITE);
        tft.print(F("TX"));
        tft.setCursor(POS_RX_X,POS_RX_Y);
        tft.setTextColor(ST77XX_RED);
        tft.print(F("RX"));
        InitTimers();
        SetPinFrequency(FILTER1_PIN,(uint32_t)radio.txfilter1*100UL);
        pwmWrite(FILTER1_PIN,128);
        SetPinFrequency(FILTER2_PIN,(uint32_t)radio.txfilter2*100UL);
        pwmWrite(FILTER2_PIN,128);
        SetPinFrequency(MIXER_PIN,DEFAULT_MIXER);
        pwmWrite(MIXER_PIN,128);
        digitalWrite(CW_PIN,LOW);
        digitalWrite(CW_TONE_PIN,LOW);
        digitalWrite(TX_PIN,HIGH);
        radio.tx = true;
        radio_state = STATE_LSB_TX;
        delayMS(30); // debounce PTT
        break;
      }
      case STATE_LSB_TX:
      {
        // wait for PTT to go high (stop TX)
        if (sense<PTT_THRESHOLD)
        {
          // PTT still in effect
          break;
        }
        // go back to receive
        radio_state = STATE_SSB_RECEIVE_INIT;
        break;
      }
      case STATE_USB_TX_INIT:
      {
        digitalWrite(MUTE_PIN,HIGH);
        so.setFrequency(radio.frequency+DEFAULT_MIXER_ACTUAL);
        tft.setTextSize(2);
        tft.setCursor(POS_TX_X,POS_TX_Y);
        tft.setTextColor(ST77XX_WHITE);
        tft.print(F("TX"));
        tft.setCursor(POS_RX_X,POS_RX_Y);
        tft.setTextColor(ST77XX_RED);
        tft.print(F("RX"));
        InitTimers();
        SetPinFrequency(FILTER1_PIN,(uint32_t)radio.txfilter1*100UL);
        pwmWrite(FILTER1_PIN,128);
        SetPinFrequency(FILTER2_PIN,(uint32_t)radio.txfilter2*100UL);
        pwmWrite(FILTER2_PIN,128);
        SetPinFrequency(MIXER_PIN,DEFAULT_MIXER);
        pwmWrite(MIXER_PIN,128);
        digitalWrite(CW_PIN,LOW);
        digitalWrite(CW_TONE_PIN,LOW);
        digitalWrite(TX_PIN,HIGH);
        radio.tx = true;
        radio_state = STATE_USB_TX;
        delayMS(30); // debounce PTT
        break;
      }
      case STATE_USB_TX:
      {
        // wait for PTT to go high (stop TX)
        if (sense<PTT_THRESHOLD)
        {
          // PTT still in effect
          break;
        }
        // go back to receive
        radio_state = STATE_SSB_RECEIVE_INIT;
        break;
      }
      case STATE_CW_TX_INIT:
      {
        digitalWrite(MUTE_PIN,LOW);
        so.setFrequency(radio.frequency);
        cw_tone_on();
        digitalWrite(CW_PIN,HIGH);
        digitalWrite(CW_TONE_PIN,LOW);
        digitalWrite(TX_PIN,HIGH);
        radio.tx = true;
        tft.setTextSize(2);
        tft.setCursor(POS_TX_X,POS_TX_Y);
        tft.setTextColor(ST77XX_WHITE);
        tft.print(F("TX"));
        tft.setCursor(POS_RX_X,POS_RX_Y);
        tft.setTextColor(ST77XX_RED);
        tft.print(F("RX"));
        radio_state = STATE_CW_TX;
        delayMS(10); // debounce key
        break;
      }
      case STATE_CW_TX:
      {
        // wait for PTT to go high (stop TX)
        if (sense<PTT_THRESHOLD)
        {
          // PTT still in effect
          break;
        }
        // disable tone
        cw_tone_off();
        // go back to receive
        radio_state = STATE_CW_RECEIVE_INIT;
        break;
      }
      case STATE_OPTION_INIT:
      {
        // set the first option
        option = 0;
        highlight_option(option);
        radio_state = STATE_OPTION_WAIT;
        break;
      }
      case STATE_OPTION_WAIT:
      {
        // wait for encoder button release
        delayMS(30); // debounce button
        if (digitalRead(ENCBUT_PIN)==LOW)
        {
          // stay in this state until button released
          break;
        }
        radio_state = STATE_OPTION_SELECT;
        break;
      }
      case STATE_OPTION_SELECT:
      {
        if (k==0)
        {
          if (digitalRead(ENCBUT_PIN)==LOW)
          {
            radio_state = STATE_OPTION_EXIT;
            next_state = STATE_NO_STATE;
            switch (option)
            {
              case OPTION_MODE_LSB:
              {
                radio.mode = MODE_LSB;
                next_state = STATE_SSB_RECEIVE_INIT;
                break;
              }
              case OPTION_MODE_USB:
              {
                radio.mode = MODE_USB;
                next_state = STATE_SSB_RECEIVE_INIT;
                break;
              }
              case OPTION_MODE_CWL:
              {
                radio.mode = MODE_CWL;
                next_state = STATE_CW_RECEIVE_INIT;
                break;
              }
              case OPTION_MODE_CWU:
              {
                radio.mode = MODE_CWU;
                next_state = STATE_CW_RECEIVE_INIT;
                break;
              }
              case OPTION_MODE_DIG:
              {
                break;
              }
              case OPTION_STEP:
              {
                radio_state = STATE_STEP_INIT;
                break;
              }
              case OPTION_LCK:
              {
                radio.locked = !radio.locked;
                show_frequency();
                break;
              }
              case OPTION_SCP:
              {
                waterfall_enabled = !waterfall_enabled;
                tft.fillRect(0,82,240,53,ST77XX_BLACK);
                if (waterfall_enabled)
                {
                  last_option = 5;
                  break;
                }
                init_options();
                break;
              }
            }
          }
          break;
        }
        const int8_t previous_option = option;
        if (k>0)
        {
          // move to the next option
          option++;
          if (option>last_option)
          {
            option = 0;
          }
        }
        else
        {
          // move to the previous option
          option--;
          if (option<0)
          {
            option = last_option;
          }
        }
        clear_option(previous_option);
        highlight_option(option);
        break;
      }
      case STATE_OPTION_EXIT:
      {
        // the option has been selected
        // wait for encoder button release
        // return to the previous state
        // (unless explicitly changed)
        delayMS(30); // debounce button
        if (digitalRead(ENCBUT_PIN)==LOW)
        {
          // stay in this state until button released
          break;
        }
        show_mode();
        clear_option(option);
        if (next_state==STATE_NO_STATE)
        {
          switch (radio.mode)
          {
            case MODE_LSB: radio_state = STATE_SSB_RECEIVE; break;
            case MODE_USB: radio_state = STATE_SSB_RECEIVE; break;
            case MODE_CWU: radio_state = STATE_CW_RECEIVE;  break;
            case MODE_CWL: radio_state = STATE_CW_RECEIVE;  break;
            default: radio_state = STATE_SSB_RECEIVE_INIT;
          }
          break;
        }
        radio_state = next_state;
        break;
      }
      case STATE_STEP_INIT:
      {
        // the option has been selected
        // wait for encoder button release
        // always return to receive mode
        delayMS(30); // debounce button
        if (digitalRead(ENCBUT_PIN)==LOW)
        {
          // stay in this state until button released
          break;
        }
        radio_state = STATE_STEP_SELECT;
        break;
      }
      case STATE_STEP_SELECT:
      {
        if (k==0)
        {
          if (digitalRead(ENCBUT_PIN)==LOW)
          {
            radio_state = STATE_OPTION_EXIT;
          }
          break;
        }
        const uint32_t current_tuning_step = radio.tuning_step;
        if (k>0)
        {
          for (uint8_t p=0;p<k;p++)
          {
            radio.tuning_step *= 10;
            if (radio.tuning_step>1000000UL)
            {
              radio.tuning_step = 1000000UL;
              break;
            }
          }
        }
        else
        {
          k = -k;
          for (uint8_t p=0;p<k;p++)
          {
            radio.tuning_step /= 10;
            if (radio.tuning_step==0)
            {
              radio.tuning_step = 1;
              break;
            }
          }
        }
        show_tuning_step(current_tuning_step);
        break;
      }
    }
  }
}
