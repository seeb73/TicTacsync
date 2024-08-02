/*
                             a Timestamp generator for
                             Adafruit QT Py board
                             https://www.adafruit.com/product/4600

  This is an ugly patchwork of code snippets found on the tubes. It produces a
  BFSK modulated audio signal via Direct Digital Synthesis on the SAMD21 DAC
  output. Timing of DAC samples are done with TC5. The payload is the UTC
  time obtained from the GPS attached to Serial1 (when reception permits it).
  Binary word output is triggered by a hardware interrupt: the GPS one pulse
  per second (aka PPS) cabled to pin A1 . Before the BFSK word starts a small
  blip is output to the DAC to precisely mark the start of the UTC second.
  This blip is a full scale ramp attained in 10 samples and used by the
  python desktop post production software tictacsync. In case of too weak GPS
  signals a standalone mode is initiated on power up by the user by grounding
  pin A2. The RTC is then used to generate an internal PPS and arbitrary
  date/time values are modulated (bit #3 signals this Non-GPS state).

  For more details go to https://tictacsync.org

  MIT LicenseCopyright (c) 2021 Raymond Lutz
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/



#include <Adafruit_NeoPixel_ZeroDMA.h>  // patched lib for onboard NeoPix
// Adafruit_NeoPixel lib doesnt work with TC5, it seems
// see https://forums.adafruit.com/viewtopic.php?t=201285
#include <RTCZero.h>
#include <MicroNMEA.h>
#define NEOPIXPIN 11 
#define NUM_PIXELS 1
#define DATAPIN 7
#define CLOCKPIN 8
#define BLOCKWAITINGFORSERIAL false
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
#define gps Serial1
#define console Serial
#define VERSIONNUM 0    // two bits max
#define YEAR_ZERO 2021  // for encoding year with offset
#define DAC_AMPLITUDE 511
#define DAC_MID_VALUE 511        // (2**10 - 1) / 2
#define LED_FLASH_DURATION 0.02  // sec
#define RAMP_STEPS 10            // length of sync pulse

/*
  N_SYMBOLS, number of bits:
  1 sync +
  2 bits for version
  1 bit for clock source: bit=1 for GPS; bit=0 for RTC +
  6 bits for secs + 6 bits for min +
  5 bits for hr + 5 bits for day + 4 bit for month +
  5 bits for year offset (since YEAR_ZERO) = 35
*/


//next lines pasted from FSKfreqCalculator.py output (so each FSK symbol ends at y=0):
#define N_SYMBOLS 35
#define DAC_OUT_FREQ 96000        // in Hertz
#define N_SAMPLES_WAVETABLE 1371  // 14.281 ms each of 35 symbols
#define f1 630.00                 // for BFSK, in Hertz
#define f2 1190.00

// flag set on power-up, according to A2 High or Low state
volatile bool GPS_enabled = false;
// flag to catch 1PPS hardware IRQ on A1, polled in loop()
volatile bool PPSarrived = false;
// flag to catch software IRQ from TC5, polled in loop()
volatile bool DACtimeToWriteOut = false;
char nmeaBuffer[100];
char timecodeword[N_SYMBOLS] = { 0 };
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
Adafruit_NeoPixel_ZeroDMA pixels(NUM_PIXELS, NEOPIXPIN, NEO_GRB);
RTCZero rtc;
//volatile bool ppsTriggered = false;
uint16_t *wave_tables_f1_f2;  // array to store sinewave points
int i_sample = 0;
void TC5forDACconf();
void tcENABLE();

void printUnknownSentence(MicroNMEA &nmea) {
  console.println();
  console.print("Unknown sentence: ");
  console.println(nmea.getSentence());
}

void TC5_Handler(void) {
  //  console.print("in tc5handler");
  DACtimeToWriteOut = true;
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;  //don't change this, it's part of the timer code
}

void PPSexternalInt_handler(void) {
  // trig on pin A1
  DACtimeToWriteOut = true;
  PPSarrived = true;
  tcENABLE();
  // turn on LED
  pixels.setPixelColor(0, pixels.Color(255, 0, 255));
  pixels.show();
}

void alarmMatch() {
  //  DACtimeToWriteOut = true;
  //  PPSarrived = true;
  //  tcENABLE();
  setAlarmNextSecond();
  PPSexternalInt_handler();
}

void setup() {
  pixels.begin();  // Initialize pins for output
  pixels.setBrightness(10);
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  console.begin(9600);
  while (!console and BLOCKWAITINGFORSERIAL)
    ;
  gps.begin(9600);  // gps
  console.println("in setup()");
  TC5forDACconf();  //configure the timer to run at <sampleRate>Hertz
  //  nmea.setUnknownSentenceHandler(printUnknownSentence);
  MicroNMEA::sendSentence(gps, PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // pullup on Trinket M0 silkscreened pin "0" for GPS or RTC mode switch
  // checked on powering up
  pinMode(A2, INPUT_PULLUP);
  delay(500);
  if (not digitalRead(A2))  // read switch position, 0 = pushed = no GPS
  {
    GPS_enabled = false;
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    RTCconf();
  } else {
    GPS_enabled = true;
    // Adaruit Feather M0 pin '15/A1'
    // Adaruit QT py pin '1/A1'
    attachInterrupt(digitalPinToInterrupt(A1), PPSexternalInt_handler, RISING);
    pixels.show();
  }
  // set the DAC for 10 bits of resolution (max)
  analogWriteResolution(10);
  // Allocate the buffers where the samples are stored
  wave_tables_f1_f2 = (uint16_t *)malloc(2 * N_SAMPLES_WAVETABLE * sizeof(uint16_t));
  // calculate sinus values
  genSin();
}

void loop() {
  int DACvalue;
  if (PPSarrived) {
    PPSarrived = false;
    i_sample = 0;
    punchGPSintoWord();  // later?
  }
  if (DACtimeToWriteOut) {
    DACtimeToWriteOut = false;
    // getSampleValue();
    int i_symbol, DAC_value, i_wavetable;
    int bit_value;

    i_symbol = i_sample / N_SAMPLES_WAVETABLE;  // sync = symbol 0
    i_wavetable = i_sample % N_SAMPLES_WAVETABLE;
    bit_value = timecodeword[i_symbol];
    DAC_value = wave_tables_f1_f2[2 * i_wavetable + bit_value];
    if (i_symbol == int(LED_FLASH_DURATION * DAC_OUT_FREQ / N_SAMPLES_WAVETABLE))  // turn off blinking LED
    {
      pixels.clear();
      pixels.show();
    }
    if (i_symbol == 0 && i_sample <= RAMP_STEPS)  // still in ramp sync pulse
    {
      DAC_value = int(511 * i_sample / RAMP_STEPS) + DAC_MID_VALUE;
      //      console.println(DAC_value);
    }
    if (i_symbol == 0 && i_sample > RAMP_STEPS)  // sync pulse finished
      DAC_value = DAC_MID_VALUE;
    if (i_symbol == N_SYMBOLS - 1 && i_wavetable == (N_SAMPLES_WAVETABLE - 1)) {  // last symbol and last sample
      tcDISABLE();                                                                // stop triggering DAC outputs
      DAC_value = DAC_MID_VALUE;
    }
    analogWrite(A0, DAC_value);
    i_sample += 1;
  }
  while (!PPSarrived && gps.available() && GPS_enabled) {
    char c = gps.read();
    //    console.print(c);
    nmea.process(c);
  }
}

void punchGPSintoWord() {
  int SS, MM, HH, DD, MT, YY;
  if (GPS_enabled) {
    SS = nmea.getSecond();
    MM = nmea.getMinute();
    HH = nmea.getHour();
    DD = nmea.getDay();
    MT = nmea.getMonth();
    YY = nmea.getYear();
    nmea.clear();
  } else {
    SS = RTC->MODE2.CLOCK.bit.SECOND;
    MM = RTC->MODE2.CLOCK.bit.MINUTE;
    HH = RTC->MODE2.CLOCK.bit.HOUR;
    DD = RTC->MODE2.CLOCK.bit.DAY;
    MT = RTC->MODE2.CLOCK.bit.MONTH;
    YY = RTC->MODE2.CLOCK.bit.YEAR;
  }
  copyBits2timecodeWord(VERSIONNUM, 1, 2);   // starts at 1 because sync bit is  i=0
  copyBits2timecodeWord(GPS_enabled, 3, 1);  // clock source, RTC or GPS
  copyBits2timecodeWord(SS, 4, 6);
  copyBits2timecodeWord(MM, 10, 6);
  copyBits2timecodeWord(HH, 16, 5);
  copyBits2timecodeWord(DD, 21, 5);
  copyBits2timecodeWord(MT, 26, 4);
  copyBits2timecodeWord(YY - YEAR_ZERO, 30, 5);
  // for (int i = 0; i <= N_SYMBOLS; i++) {
  //   if (i == 3 || i == 1 || i == 4 || i == 10 || i == 16 || i == 21 || i == 26 || i == 30)
  //     console.print(" ");
  //   console.print(timecodeword[i], BIN);
  // }
  // console.println();
  //
  // timecodeword[] will have this binary content
  // (filled from GPS time output or from RTC values):
  // bit 0   :   Sync Pulse (no value)
  // bit 1   :   version num b0
  // bit 2   :   version num b1
  // bit 3   :   clock source: 1=GPS, 0=RTC
  // bit 4   :   seconds b0 (LSB)
  // bit 5   :   seconds b1
  // bit 6   :   seconds b2
  // bit 7   :   seconds b3
  // bit 8   :   seconds b4
  // bit 9   :   seconds b5
  // bit 10  :   minutes b0 (LSB)
  // bit 11  :   minutes b1
  // bit 12  :   minutes b2
  // bit 13  :   minutes b3
  // bit 14  :   minutes b4
  // bit 15  :   minutes b5
  // bit 16  :   hours b0 (LSB)
  // bit 17  :   hours b1
  // bit 18  :   hours b2
  // bit 19  :   hours b3
  // bit 20  :   hours b4
  // bit 21  :   day b0 (LSB)
  // bit 22  :   day b1
  // bit 23  :   day b2
  // bit 24  :   day b3
  // bit 25  :   day b4
  // bit 26  :   month b1 (LSB)
  // bit 27  :   month b2
  // bit 28  :   month b3
  // bit 29  :   month b4
  // bit 30  :   year offset b1 (LSB)
  // bit 31  :   year offset b2
  // bit 32  :   year offset b3
  // bit 33  :   year offset b4
  // bit 34  :   year offset b5
}

void TC5forDACconf() {
  // from https://svn.larosterna.com/oss/trunk/arduino/zerotimer/gist.cpp
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
  tcReset();  //reset TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
  uint16_t TC5Val = (uint16_t)(48e6 / DAC_OUT_FREQ - 1);
  TC5->COUNT16.CC[0].reg = TC5Val;
  while (tcIsSyncing())
    ;
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing())
    ;  //wait until TC5 is done syncing
}

bool tcIsSyncing() {
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void tcENABLE() {
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;  //set the CTRLA register
  while (tcIsSyncing())
    ;  //wait until snyc'd
}

void tcReset() {
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing())
    ;
  while (TC5->COUNT16.CTRLA.bit.SWRST)
    ;
}

void tcDISABLE() {
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing())
    ;
}

void genSin() {
  // NB: values for the two sinus are interleaved
  const float pi2 = 2 * 3.141592653;  //2 x pi
  float phase1, phase2;

  for (int i = 0; i < N_SAMPLES_WAVETABLE; i++) {  // loop to build sine wave based on sample count
    phase1 = pi2 * (float)i * f1 / DAC_OUT_FREQ;   // calculate value in radians for sin()
    phase2 = pi2 * (float)i * f2 / DAC_OUT_FREQ;
    wave_tables_f1_f2[2 * i] = ((int)(DAC_AMPLITUDE * sin(phase1) + DAC_MID_VALUE));      // f1
    wave_tables_f1_f2[2 * i + 1] = ((int)(DAC_AMPLITUDE * sin(phase2) + DAC_MID_VALUE));  // f2
  }
}

void copyBits2timecodeWord(int val, int where, int nbits) {
  for (int i = 0; i < nbits; i++) {
    timecodeword[where] = bitRead(val, i);
    where += 1;
  }
}

void setAlarmNextSecond() {
  RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND =
    (RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND + 1) % 60;
}

void RTCconf() {
  rtc.begin();  // initialize RTC 24H format
  rtc.setTime(0, 0, 0);
  rtc.setDate(19, 11, 22);
  rtc.setAlarmTime(0, 0, 0);
  setAlarmNextSecond();
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(alarmMatch);
}
