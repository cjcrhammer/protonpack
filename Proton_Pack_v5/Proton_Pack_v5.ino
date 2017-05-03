/*
Proton Pack Version 5
The proton pack is based on a FSM design.
 
Implement the solution with a Finite State
Machine or FSM.
 
Following the excellent description at
http://hacking.majenko.co.uk/finite-state-machine
first break down the problem into states.
 
Identify which states are Transitional (T) and
which are Static (S). A Static state is one in
which the FSM is waiting for stimulus, and is
taking no actions. A Transitional State is a
state which causes actions, but doesn't look
for stimulus.
 
A Transitional State runs once and immediately
moves on to a Static State.
 
State 0: Power Off State; waiting for main power turn on (S)
State 1: Initializing; powers up the proton pack (T)
State 2: Running; waits for power off or trigger press (S)
State 3: Trigger Pressed; Firing up main gun (T)
State 4: Trigger hold; waiting for trigger release (S) 
State 5: Trigger release; power down gun (T) 
State 6: Main power turned off; powering down the system (S)
state 7: Future for to play the song (T)

  The circuit:
  * connect TM1637 pin CLK to Arduino pin D2
  * connect TM1637 pin DIO to Arduino pin D3
  
  * connect 6d7s DIO to Arduino pin D6 (data)
  * connect 6d7s SCK to Arduino pin D5 (clock)
  * connect 6d7s RCK to Arduino pin D4 (latch)

  
  * connect 6d7s       VCC to 5V
  * connect 6d7s       GND to GND
  * connect TM1637 pin Vcc to Arduino pin 5V
  * connect TM1637 pin GND to Arduino pin GND

  * connect WS2801 Ground to Arduino pin GND and Separate Power ground
  * connect WS2801 DIO to Arduino pin 8 
  * connect WS2801 Clock to Arduino pin7
  * connect WS2801 Power to separate power supply VCC (5V)

*/
 
// Use names for states so it's easier to
// understand what the code is doing
const int S_POWER_OFF = 0;
const int S_INITIALIZING = 1;
const int S_RUNNING = 2;
const int S_FIRING_RU = 3;
const int S_FIRING_CS = 4;
const int S_FIRING_RD = 5;
const int S_POWER_DOWN = 6;
const int S_THEME_SONG = 7;
 
/*
   Setup for various libraries used in this display
*/

// include the SevenSegmentTM1637 library
#include "SevenSegmentTM1637.h"
#include "SevenSegmentExtended.h"
#include "SevenSegmentFun.h"
//#include "EightSegment.h"
//#include <ShiftDisplay.h>
// include the WS2801 library
#include "Adafruit_WS2801.h"
#include "SPI.h" // Comment out this line if using Trinket or Gemma
#ifdef __AVR_ATtiny85__
 #include <avr/power.h>
#endif

/* initialize global TM1637 Display object
*  The constructor takes two arguments, the number of the clock pin and the digital output pin:
* SevenSegmentTM1637(byte pinCLK, byte pinDIO);
*/

/*
   Setup for various PINS
*/

// 4 Digit 7-Segment Display Pins
const byte PIN_CLK = 2;   // define CLK pin (any digital pin)
const byte PIN_DIO = 3;   // define DIO pin (any digital pin)
// 6 Digit 7-Segment Display Pins
const byte PIN_6RCK = 4;  // 6 digit RCK (latch) pin  (This pin is no longer used with new 6 segment chips.
const byte PIN_6SCK = 5;  // 6 digit SCK (clock) pin
const byte PIN_6DIO = 6;  // 6 digit DIO pin
//  WS2801 PINS for Synchrotron, Gun Barrell, & Tip
uint8_t dataPin  = 8;    // DIO (White) on WS2801 Pixels 
uint8_t clockPin = 7;    // Clock (Green) on WS2801 Pixels 
// Pins for the Buttons and Switches
const int PIN_MainPower = A1; //Main power switch
const int PIN_Trigger = A2; //Trigger Button
const int PIN_Theme = A3; //Button to play theme song
//PINS to trigger sounds
const int PIN_Snd_Running = 9; //Powerup
const int PIN_Snd_Firing = 10; //Firing
const int PIN_Snd_Theme = 11; //Button to play theme song

/*
 * Setup for the 6 Digit delay??? Not sure why this is used.
 */
const int digit6Delay=1000;  // 6 digit delay has to be larger than 4 digit delay
const long synchro_nom=408346;  //This is the number the 6-digit display is usually around in normal state
const long synchro_fire=210000; //This is the number the 6-digit display is usually around in firing state
const int volt_nom=4332; //This is the number the 4-digit display is usually around in normal state
const int volt_fire=2200; //This is the number the 4-digit display is usually around in firing state
  
const int time_init = 5000; //time in milliseconds for the init screen to take place
const int time_fire_ru = 2000; //time in milliseconds for the init of the firing
const int time_fire_rd = 3000; //time in milleseconds for the ramp down from firing
const int time_pdown = 4000; //time in milliseconds for the power down

const int strip_synchro = 20; //number of pixels for sycnhrotron
const int strip_barrell = 8; //number of pixels for barrel
const int strip_tip = 3; //number of pixels for tip

int ramp_speed = synchro_nom / time_init; //Increments based on rpm/ms
        
int currDigit; 
int cnt,x,i ;  // used in loop
int my4digit;
int mydigit1, mydigit2;   // stores first 3 and last 3 digits of 6 seg

int effOnePos;
int effOneCnt;
int effOneLength;  // Cntr used by first effect, Length of the effect.  We may eventually put this in an array


SevenSegmentFun    fourDigit(PIN_CLK, PIN_DIO);  // setup 4 digit display
SevenSegmentFun    sixDigit(PIN_6SCK, PIN_6DIO);  // setup 6 digit display
//ShiftDisplay sixDigit(PIN_6RCK,PIN_6SCK,PIN_6DIO, COMMON_CATHODE, 6); // setup 6 digit display

// Don't forget to connect the ground wire to Arduino ground,
// and the +5V wire to a +5V supply

// Set the first variable to the NUMBER of pixels. 25 = 25 pixels in a row
Adafruit_WS2801 strip = Adafruit_WS2801(32, dataPin, clockPin);

void setup()
{
  Serial.begin(9600);         // initializes the Serial connection @ 9600 baud
  fourDigit.begin();            // initializes the display
  fourDigit.setBacklight(25);  // set the brightness to 50 %
  //Sets up the 6 digit display
  sixDigit.begin();
  sixDigit.setBacklight(25);
 /* REMOVE OLD 6-digit code    
    
  //pinMode(PIN_6SCK, OUTPUT); // sets the digital pin as output
  //pinMode(PIN_6RCK, OUTPUT); // sets the digital pin as output
  //pinMode(PIN_6DIO, OUTPUT); // sets the digital pin as output
*/
  pinMode(PIN_Snd_Running, OUTPUT);
  pinMode(PIN_Snd_Firing, OUTPUT);
  pinMode(PIN_Snd_Theme, OUTPUT);

  digitalWrite(PIN_Snd_Running, HIGH);
  digitalWrite(PIN_Snd_Firing, HIGH);
  digitalWrite(PIN_Snd_Theme, HIGH);
  
  randomSeed(analogRead(0));  //don't like this random read
  mydigit1=random(1,1000)-1;
  mydigit2=random(1,1000)-1;
  my4digit=random(1,10000)-1;
  cnt=0;
  i=0;

  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  
    clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
  #endif

  strip.begin();

  // Update LED contents, to start they are all 'off'
  strip.show();
  effOnePos=5; // Intitial position of effect
  effOneLength=100;
  effOneCnt=effOneLength;
}
 
void loop()
{
  // start off with everything in powerdown state
  // The keyword "static" makes sure the variable
  // isn't destroyed after each loop
  static int state = S_POWER_OFF ;
 
  // To store the current time  for delays
  static unsigned long ts;
 
  switch (state)
  {
    case S_POWER_OFF:
      // Turn off 6,4, and all lights
    //  reset6Disp(); // resets the 6 digit display
     // byte clearbyte=B11111111;
/* REMOVE OLD 6-digit code   
  for (int j=0;j<6;j++) {
      digitalWrite(PIN_6RCK, LOW);
      shiftOut(PIN_6DIO, PIN_6SCK, LSBFIRST, B11111111);
      shiftOut(PIN_6DIO, PIN_6SCK, LSBFIRST, LOC[j]);
      digitalWrite(PIN_6RCK, HIGH);
    }
   */
      strip.begin(); //resets the ws2801 lights
      strip.show();
      fourDigit.begin();            // 4-digit initializes the display
      fourDigit.setBacklight(25);  // set the brightness to 50 %
      sixDigit.begin();            // 6-digit initializes the display
      sixDigit.setBacklight(25);  // set the brightness to 50 %
      ts = millis();  // Remember the current time
      state = S_INITIALIZING;
 
      break;
 
    case S_INITIALIZING:
 
      if ( digitalRead (PIN_MainPower) == HIGH) {
        /*
         * Insert code to start the power up sequence
         */
        int current_speed = 100 * (millis() - ts);
/* REMOVE OLD 6-digit code           
 * disp6Digit(current_speed);  
 */
        fourDigit.print(0001);
        sixDigit.print(123456);
        if (millis() > ts + time_init)
        {
          state = S_RUNNING;
        }
      }
 
      break;
 
    case S_RUNNING:
         /*
         * Insert code to maintain running mode
         */
         sixDigit.print(synchro_nom);
         fourDigit.print(volt_nom);

        digitalWrite(PIN_Snd_Running, HIGH);
        digitalWrite(PIN_Snd_Firing, HIGH);
        digitalWrite(PIN_Snd_Theme, HIGH);
  
      if ( digitalRead (PIN_MainPower) == LOW ) {
        ts = millis(); //remember the current time
        state = S_POWER_DOWN;
      }
      if (digitalRead (PIN_Trigger) == HIGH ) {
        ts = millis(); //remember the current time
        state = S_FIRING_RU;
      }
      effOne();
      strip.show();
      break;
 
    case S_FIRING_RU:
         /*
         * Insert code to start the firing power up sequence
         */     
         // If two seconds have passed, then move on to the next state.
        sixDigit.print(1);
        fourDigit.print(1); 
        digitalWrite(PIN_Snd_Running, LOW);
          
      if (millis() > ts + time_fire_ru)
      {
        state = S_FIRING_CS;
      }
 
      break;
 
    case S_FIRING_CS:
         /*
         * Insert code to continue firing
         */
        sixDigit.print(synchro_fire);
        fourDigit.print(volt_fire);
        digitalWrite(PIN_Snd_Firing, LOW);
        digitalWrite(PIN_Snd_Running, HIGH);
       if (digitalRead (PIN_Trigger) == LOW ) {
        ts = millis(); //remember the current time
        state = S_FIRING_RD;
      }

      break;
 
    case S_FIRING_RD:
 
         /*
         * Insert code to ramp down the gun
         */
         sixDigit.print(2);
        fourDigit.print(2);
        digitalWrite(PIN_Snd_Firing, HIGH);
        digitalWrite(PIN_Snd_Theme, LOW);
     if (millis() > ts + time_fire_rd)
      {
        state = S_RUNNING;
      }
 
      break;
 
    case S_POWER_DOWN:
         /*
         * Insert code to powerdown the system
         */
        sixDigit.print(3);
        fourDigit.print(3);
      if (millis() > ts + time_pdown)
      {
        state = S_POWER_OFF;
      }

      break;
 
    case S_THEME_SONG:
 
         /*
         * Insert code to play the theme song
         */
      break;
 
  } // end of switch
 
  // other things could go on here maybe this is where you display what is going on.

  //Perform the 4 Digit actions based on state

  //Perform the Synchrotron Actions based on state

  //Perform the Barrell Actions based on state

  //Perform the Barrell Tip Actions based on state
  
 
} // end of loopvoid setup() {
  // put your setup code here, to run once:

//}

void effOne() {
  if (effOneCnt < 1) {
    effOnePos++;
    if (effOnePos > 12) {
      effOnePos=5;
    }
    effOneCnt=effOneLength;
  }
  cyclotronSiren(effOnePos);
  effOneCnt--;
}

/*void setup6Digit() {
  pinMode(PIN_6SCK, OUTPUT); // sets the digital pin as output
  pinMode(PIN_6RCK, OUTPUT); // sets the digital pin as output
  pinMode(PIN_6DIO, OUTPUT); // sets the digital pin as output
};
*/
void synchr_spin() {
   for (int i=0;i<strip_synchro;i++) {
      strip.setPixelColor(i-1, Color(255, 0, 0));
      if (i==0) {strip.setPixelColor(20,Color(0,0,0));}
      strip.setPixelColor(i-2,Color(0,0,0));
      strip.show();  
   }
}
/*void reset6Disp() {
      byte clearbyte=B11111111;
      for (int x=0;x<6;x++) {
   digitalWrite(PIN_6RCK, LOW);
   shiftOut(PIN_6DIO, PIN_6SCK, LSBFIRST, clearbyte);
   shiftOut(PIN_6DIO, PIN_6SCK, LSBFIRST, LOC[x]);
   digitalWrite(PIN_6RCK, HIGH);
      }
};

void disp6OneD(int num, int dispnum) {
  byte dispbyte  =  LOC[dispnum-1];
  byte digitbyte =  NUMBERS[num];
  digitalWrite(PIN_6RCK, LOW);
  shiftOut(PIN_6DIO, PIN_6SCK, LSBFIRST, digitbyte);
  shiftOut(PIN_6DIO, PIN_6SCK, LSBFIRST, dispbyte);
  digitalWrite(PIN_6RCK, HIGH);
};

void disp6Digit(long synchro) {
  reset6Disp();
  int nums[6] = {0,0,0,0,0,0};
  long d1=(synchro/100000)%10;
  long d2=(synchro/10000)%10;
  long d3=(synchro/1000)%10;
  long d4=(synchro/100)%10;
  long d5=(synchro/10)%10;
  long d6=(synchro)%10;
  nums[0]=d1;
  nums[1]=d2;
  nums[2]=d3;
  nums[3]=d4;
  nums[4]=d5;
  nums[5]=d6;
  for (int x=0;x<6;x++) {
    disp6OneD(nums[x], x+1);
  }
  
};
*/
/* Helper functions */

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
   return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170; 
   return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

/* Begin Add by Kurt on 3/20/2017

Added helper functions of the following:

getNextDigit (min, max, changeval) 
   where min is the minimum value to change (lower bound)
    and max is the maximum value to change (upper bound)
    and changeval is the AMOUNT of change either up or down.
   program assumes that the global variable currValue holds the 
     the current value of the digit in question. 

cyclotronSiren (position)
   where position is between 5 and 12
   this funciton sets the values depending on the position of the "siren"
   assumes that the global variable called strip is declared and is of type 2801
   also assumes that your configuration is 5-12 for inner circle of lights and 13-20 for the outer circle

Ramp and tipFlash
 */

long getNextDigit(long minval, long maxval, int changeval) {
 float setpoint;
 long retval; 
 int randval;
 setpoint = currDigit - minval;
 //Serial.println(setpoint);
 setpoint = setpoint / (maxval - minval);
 setpoint = setpoint * 100;
 randval = random(1,100);
  if (randval < setpoint ) {
  retval = currDigit - changeval;
 } else {
  retval = currDigit + changeval;
 }
 if (retval > maxval) {
  retval = maxval;
 }
 if (retval < minval) {
  retval = minval;
 }
 return (retval);
};

void cyclotronSiren(int newpos) {
        // rotation of the cyclotron is for pixels 5 through 12 (inner ring)
        //   we set the second half (outer ring) based on the first half (8-20) which mirrors 5-12
        // figure out the n-2, n-1, and n+1 pixel positions
        int a, b, c; // a = pixel to turn off (pos-2), b&c pixels to turn 50 red (pos-1 and pos+1)
        if (newpos > 6) {
                a=newpos-2; // turn off pixel
                b=newpos-1; // pos - 1 pixel
        } else {
                if (newpos == 6) {
                        a=12;  // turn off pixel
                        b=5;   // pos - 1 pixel
                } else {   // newpos == 5
                        a=11;  // turn off pixel
                        b=12;  // pos - 1 pixel
                }
        }
        if (newpos < 12) {
                c=newpos+1;  // pos + 1 pixel
        } else {
                c=5;            // pos + 1 pixel
        }

        // next set half the lights for the siren (5 thru 12)
        strip.setPixelColor(a, Color(0,0,0));
        strip.setPixelColor(b, Color(127,0,0));
        strip.setPixelColor(c, Color(127,0,0));
        strip.setPixelColor(newpos, Color(255,0,0));

        // next set the other half of the lights for the siren (13 thru 20)
        strip.setPixelColor(a+8, Color(0,0,0));
        strip.setPixelColor(b+8, Color(127,0,0));
        strip.setPixelColor(c+8, Color(127,0,0));
        strip.setPixelColor(newpos+8, Color(255,0,0));
}

// for ramping the cyclotron, use rampStart of 1 and rampEnd of 20
// for ramping the barrel, use rampStart of 21, and rampEnd of 28
// Note, this will ramp red color only
void Ramp(int intensity, int rampStart, int rampEnd) {
    if (intensity > 255) {
  intensity=255;
 }
 if (intensity < 0) {
  intensity=0;
 }
 for (int x=rampStart;x<(rampEnd+1);x++) {
  strip.setPixelColor(x, Color(intensity,0,0));
 }
}

void RandWhite(int intensity, int startval, int endval) {
 RandColor(startval, endval, intensity, intensity, intensity);
}

void RandBlue(int intensity, int startval, int endval) {
 RandColor(startval, endval, 0,0,intensity);
}

void RandColor(int startval, int endval, int r, int g, int b) {
 int randval;
 randval=random(startval, endval);
 strip.setPixelColor(randval, Color(r,g,b));
}

// tipFlash will be used to set the white value of the tip.  If zero is presented, it will turn off the light.
void tipFlash(int intensity) {
    if (intensity > 255) {
      intensity=255;
    }
    if (intensity < 0) {
      intensity=0;
    }
   int randval;
   // Set the tip to the intensity level
   for (int x=29; x<33; x++) {
      strip.setPixelColor(x, Color(intensity,intensity,intensity));
   }

   // Random blue 20% of the time if intensity is other than zero
   if (intensity > 0) {
      randval=random(1,100);
      if (randval <21) {
         RandBlue(intensity, 29, 32);
      }
   }
}
