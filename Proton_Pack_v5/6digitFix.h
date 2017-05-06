
/*
  6digitFix.h - Library for 6 digit 7 segement hardware based on the TM1637 chip.  It is an extension 4 digit 
  7 segement library from Arduino
  Created by Kurt A Gustafson, May 4, 2017.
  Released into the public domain.
*/
#ifndef sixDigitFix_h
#define sixDigitFix_h
#include "SevenSegmentTM1637.h"
#include "Arduino.h"


class sixDigitSevenSeg
{
  public:
    sixDigitSevenSeg();
    void displayNum(SevenSegmentTM1637& display, long inNum, bool setZero);
  private:
	uint8_t myNums[11];
	//= {TM1637_CHAR_0, TM1637_CHAR_1, TM1637_CHAR_2, TM1637_CHAR_3, TM1637_CHAR_4, TM1637_CHAR_5, TM1637_CHAR_6, TM1637_CHAR_7, TM1637_CHAR_8, TM1637_CHAR_9,TM1637_CHAR_SPACE};
	int a,b,c,d,e,f;  // holds the 6 digits; a = leftmost digit, f right most.
	int commPos[6];
	//={3,2,1,6,5,4};  // position of the displays
	void setNum(long inNum, bool setZero);
};

#endif