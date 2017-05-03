/*
Eight Segment Setup
*/
/*
   Setup Digits, Alpha, and Positial bytes for 6 digit display
*/

const byte NUMBERS[] = {
        B00000011, // 0
        B10011111, // 1
        B00100101, // 2
        B00001101, // 3
        B10011001, // 4
        B01001001, // 5
        B01000001, // 6
        B00011111, // 7
        B00000001, // 8
        B00011001, // 9
};
const byte CHAR[] = {
        B00010001, // A
        B00000001, // B
        B01100011, // C
        B10000101, // D
        B01100001, // E
        B01110001, // F
        B00001001, // G
        B10010001, // H
        B10011111, // I
        B10000111, // J
        B10010001, // K
        B11100011, // L
        B00010011, // M
        B11010101, // N
        B11000101, // O
        B00110001, // P
        B00100001, // Q
        B11110101, // R
        B01001001, // S
        B11110001, // T
        B10000011, // U
        B11000111, // V
        B10000011, // W
        B11011001, // X
        B10001001, // Y
        B00100101,// Z
};


const byte LOC[] = {
        B00001000, // first digit
        B00000100, // second digit
        B00000010, // third digit
        B10000000, // fourth digit
        B01000000, // fifth digit
        B00100000, // sixth digit
}; 
