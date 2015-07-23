/*
Data Logger to Excel

every 0.5 sec A0 and A1 values are keyed into Excel if Pin 4 is GND

*/

#include <U8glib.h>
#include <Encoder.h>
#include <EEPROM.h>

const int buttonPin = 5;          // inpur for rotary push button

int V2;
int V3;
int reading;
int oldReading;
int oldPosition;
int newPosition;
boolean keyboardOn = false;
boolean firstTime = true;
float mAfactor = 5.5;

float mAaverage = 0;
float voltAverage = 0;

long tt = 0;

long address = 0;

float mAmpere;
float volt;

float vcc;
int ADCvalue;
float voltage;

float zeroMA = 5000.0;
int adjVolt = 5000;

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);    // I2C / TWI

Encoder myEnc(0, 1);

void setup() {
  Serial.begin(9600);
  delay(2000);


  // make the pushButton pin an input:
  pinMode(buttonPin, INPUT_PULLUP);

  // A0 to A3 as input
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  // initialize control over the keyboard:
  Keyboard.begin();
  u8g.setColorIndex(1); //BW Display
  u8g.setRot180();
  Serial.println("setup");
}

void loop() {

  initR();
  mAaverage = normAmpere(analogRead(A1)) - zeroMA;
  voltAverage = normVolt(analogRead(A0)) * (adjVolt / 5000.0);
  while (1 == 1) {

    V2 = analogRead(A2);
    V3 = analogRead(A3);

    mAmpere = normAmpere(analogRead(A1)) - zeroMA;
    mAaverage = ((mAaverage * 9.0) + mAmpere) / 10.0;

    volt = normVolt(analogRead(A0)) * (adjVolt / 5000.0);
    voltAverage = ((voltAverage * 9.0) + volt) / 10.0;

    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_unifont);
      u8g.setPrintPos(0, 16);
      u8g.print("Volt");
      u8g.setPrintPos(50, 16);
      u8g.print(voltAverage, 2);

      u8g.setPrintPos(0, 32);
      u8g.print("mA");
      u8g.setPrintPos(50, 32);
      u8g.print(mAaverage, 0);

      u8g.setPrintPos(0, 48);
      u8g.print("Keyboard");
      u8g.setPrintPos(80, 48);
      u8g.print(keyboardOn);

      u8g.setPrintPos(0, 63);
      u8g.print("IN1");
      u8g.setPrintPos(30, 63);
      u8g.print(V2);
      u8g.setPrintPos(68, 63);
      u8g.print("IN2");
      u8g.setPrintPos(97, 63);
      u8g.print(V3);

    } while ( u8g.nextPage() );

      if (!digitalRead(buttonPin)) {  // encoder pressed
        keyboardOn = !keyboardOn;
        while (!digitalRead(buttonPin)) {}
      }
    if ((millis() - tt) > 3000) break;
    tt = millis();


    if (keyboardOn) {

      // output to keyboard

      if (firstTime) {
        keyHeader();
        firstTime = false;
      }

      Keyboard.print(millis() / 1000);
      Keyboard.print(char(9));
      Keyboard.print(voltAverage, 2);
      Keyboard.print(char(9));
      Keyboard.print(mAaverage, 0);
      Keyboard.print(char(9));
      Keyboard.print(V2);
      Keyboard.print(char(9));
      Keyboard.print(V3);
      Keyboard.println(char(13));

      while (millis() - tt < 1000) {}
    }
  }
}

void adjustVolt() {

  newPosition = myEnc.read();
  oldPosition  = newPosition;
  adjVolt = EEPROMReadlong(address);
  voltAverage = normVolt(analogRead(A0)) * (adjVolt / 5000.0);
  //  if (adjVolt < 2000) adjVolt = 5000;
  do {
    volt = normVolt(analogRead(A0)) * (adjVolt / 5000.0);
    voltAverage = ((voltAverage * 9.0) + volt) / 10.0;

    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_unifont);
      u8g.setPrintPos(0, 16);
      u8g.print("Volt");
      u8g.setPrintPos(80, 16);
      u8g.print(voltAverage, 2);
    } while ( u8g.nextPage() );

    newPosition = myEnc.read();
    int diff = newPosition - oldPosition;

    if (diff != 0) {
      adjVolt = adjVolt - (diff * 5);
      oldPosition = newPosition;
    }
  } while (digitalRead(buttonPin));
  EEPROMWritelong(address, adjVolt);
  // wait till released
  while (!digitalRead(buttonPin)) {}
}

void adjustAmpere() {
  zeroMA = normAmpere(analogRead(A1));
  Serial.print("first ");
  Serial.println(zeroMA);

  do {
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_unifont);
            u8g.setPrintPos(0, 16);
      u8g.print("NO CURRENT !!!");
      u8g.setPrintPos(0, 48);
      u8g.print("Zero mA");
      u8g.setPrintPos(70, 48);
      u8g.print(zeroMA, 0);
    } while ( u8g.nextPage() );

    float zz = normAmpere(analogRead(A1));
    zeroMA = ((zeroMA * 49.0) + zz) / 50.0;
  } while (digitalRead(buttonPin));
  // wait till released
  while (!digitalRead(buttonPin)) {}
}


void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void keyHeader() {
  Keyboard.print("Time");
  Keyboard.print(char(9));
  Keyboard.print("Volt");
  Keyboard.print(char(9));
  Keyboard.print("mA");
  Keyboard.print(char(9));
  Keyboard.print("IN 1");
  Keyboard.print(char(9));
  Keyboard.print("IN 2");
  Keyboard.println(char(13));
}


void initR() {
  Serial.println("init");
  adjustAmpere();
  delay(200);
  adjustVolt();
  tt = millis();
  keyboardOn = false;
  firstTime = true;
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result;
}

float normVolt(int reading) {
  float vcc = readVcc() / 1000.0;
  float voltage = (reading / 1023.0) * vcc;
  /* Serial.print (reading);
   Serial.print(", ");
   Serial.print(vcc);
   Serial.print(", ");
   Serial.println(voltage);
  */
  return voltage;
}

float normAmpere(int reading) {
  float vcc = readVcc();
  float mAmp = (reading - 512) * vcc * 5.1638E-3;  // 1023.0 * 5.28;
  /*  Serial.print (reading);
    Serial.print(", ");
    Serial.print(vcc);
    Serial.print(", ");
    Serial.println(mAmp);
  */
  return mAmp;
}

