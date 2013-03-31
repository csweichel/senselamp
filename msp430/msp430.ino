
#define PIN_RELAY 10
#define PIN_PIR   11
#define PIN_DHT   12
#define PIN_LDR   A4
#define LDR_AVG   10
//define NO_LED
#define NO_PREAMBLE
#define USE_DHT22

#ifndef NO_PREAMBLE
#define _PREAMBLE_COUNT 10
#define _PREAMBLE_CHAR  '!'
#endif

#ifndef NO_LED
#define LED_BOOT  GREEN_LED
#define LED_LAMP  RED_LED
#endif

#define DHT_OK                0
#define DHT_ERROR_CHECKSUM   -1
#define DHT_ERROR_TIMEOUT    -2

int buttonState = LOW;
unsigned int buttonDebounce = 0;
int careAboutPIR = 1;

#define MAXTIMINGS 85
int external_readDHT22() {
  uint8_t lastState = HIGH;
  uint16_t counter = 0;
  uint8_t j = 0, i;
  
  uint8_t data[6];
  uint8_t _pin = PIN_DHT;
  uint32_t _lastMillis;
  boolean _lastResult;
  uint32_t _temperature, _humidity;
  
  _temperature = 0;
  _humidity    = 0;
  
  // pull the pin high and wait 250 milliseconds
  digitalWrite(_pin, HIGH);
  delay(250);
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  
  // now pull it low for ~20 milliseconds
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);
  
  noInterrupts();
  
  digitalWrite(_pin, HIGH);
  delayMicroseconds(40);
  pinMode(_pin, INPUT);
  
  // read in timings
  for ( i=0; i< MAXTIMINGS; i++) {
    counter = 0;
    while (digitalRead(_pin) == lastState) {
      counter++;
#if defined(__MSP430G2452__) || defined(__MSP430G2553__) || defined(__MSP430G2231__) // LaunchPad specific
      // LaunchPad implementation
      // LaunchPad faster than Arduino
      // 1. replace delayMicroseconds(1) with delayMicroseconds(3)
      // or
      // 2. compare counter to a higher number
      // by energia » Tue Jun 26, 2012 9:24 pm
      // see http://www.43oh.com/forum/viewtopic.php?p=20821#p20821
      delayMicroseconds(3);
#else // other boards - not tested
      delayMicroseconds(1);
#endif
      if (counter == 255) {
        break;
      }
    }
    lastState = digitalRead(_pin);
    
    if (counter == 255) break;
    
    // ignore first 3 transitions
    if ((i >= 4) && (i%2 == 0)) {
      // shove each bit into the storage bytes
      data[j/8] <<= 1;
      
      // 1. replace delayMicroseconds(1) with delayMicroseconds(3)
      // or
      // 2. compare counter to a higher number
      // by energia » Tue Jun 26, 2012 9:24 pm
      // see http://www.43oh.com/forum/viewtopic.php?p=20821#p20821
      //      if (counter > 8)
      if (counter > 6)
        data[j/8] |= 1;
      j++;
    }
  }
  
  interrupts();
  
  #ifdef DEBUG
  Serial.println();
  Serial.print("debug \t");
  Serial.print("bits received \t");
  Serial.println(j, DEC);
  
  Serial.print("debug \t");
  Serial.print(data[0], HEX); 
  Serial.print(", ");
  Serial.print(data[1], HEX); 
  Serial.print(", ");
  Serial.print(data[2], HEX); 
  Serial.print(", ");
  Serial.print(data[3], HEX); 
  Serial.print(", ");
  Serial.print(data[4], HEX); 
  Serial.print(" =? ");
  Serial.println(data[0] + data[1] + data[2] + data[3] & 0xff, HEX);
  
  Serial.print("debug \t");
  Serial.print(" checksum \t");
  Serial.println( (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ? "ok" : "NO" );
  
  Serial.print("debug \t");
  Serial.print("RH% \t");
  Serial.println(data[0]*256 + data[1], DEC);
  
  Serial.print("debug \t");
  Serial.print("oC \t");
  Serial.println(data[2]*256 + data[3], DEC);
  #endif

  if((data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
    Serial.print("DHT OK h:");
    Serial.print((data[0]*256 + data[1]) / 10, DEC);
    Serial.print(".");
    Serial.print((data[0]*256 + data[1]) % 10, DEC);
    Serial.print(" t:");
    Serial.print((data[2]*256 + data[3]) / 10, DEC);
    Serial.print(".");
    Serial.print((data[2]*256 + data[3]) % 10, DEC);
    Serial.println();
  }

  // check we read 40 bits and that the checksum matches
  if ((j >= 40) && (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xff)) ) {
    
    _temperature = data[2] & 0x7f;
    _temperature *= 256;
    _temperature += data[3];
    if (data[2] & 0x80) _temperature *= -1;
    
    _humidity = data[0];
    _humidity *= 256;
    _humidity += data[1];
    return DHT_OK;
  } else {
    return DHT_ERROR_CHECKSUM;
  }
}

int readDHTBytes(unsigned char* buffer) {
  // send start signal
  pinMode(PIN_DHT, OUTPUT);
  digitalWrite(PIN_DHT, HIGH);
  delay(250);
  digitalWrite(PIN_DHT, LOW);
  delay(18);
  digitalWrite(PIN_DHT, HIGH);
  delayMicroseconds(8);
  pinMode(PIN_DHT, INPUT);

  // wait for LOW response
  {
    int i = 0;
    while(digitalRead(PIN_DHT) == LOW) {
      if(i++ >= 8) return DHT_ERROR_TIMEOUT;
      delayMicroseconds(10);
    }
  }
  // wait for HIGH response
  {
    int i = 0;
    while(digitalRead(PIN_DHT) == HIGH) {
      if(i++ >= 8) return DHT_ERROR_TIMEOUT;
      delayMicroseconds(10);
    }
  }

  // read five bytes
  int bite = 0;
  for(bite = 0; bite < 5; bite++) {
    buffer[bite] = 0;

    int bit = 0;
    for(bit = 0; bit < 8; bit++) {
      // wait until signal goes high
      while(digitalRead(PIN_DHT) != HIGH) nop();// delayMicroseconds(1);

      // if signal is still high, this bit is a 1
      delayMicroseconds(30);
      if(digitalRead(PIN_DHT) == HIGH) {
        buffer[bite] |= 1 << (7 - bit);
        while(digitalRead(PIN_DHT) != LOW) nop();//delayMicroseconds(1);
        //delayMicroseconds(10);
      }
    }
  }
  
  Serial.print("DHT DUMP"); for(bite = 0; bite < 5; bite++) { Serial.print(" 0x"); Serial.print(buffer[bite], HEX); }; Serial.println();
  
  unsigned char checksum = 0;
  for(bite = 0; bite < 4; bite++) checksum += buffer[bite];
  if(checksum != buffer[4]) {
    return DHT_ERROR_CHECKSUM;
  }
  
  return DHT_OK;
}

int readDHT11(int* humidity, int* temperature) {
  unsigned char buffer[5];
  noInterrupts();
  int result = readDHTBytes(buffer);
  interrupts();
  if(result != DHT_OK) return result;

  *(humidity + 0)    = buffer[0];
  *(humidity + 1)    = buffer[1];
  *(temperature + 0) = buffer[2];
  *(temperature + 1) = buffer[3];
  return DHT_OK;
}

int readDHT22(float* humidity, float* temperature) {
  unsigned char buffer[5];
  noInterrupts();
  int result = readDHTBytes(buffer);
  interrupts();

  *(humidity)    = word(buffer[0], buffer[1]) * 0.1;
  *(temperature) = (buffer[2] & 0x80 ? -1 : 1) * word(buffer[2], buffer[3]) * 0.1;
  return result;
}

#ifdef USE_DHT22
void updateTemperatureReading() {
  external_readDHT22();
}

/* IF I USED MY OWN IMPLEMENTATION
void updateTemperatureReading() {
  float humidity, temperature;
  
  int success = readDHT22(&humidity, &temperature);

  Serial.print("DHT ");
  switch(success) {
    case DHT_OK: Serial.print("OK "); break;
    case DHT_ERROR_CHECKSUM: Serial.print("ERR_CHECKSUM "); break;
    case DHT_ERROR_TIMEOUT:  Serial.print("ERR_TIMEOUT "); break;
  }
  Serial.print("h:"); Serial.print(humidity);
  Serial.print(" t:"); Serial.print(temperature);
  Serial.println();
}
*/
#else
void updateTemperatureReading() {
  int humidity[2], temperature[2];
  
  int success = readDHT11(&humidity, &temperature);
  if(success == DHT_OK) {
    Serial.print("DHT h:"); Serial.print(humidity[0]); Serial.print("."); Serial.print(humidity[1]);
    Serial.print(" t:"); Serial.print(temperature[0]); Serial.print("."); Serial.print(temperature[1]);
    Serial.println();
  } else if(success == DHT_ERROR_TIMEOUT) {
    Serial.println("DHT ERROR TIMEOUT");
  } else if(success == DHT_ERROR_CHECKSUM) {
    Serial.println("DHT ERROR CHECKSUM");
  }
}
#endif

void updateLDRReading() {
  unsigned long avg = 0;
  for(int i = 0; i < LDR_AVG; i++) {
    avg += analogRead(PIN_LDR);
  }
  Serial.print("LDR l:"); Serial.println(avg / LDR_AVG);
}

void turnLampOn() {
  digitalWrite(PIN_RELAY, LOW);
  #ifndef NO_LED
  digitalWrite(LED_LAMP, HIGH);
  #endif
}

void turnLampOff() {
  digitalWrite(PIN_RELAY, HIGH);  
  #ifndef NO_LED
  digitalWrite(LED_LAMP, LOW);
  #endif
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);
  
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_DHT, OUTPUT);

  #ifndef NO_LED
  pinMode(LED_BOOT, OUTPUT);  
  digitalWrite(LED_BOOT, HIGH);
  pinMode(LED_LAMP, OUTPUT);  
  digitalWrite(LED_LAMP, LOW);
  #endif
  
  Serial.println("READY");
}

#ifndef NO_PREAMBLE
int preambleCount = _PREAMBLE_COUNT;
#endif

void loop() {
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    char inByte = Serial.read();

#ifndef NO_PREAMBLE
    if(inByte == _PREAMBLE_CHAR) {
      if(preambleCount > 0) preambleCount --;
    } else if(preambleCount == 0) {
#endif
      switch(inByte) {
        case 't': updateTemperatureReading(); break;
        case 'r': updateLDRReading(); break;
        case 'L': turnLampOn(); break;
        case 'l': turnLampOff(); break;
        case 'P': careAboutPIR = 1; break;
        case 'p': careAboutPIR = 0; break;
        case 'a': Serial.println("ALIVE"); break;
      }
#ifndef NO_PREAMBLE
      preambleCount = _PREAMBLE_COUNT;
    }
#endif
    inByte = 0;
  }
  
  int newButtonState = digitalRead(PIN_PIR);
  if(careAboutPIR && newButtonState != buttonState && newButtonState == HIGH && abs(millis() - buttonDebounce) > 100) {
    Serial.print("PIR b:");
    Serial.print(newButtonState);
    Serial.println();
    buttonDebounce = millis();
  }
  buttonState = newButtonState;
}

