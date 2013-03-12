
#define PIN_RELAY 9
#define PIN_PIR   PUSH2
#define PIN_DHT   12
#define PIN_LDR   A4
#define LDR_AVG   10
//define NO_LED
//define NO_PREAMBLE

#ifndef NO_PREAMBLE
#define _PREAMBLE_COUNT 10
#define _PREAMBLE_CHAR  '!'
#endif

#ifndef NO_LED
#define LED_BOOT  GREEN_LED
#define LED_LAMP  RED_LED
#endif

#define DHT11_OK                0
#define DHT11_ERROR_CHECKSUM   -1
#define DHT11_ERROR_TIMEOUT    -2

int buttonState = LOW;
unsigned int buttonDebounce = 0;
int careAboutPIR = 0;

int readDHT11(int* humidity, int* temperature) {
  // send start signal
  pinMode(PIN_DHT, OUTPUT);
  digitalWrite(PIN_DHT, LOW);
  delay(18);
  digitalWrite(PIN_DHT, HIGH);
  delayMicroseconds(8);
  pinMode(PIN_DHT, INPUT);

  // wait for LOW response
  {
    int i = 0;
    while(digitalRead(PIN_DHT) == LOW) {
      if(i++ >= 10) return DHT11_ERROR_TIMEOUT;
      delayMicroseconds(10);
    }
  }
  // wait for HIGH response
  {
    int i = 0;
    while(digitalRead(PIN_DHT) == HIGH) {
      if(i++ >= 8) return DHT11_ERROR_TIMEOUT;
      delayMicroseconds(10);
    }
  }

  // read five bytes
  unsigned char buffer[5], byte = 0;
  for(byte = 0; byte < 5; byte++) {
    buffer[byte] = 0;

    int bit = 0;
    for(bit = 0; bit < 8; bit++) {
      // wait until signal goes high
      while(digitalRead(PIN_DHT) == LOW) delayMicroseconds(1);

      // if signal is still high, this bit is a 1
      delayMicroseconds(35);
      if(digitalRead(PIN_DHT) == HIGH) {
        buffer[byte] |= 1 << (7 - bit);
        while(digitalRead(PIN_DHT) == HIGH) delayMicroseconds(1);
        delayMicroseconds(10);
      }
    }
  }

  unsigned char checksum = 0;
  for(byte = 0; byte < 4; byte++) checksum += buffer[byte];
  if(checksum != buffer[4]) return DHT11_ERROR_CHECKSUM;

  *(humidity + 0)    = buffer[0];
  *(humidity + 1)    = buffer[1];
  *(temperature + 0) = buffer[2];
  *(temperature + 1) = buffer[3];
  return DHT11_OK;
}

void updateTemperatureReading() {
  int humidity[2], temperature[2];
  if(readDHT11(humidity, temperature) == DHT11_OK) {
    Serial.print("DHT h:"); Serial.print(humidity[0]); Serial.print("."); Serial.print(humidity[1]);
    Serial.print(" t:"); Serial.print(temperature[0]); Serial.print("."); Serial.print(temperature[1]);
    Serial.println();
  } else {
    
  }
}

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

