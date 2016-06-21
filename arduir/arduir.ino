const String progname = "ArduIR v0.1";

#include <avr/eeprom.h>
#include <IRremote.h>

// uncomment this line to debug 
#define ARDUIR_DEBUG

// uncomment this line if using a Common Anode RGB LED
#define RGB_LED_COMMON_ANODE

// set colcors for the RGB LED
#define LED_OFF              setLedColor(0, 0, 0)
#define LED_LIGHT_RED        setLedColor(255, 0, 0)
#define LED_LIGHT_GREEN      setLedColor(0, 255, 0)
#define LED_LIGHT_BLUE       setLedColor(0, 0, 255)
#define LED_LIGHT_PURPLE     setLedColor(255, 0, 255)
#define LED_LIGHT_YELLOW     setLedColor(255, 180, 0)

// pin definition
const uint8_t BUTTON_PIN = 5;
const uint8_t POWERSTATE_PIN = 6;
const uint8_t SWITCH_PIN = 7;
const uint8_t RGB_LED_BLUE_PIN = 8;
const uint8_t RGB_LED_GREEN_PIN = 9;
const uint8_t RGB_LED_RED_PIN = 10;
const uint8_t RECV_PIN = 11;

// powerstate
uint8_t powerState = 0;
uint8_t oldPowerState = 2;         // 0 => off; 1 => on; 2 => reset RGB LED to powerState value

// button and menu variable
bool buttonActive = false;         // true if button is pushed
uint16_t menuHoldTime = 2000;      // time to hold the button to activate the menu
uint16_t holdTime = 300;           // time to hold the button for menu entries
uint32_t firstTime = 0;
bool waitForButtonPress = true;
bool readNewButtonCode = false;
bool inMenuMode = false;


// IArduino-IRremote library
IRrecv irrecv(RECV_PIN);
IRsend irsend;
decode_results results;

// IR variable
int8_t codeType = -1;                // type of the IR code
int32_t codeValue;                   // IR code value if not raw
uint16_t rawCodes[RAWBUF];           // IR durations if raw
int16_t codeLen;                     // IR length of the code
uint8_t toggleBit = 0;               // RC5/6 toggle bit state


// EEPROM
#define INIT_DATA                    {116, -1, 99999999, 11111, 11111, 0, 100}

typedef struct s_eeprom {
  uint8_t avail;                     // have we ever saved data? 147 = saved data available
  int8_t type;                       // saved type of the IR code
  int32_t value;                     // saved value of the IR code if not raw
  uint16_t rawcode;                  // saved durations of the IR code if raw 
  int16_t len;                       // length if the IR code
  uint8_t startsys;                  // do we want to start the system after power comes back?
  uint8_t bright;                    // saved value for the brightness of the RGB LED
} SAVED_DATA;

SAVED_DATA EEMEM eeprom = INIT_DATA; // data in eeprom
SAVED_DATA sram;                     // data in sram

void restoreData(void){
  eeprom_read_block(&sram, &eeprom, sizeof(SAVED_DATA));
  if (sram.avail != 147) {
    sram = INIT_DATA;
  }
}

void saveData(void){
  if (sram.avail == 116) {
    sram.avail = 147;
  }
  eeprom_write_block(&sram, &eeprom, sizeof(SAVED_DATA));
}

void deleteData(void) {
  sram.avail = 85;
  saveData();
  restoreData();
#ifdef ARDUIR_DEBUG
  Serial.println("Deleted saved data!");
#endif
}


#ifdef ARDUIR_DEBUG
String inputString = "";        // a string to hold incoming serial data
boolean stringComplete = false; // whether the string is complete

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); // get the new byte
    inputString += inChar;             // add it to the inputString
    if (inChar == '\n') {              // if the incoming character is a newline, set stringComplete to true
      stringComplete = true;
    }
  }
}

String codeTypeToString(int p) {
    switch (p) {
        case 1: return("RC5"); break;
        case 2: return("RC6"); break;
        case 3: return("NEC"); break;
        case 4: return("SONY"); break;
        case 5: return("PANASONIC"); break;
        case 6: return("JVC"); break;
        case 7: return("SAMSUNG"); break;
        case 8: return("WHYNTER"); break;
        case 9: return("AIWA_RC_T501"); break;
        case 10: return("LG"); break;
        case 15: return("DENON"); break;
        default: return("UNKNOWN"); break;
    }
}
#endif

void checkCode(decode_results *results) {
  codeType = results->decode_type;

  if (codeType == UNKNOWN) {
    codeLen = results->rawlen - 1;
    // To store raw codes:
    // Drop first value (gap)
    // Convert from ticks to microseconds
    // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
    for (int i = 1; i <= codeLen; i++) {
      if (i % 2) {
        // Mark
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK - MARK_EXCESS;
      } else {
        // Space
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK + MARK_EXCESS;
      }
    }
  }
  if (codeType == NEC) {
    if (results->value == REPEAT) {
      // Don't record a NEC repeat value as that's useless.
      return;
    }
  }
  codeValue = results->value;
  codeLen = results->bits;
  if (codeType == RC5 || codeType == RC6) {
    // Put the toggle bit into the code to send
    codeValue = codeValue & ~(1 << (codeLen - 1));
    codeValue = codeValue | (toggleBit << (codeLen - 1));
  }
#ifdef ARDUIR_DEBUG
  Serial.print("<< " + codeTypeToString(codeType) + " received\tDEC:");
  Serial.print(codeValue, DEC);
  Serial.print("\tHEX: ");
  Serial.print(codeValue, HEX);
  if (codeType == RC5 || codeType == RC6) {
    Serial.print("\tToggle Bit:");
    Serial.print(toggleBit);
  }
  if (codeValue == sram.value) {
    Serial.println("\t***");
  } else {
    Serial.println();
  }
#endif
}

void sendCode(void) {
  if (codeType == NEC) {
    irsend.sendNEC(codeValue, codeLen);
  } else if (codeType == SONY) {
    irsend.sendSony(codeValue, codeLen);
  } else if (codeType == PANASONIC) {
    irsend.sendPanasonic(codeValue, codeLen);
  } else if (codeType == JVC) {
    irsend.sendPanasonic(codeValue, codeLen);
  } else if (codeType == RC5) {
    irsend.sendRC5(codeValue, codeLen);
  } else if (codeType == RC6) {
    irsend.sendRC6(codeValue, codeLen);
  } else if (codeType == UNKNOWN) {
    irsend.sendRaw(rawCodes, codeLen, 38); // Assume 38kHz
  }
#ifdef ARDUIR_DEBUG
  Serial.print(">> " + codeTypeToString(codeType) + " send\tDEC: ");
  Serial.print(codeValue, DEC);
  Serial.print("\t HEX: ");
  Serial.println(codeValue, HEX);
#endif
}

void setLedColor(int red, int green, int blue) {
  red = map(sram.bright, 0, 100, 0, red);
  green = map(sram.bright, 0, 100, 0, green);
  blue = map(sram.bright, 0, 100, 0, blue);    
#ifdef RGB_LED_COMMON_ANODE
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
#endif
  analogWrite(RGB_LED_RED_PIN, red);
  analogWrite(RGB_LED_GREEN_PIN, green);
  analogWrite(RGB_LED_BLUE_PIN, blue);  
}

void blinkLedRed(int p, int d) {
  for (int i=1; i <= p; i++) {
    LED_OFF;
    delay(d);
    LED_LIGHT_RED;
    if (p != 1) {
        delay(d);
    }
  }
}

void blinkLedGreen(int p, int d) {
  for (int i=1; i <= p; i++) {
    LED_OFF;
    delay(d);
    LED_LIGHT_GREEN;
    if (p != 1) {
        delay(d);
    }
  }
}

void blinkLedBlue(int p, int d) {
  for (int i=1; i <= p; i++) {
    LED_OFF;
    delay(d);
    LED_LIGHT_BLUE;
    if (p != 1) {
        delay(d);
    }
  }
}

void blinkLedPurple(int p, int d) {
  for (int i=1; i <= p; i++) {
    LED_OFF;
    delay(d);
    LED_LIGHT_PURPLE;
    if (p != 1) {
        delay(d);
    }
  }
}

int getMenuEntry() {
  unsigned long waitTime;
  int i = 0;
  bool waitForButtonPress = true;

  waitTime = millis();
  while (waitForButtonPress == true) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      if (buttonActive == false) {
        buttonActive = true;
        firstTime = millis();
      } else {
        if (millis() - firstTime > holdTime) {
          i++;
          blinkLedPurple(1, 200);
          delay(200);
          while (digitalRead(BUTTON_PIN) == LOW) {
            blinkLedPurple(2, 200);
          }
          buttonActive = false;
          waitTime = millis();
        }
      }
    } else {
      if (millis() - waitTime > 3000) {
        if (i == 0) {
          blinkLedRed(4, 200);
        }
        waitForButtonPress = false;
      }
    }
  }
  return i;
}

void menuMode() {
  int menuEntry = 0;
  int subMenuEntry = 0;
  
  inMenuMode = true;
#ifdef ARDUIR_DEBUG
  Serial.println(F("Now in menu mode"));
#endif
  menuEntry = getMenuEntry();
#ifdef ARDUIR_DEBUG
  Serial.print(F("Selected menu entry: "));
  Serial.println(menuEntry);
#endif
  if (menuEntry != 0) {
    blinkLedPurple(menuEntry, 500);
  }
  switch (menuEntry) {
    case 1:
      readNewButtonCode = true;
#ifdef ARDUIR_DEBUG
      Serial.println(F("Push a button that should be stored!"));
#endif
      break;
    case 2:
      if (sram.startsys == 1) {
        sram.startsys = 0;
      } else {
        sram.startsys = 1;
      }
      saveData();
      inMenuMode = false;
      break;
    case 3:

    case 4:
      subMenuEntry = getMenuEntry();
      switch (subMenuEntry) {
        case 1:
          sram.bright = 50;
          break;
        case 2:
          sram.bright = 60;
          break;
        case 3:
          sram.bright = 70;
          break;
        case 4:
          sram.bright = 80;
          break;
        case 5:
          sram.bright = 90;
          break;
        case 6:
        default:
          sram.bright = 100;
          break;
      }
      saveData();
      inMenuMode = false;
      break;
    case 5:
      deleteData();
      break;
    default:
#ifdef ARDUIR_DEBUG
     Serial.println(F("Entry in menu does not exist!"));
#endif
      inMenuMode = false;
      break;
  }
}

void toggleTheSystem() {
  unsigned int p = 0;

  LED_LIGHT_YELLOW;
  digitalWrite(SWITCH_PIN, HIGH);
  delay(400);
  digitalWrite(SWITCH_PIN, LOW);
  
  // sometimes the system won't start with the first action
  // so repeat it, the RGB LED will fading through the colors
  while (digitalRead(POWERSTATE_PIN) == LOW && powerState == 0) {
    delay(100);
    p++;
    if (p >= 10) {
      digitalWrite(SWITCH_PIN, HIGH);
      delay(400);
      digitalWrite(SWITCH_PIN, LOW);
      p = 0;
    }
  }
}

void setup() {
#ifdef ARDUIR_DEBUG
  Serial.begin(9600);
  inputString.reserve(200);
  Serial.print(F("### "));
  Serial.print(progname);
  Serial.println(F(" ###"));
  Serial.println(F("You are in debug mode"));
  Serial.println(F("Known commands: setbutton; showdata, deletedata"));
#endif
  restoreData();
  if (sram.bright <= 50) {
    sram.bright = 50;
  }
  pinMode(RGB_LED_RED_PIN, OUTPUT);
  pinMode(RGB_LED_GREEN_PIN, OUTPUT);
  pinMode(RGB_LED_BLUE_PIN, OUTPUT);
  LED_OFF;
  pinMode(SWITCH_PIN, OUTPUT);
  pinMode(POWERSTATE_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);
  irrecv.enableIRIn();
  irrecv.blink13(false);
  delay(1000);
  if ((digitalRead(POWERSTATE_PIN) == LOW) && (sram.startsys == 1)) {
    toggleTheSystem();
  }
}

void loop() {

#ifdef ARDUIR_DEBUG
  if (stringComplete) {
    Serial.print(F("Received command "));
    Serial.print(inputString);
    if (inputString.compareTo("setbutton\n") == 0 ) {
      inMenuMode = true;
      readNewButtonCode = true;
      Serial.println(F("Push a button that should be stored!"));
    } else if (inputString.compareTo("showdata\n") == 0) {
      if (sram.avail == 147) {
        Serial.println(F("Saved data:"));
        Serial.print(F("\tButton protocol:\t"));
        Serial.println(codeTypeToString(sram.type));
        if (sram.type != -1) {
          Serial.print(F("\tButton code:\t\tDEC "));
          Serial.print(sram.value, DEC);
          Serial.print(F("    HEX "));
          Serial.println(sram.value, HEX);
        } else {
          Serial.print(F("\tRAW Button code:\t"));
          Serial.println(sram.rawcode);
        }
        Serial.print(F("\tStart System:\t\t"));
        Serial.println(sram.startsys);
        Serial.print(F("\tBrightness LED:\t\t"));
        Serial.println(sram.bright);
      } else {
        Serial.println(F("No saved data!"));
      }
    } else if (inputString.compareTo("deletedata\n") == 0) {
      deleteData();
    } else {
      Serial.println(F(" - Unknown command!"));
    }
    inputString = ""; // clear the string
    stringComplete = false;
  }
#endif

  // if the variable inMenuMode is true, we are in menu mode and want to store a new code
  // the LED should lighting purple
  if (inMenuMode == true) {
    LED_LIGHT_PURPLE;
  } else {
    powerState = digitalRead(POWERSTATE_PIN);
    if (powerState != oldPowerState) {
      if (powerState == 0) {
        LED_LIGHT_RED;
      } else {
        LED_LIGHT_GREEN;
      }
      oldPowerState = powerState;
    }
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    oldPowerState = 2; // reset RGB LED to powerstate (green or red) after menu mode
    if (buttonActive == false) {
      buttonActive = true;
      firstTime = millis();
    } else {
      if (millis() - firstTime > menuHoldTime) {
        blinkLedPurple(2, 500);
        while (digitalRead(BUTTON_PIN) == LOW) {
          blinkLedPurple(2, 200);
        }
        buttonActive = false;
        menuMode();
      }
    }
  } else {
    buttonActive = false;
  }

  if (irrecv.decode(&results)) {
    checkCode(&results);

    // read new button code and save it, when different from stored code
    // readNewButtonCode could only be set from menu, so we are in menu mode, LED is lighting yellow
    if (readNewButtonCode) {
      blinkLedPurple(3, 30);
      if (codeValue != sram.value) {  // check new codeValue with stored value in sram
                                      // is it not the same then store it to eeprom
        sram.type = codeType;
        sram.value = codeValue;
        saveData();                   // here we save it
        delay(10);
        restoreData();                // read back the stored data from eepreom, and check it
        if (codeValue != sram.value || codeType != sram.type) {
          blinkLedRed(4, 200);        // button code could not be stored
#ifdef ARDUIR_DEBUG
          Serial.println(F("New code could not be stored from remote! Something went wrong?"));
          Serial.print(F(" Got: "));
          Serial.print(codeTypeToString(codeType) + " ");
          Serial.println(codeValue, DEC);
          Serial.print(F(" Stored: "));
          Serial.print(codeTypeToString(sram.type) + " ");
          Serial.println(sram.value, DEC);
#endif
        } else {                      // button code could be stored
          blinkLedGreen(2, 500);
#ifdef ARDUIR_DEBUG
          Serial.println(F("Stored new code from remote!"));
#endif
        }
      } else {                       // button code is the same as stored
        blinkLedGreen(2, 500);
#ifdef ARDUIR_DEBUG  
        Serial.println(F("The new button code was not updated, same as stored!"));
#endif
      }
      readNewButtonCode = false;
      inMenuMode = false;
    // if we do not store a new button code, then we are in normal operating mode
    } else {
      if (codeType == sram.type && codeValue == sram.value) {
        blinkLedBlue(3, 30);
        LED_OFF;
        toggleTheSystem();
      } else {
        if (powerState == 1) {
          sendCode();
          blinkLedBlue(3, 30);
          LED_OFF;
          irrecv.enableIRIn(); // re-enable receiver
#ifdef ARDUIR_DEBUG
        } else {
          Serial.println(F("   Nothing send, powerstate is not 1!"));
#endif
        }
      }
    }
    irrecv.resume();   // receive the next value
    oldPowerState = 2; // reset RGB LED to powerstate (green or red)
  }
  delay(10);
}
