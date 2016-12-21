#define PROGNAME    "ArduIR"
#define PROGVERS    "0.3"

#include <avr/eeprom.h>
#include <IRremote.h>

// uncomment the next line to debug 
//#define ARDUIR_DEBUG



// uncomment the line if using a Common Anode RGB LED
// comment the line with '//' if using a Common Cathode RGB LED
#define RGB_LED_COMMON_ANODE

// set colcors for the RGB LED => setLedColor(red, green, blue)
#define LED_OFF              setLedColor(0, 0, 0)     // no color, LED is off
#define LED_LIGHT_RED        setLedColor(255, 0, 0)   // color for device is off
#define LED_LIGHT_GREEN      setLedColor(0, 255, 0)   // color for device is on
#define LED_LIGHT_BLUE       setLedColor(0, 0, 255)   // color for incoming IR signal
#define LED_LIGHT_PURPLE     setLedColor(255, 0, 255) // color for menu mode
#define LED_LIGHT_CYAN       setLedColor(0, 255, 255) // color for check for the device comes up



// Menumodes
// Menu 1: Save new button code from the remote to power up the device - here is no submenu
// Menu 2: Start the device when power is back with submenuentry 1 (inverts the state)
//          0 - start device only with button code (default)
//          1 - start device with button code and when power is back
// Menu 3: change RC5/RC6 ToggleBit state with submenuentry 1
// Menu 4: Set brightness off the RGB LED, with the submenus
// Menu 5: Delete all saved data with submenuentry 3
#define MAX_MENU_MODES  5



// pin definition
// note: pin 3 is used as output in IRremote, so we can not use them
const uint8_t POWER_BUTTON_PIN = 10;
const uint8_t DOWN_BUTTON_PIN = 8;
const uint8_t UP_BUTTON_PIN = 4;
const uint8_t POWERSTATE_PIN = 7;
const uint8_t SWITCH_PIN = 2;
const uint8_t RGB_LED_BLUE_PIN = 5;
const uint8_t RGB_LED_GREEN_PIN = 6;
const uint8_t RGB_LED_RED_PIN = 9;
const uint8_t RECV_PIN = 11;

// powerstate
uint8_t powerState = 0;
uint8_t oldPowerState = 2;           // 0 => off; 1 => on; 2 => reset RGB LED to powerState value

// button and menu variable
bool buttonActive = false;           // true if button is pushed
uint16_t shutdownHoldTime = 10000;   // time to hold the button to hard power off the device
uint16_t warningHoldTime = 7000;     // time to hold the button when the LED will blinking RED
uint16_t menuHoldTime = 5000;        // time to hold the button to activate the menu mode
uint16_t menuActiveTime = 5000;      // how long we should wait in menu mode for a button press
uint16_t buttonHoldTime = 200;       // time to hold the button to get recognized
uint16_t newIrCodeWaitTime = 20000;  // how long we should wait to receive an new ir code
uint32_t firstNewIrCodeTime = 0;
uint32_t firstTime = 0;
bool waitForButtonPress = true;
bool readNewIrCode = false;          // if true we will try to save a new ir button code





// IArduino-IRremote library
IRrecv irrecv(RECV_PIN);
IRsend irsend;
decode_results results;

// IR variable
int8_t codeType = -1;                // type of the IR code
int32_t codeValue;                   // IR code value if not raw
uint16_t rawCodes[RAWBUF];           // IR durations if raw
int16_t codeLen;                     // IR length of the code



// EEPROM & SRAM data
#define INIT_DATA                    {116, -1, 12345, 11011, 1, 22, 0, 80}  // initial data, if nothing is stored correctly


typedef struct s_eeprom {
  uint8_t avail;                      // have we ever saved the data? - this first value set the availabilty:
                                        //   85 - data was deleted
                                        //  116 - initial data
                                        //  147 - correctly saved data
  int8_t type;                        // type of the IR code (RC5, RC6, NEC, etc.; 0 or -1 = type is UNKNOWN)
  int32_t value;                      // value of the IR code if not raw
  uint16_t rawcode;                   // durations of the IR code if raw
  uint8_t togglebit;                  // RC5/6 toggle bit state (0 = don't touch the bit; 1 = set bit to 1)
  int16_t len;                        // length if the IR code
  uint8_t startsys;                   // do we want to start the device after power comes back?
  uint8_t bright;                     // value for the brightness of the RGB LED
} SAVED_DATA;

SAVED_DATA EEMEM eeprom = INIT_DATA;  // data in eeprom
SAVED_DATA sram;                      // data in sram

void restoreData(void){
  eeprom_read_block(&sram, &eeprom, sizeof(SAVED_DATA));
  if (sram.avail != 147) {  // if we never have saved the data, or we deleted them, then take the init_data
    sram = INIT_DATA;
  }
}

void saveData(void){
  if (sram.avail == 116) {  // if we want to store the data, then set it to 147, but not if we deleted them before
    sram.avail = 147;
  }
  eeprom_write_block(&sram, &eeprom, sizeof(SAVED_DATA));
}

void deleteData(void) {
  sram.avail = 85;  // override the availability
  saveData();       // save data to eeprom from sram
  restoreData();    // restore data - if availablity is not 147, then load init_data
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
    if (inChar == '\n') {
      stringComplete = true;           // if the incoming character is a newline, set stringComplete to true
    } else {
      inputString += inChar;           // else add the character to the inputString
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




void checkIRCode(decode_results *results) {
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
  if ((codeType == RC5 || codeType == RC6) && sram.togglebit == 1) {
    // Put the toggle bit into the code to send
    codeValue = codeValue & ~(1 << (codeLen - 1));
    codeValue = codeValue | (sram.togglebit << (codeLen - 1));
  }
#ifdef ARDUIR_DEBUG
  Serial.print(F("<< "));
  Serial.print(codeTypeToString(codeType));
  Serial.print(F(" received\tDEC: "));
  Serial.print(codeValue, DEC);
  Serial.print(F("\tHEX: "));
  Serial.print(codeValue, HEX);
  Serial.print(F("\tLength: "));
  Serial.print(codeLen, DEC);
  if (codeValue == sram.value) {
    Serial.println(F("\t***"));
  } else {
    Serial.println();
  }
#endif
}

void sendIRCode(void) {
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
  Serial.print(F(">> "));
  Serial.print(codeTypeToString(codeType));
  Serial.print(F(" send\tDEC: "));
  Serial.print(codeValue, DEC);
  Serial.print(F("\tHEX: "));
  Serial.print(codeValue, HEX);
  Serial.print(F("\tLength: "));
  Serial.println(codeLen, DEC);
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
    LED_LIGHT_RED;
    delay(d);
    LED_OFF;
    delay(d);
  }
}

void blinkLedGreen(int p, int d) {
  for (int i=1; i <= p; i++) {
    LED_LIGHT_GREEN;
    delay(d);
    LED_OFF;
    delay(d);
  }
}

void blinkLedBlue(int p, int d) {
  for (int i=1; i <= p; i++) {
    LED_LIGHT_BLUE;
    delay(d);
    LED_OFF;
    delay(d);
  }
}

void blinkLedPurple(int p, int d) {
  for (int i=1; i <= p; i++) {
    LED_OFF;
    delay(d);
    LED_LIGHT_PURPLE;
    delay(d);
  }
}

void blinkIR() {
  blinkLedBlue(2, 20);
}

void blinkGood() {
  blinkLedGreen(2, 400);
}

void blinkError() {
  blinkLedRed(4, 100);
}

void blinkButtonError() {
  blinkLedRed(2, 100);
}

void blinkMenuMode() {
  blinkLedPurple(1, 400);
}


int getMenuEntry() {
  unsigned long waitTime;
  int i = 0;
  bool waitForButtonPress = true;
  bool buttonUpActive = false;
  bool buttonDownActive = false;
  bool buttonPowerActive = false;

  LED_LIGHT_PURPLE;
#ifdef ARDUIR_DEBUG
  Serial.println(F("## Please slect an menu entry"));
#endif
  waitTime = millis();
  while (waitForButtonPress == true) {
    if (digitalRead(UP_BUTTON_PIN) == LOW) {
      if ((buttonUpActive == false) && (buttonDownActive == false) && (buttonPowerActive == false)) {
        buttonUpActive = true;
        firstTime = millis();
      } else {
        if (millis() - firstTime > buttonHoldTime) {
          i++;
          blinkMenuMode();
          // if the button was not realeased, let blink the LED red
          while (digitalRead(UP_BUTTON_PIN) == LOW) {
            blinkButtonError();
          }
          buttonUpActive = false;
          waitTime = millis();
        }
      }
    }
    if (digitalRead(DOWN_BUTTON_PIN) == LOW) {
      if ((buttonUpActive == false) && (buttonDownActive == false) && (buttonPowerActive == false)) {
        buttonDownActive = true;
        firstTime = millis();
      } else {
        if (millis() - firstTime > buttonHoldTime) {
          if (i <= 0) {
            i = 0;
          } else {
            i--;
          }
          blinkMenuMode();
          // if the button was not realeased, let blink the LED red
          while (digitalRead(DOWN_BUTTON_PIN) == LOW) {
            blinkButtonError();
          }
          buttonDownActive = false;
          waitTime = millis();
        }
      }
    }
    if (digitalRead(POWER_BUTTON_PIN) == LOW) {
      if ((buttonUpActive == false) && (buttonDownActive == false) && (buttonPowerActive == false)) {
        buttonPowerActive = true;
        firstTime = millis();
      } else {
        if (millis() - firstTime > buttonHoldTime) {
          LED_OFF;
          delay(800);
          // if the button was not realeased, let blink the LED red
          while (digitalRead(POWER_BUTTON_PIN) == LOW) {
            blinkButtonError();
          }
          buttonPowerActive = false;
          waitForButtonPress = false;
        }
      }
    }
    if (millis() - waitTime > menuActiveTime) {
#ifdef ARDUIR_DEBUG
      Serial.println(F("!! Menu mode timed out, because no new button has been pressed!"));
#endif
      blinkError();
      i = 0;
      waitForButtonPress = false;
    }
  }
  return i;
}




void menuMode() {
  int menuEntry = 0;
  int subMenuEntry = 0;

#ifdef ARDUIR_DEBUG
  Serial.println(F("## Menu mode active"));
#endif
  menuEntry = getMenuEntry();
#ifdef ARDUIR_DEBUG
  Serial.print(F("?? Selected entry in main menu: "));
  Serial.println(menuEntry);
#endif
  if (menuEntry >= 0 && menuEntry <= MAX_MENU_MODES) {
    // show that we accept the entry
    delay(500);
    blinkLedPurple(menuEntry, 400);
    LED_OFF;
    delay(500);
    LED_LIGHT_GREEN;
    delay(1000);
  }
  
  switch (menuEntry) {
    case 1:
      readNewIrCode = true;
      firstNewIrCodeTime = millis();
#ifdef ARDUIR_DEBUG
      Serial.println(F("[] Push a button on your ir remote, that should be stored!"));
#endif
      break;
    case 2:
      subMenuEntry = getMenuEntry();
      if (subMenuEntry == 1) {
        sram.startsys = 1 - sram.startsys;
        saveData();
        blinkGood();
      } else {
        blinkError();
      }
      break;
    case 3:
      subMenuEntry = getMenuEntry();
      if (subMenuEntry == 1) {
        sram.togglebit = 1 - sram.togglebit;
        saveData();
        blinkGood();
      } else {
        blinkError();
      }
      break;
    case 4:
      while (digitalRead(POWER_BUTTON_PIN) == HIGH) {
        if (digitalRead(UP_BUTTON_PIN) == LOW) {
          if (sram.bright >= 100) {
            sram.bright = 100;
          } else {
            sram.bright++;
          }
        }
        if (digitalRead(DOWN_BUTTON_PIN) == LOW) {
          if (sram.bright <= 0) {
            sram.bright = 0;
          } else {
            sram.bright--;
          }
        }
        LED_LIGHT_PURPLE;  // set RGB LED to the selected brightness
        delay(20);
      }
      LED_OFF;
      delay(500);
      while (digitalRead(POWER_BUTTON_PIN) == LOW) {
        blinkButtonError();
      }
      LED_OFF;
      delay(500);
      saveData();
      blinkGood();
      break;
    case 5:
      subMenuEntry = getMenuEntry();
      if (subMenuEntry == 3) {
        deleteData();
        blinkLedGreen(1, 500);
        blinkLedRed(1, 500);
        blinkLedGreen(1, 500);
        blinkLedRed(1, 500);
        blinkLedGreen(1, 500);
        blinkLedRed(1, 500);
      } else {
        blinkError();
      }
      break;
    default:
#ifdef ARDUIR_DEBUG
     Serial.println(F("!! Entry in main menu does not exist!"));
#endif
      blinkError();
      break;
  }
}



void startDevice() {
  unsigned int p = 0;

#ifdef ARDUIR_DEBUG
  Serial.println(F("** Try to starting device"));
#endif
  LED_LIGHT_CYAN;
  digitalWrite(SWITCH_PIN, HIGH);
  delay(350);
  digitalWrite(SWITCH_PIN, LOW);
  delay(2000);  // wait two seconds for the device to come up

  // sometimes the device won't start with the first action, so repeat it
  while (digitalRead(POWERSTATE_PIN) == LOW) {
#ifdef ARDUIR_DEBUG
  Serial.println(F("** Try to starting device again"));
#endif
    blinkLedGreen(1, 200);
    LED_LIGHT_CYAN;
    digitalWrite(SWITCH_PIN, HIGH);
    delay(350);
    digitalWrite(SWITCH_PIN, LOW);
    p++;
    if (p == 5) {
      break;
    }
    delay(2000);
  }
  if (digitalRead(POWERSTATE_PIN) == LOW) {
#ifdef ARDUIR_DEBUG
  Serial.println(F("!! Starting the device failed"));
#endif
    blinkError();
    LED_OFF;
    delay(1000);
  }
}



void stopDevice() {
#ifdef ARDUIR_DEBUG
  Serial.println(F("** Stopping device hard"));
#endif
  LED_LIGHT_CYAN;
  digitalWrite(SWITCH_PIN, HIGH);
  while (digitalRead(POWERSTATE_PIN) == HIGH) {
    delay(100); // wait 100ms
  }
  digitalWrite(SWITCH_PIN, LOW);
  LED_OFF;
  delay(2000);  // wait two seconds
}




void setup() {
#ifdef ARDUIR_DEBUG
  Serial.begin(9600);
  inputString.reserve(200);
  Serial.print(F("### "));
  Serial.print(PROGNAME);
  Serial.print(" ");
  Serial.print(PROGVERS);
  Serial.println(F(" ###"));
  Serial.println(F("You are in debug mode"));
  Serial.println(F("Known commands: setbutton; showdata; deletedata"));
  Serial.println();
#endif
  restoreData();
  pinMode(RGB_LED_RED_PIN, OUTPUT);
  pinMode(RGB_LED_GREEN_PIN, OUTPUT);
  pinMode(RGB_LED_BLUE_PIN, OUTPUT);
  LED_OFF;  // set LED off
  pinMode(SWITCH_PIN, OUTPUT);
  pinMode(POWERSTATE_PIN, INPUT);
  pinMode(UP_BUTTON_PIN, INPUT);
  digitalWrite(UP_BUTTON_PIN, HIGH);
  pinMode(DOWN_BUTTON_PIN, INPUT);
  digitalWrite(DOWN_BUTTON_PIN, HIGH);
  pinMode(POWER_BUTTON_PIN, INPUT);
  digitalWrite(POWER_BUTTON_PIN, HIGH);
  irrecv.enableIRIn();
  irrecv.blink13(false);
  delay(1000);
  if ((digitalRead(POWERSTATE_PIN) == LOW) && (sram.startsys == 1)) {
    startDevice();
  }
}



void loop() {


  // read device powerstate
  powerState = digitalRead(POWERSTATE_PIN);


  // LED color
  // if we want to store a new button code then the LED will lightning blue,
  // else set the RGB LED to device powerstate
  if (readNewIrCode == true) {
    if (millis() - firstNewIrCodeTime > newIrCodeWaitTime) {
#ifdef ARDUIR_DEBUG
        Serial.println(F("!! Read new ir code from remote timed out!"));
#endif
        readNewIrCode = false;
        oldPowerState = 2;  // reset RGB LED to powerstate (green or red)
    } else {
      LED_LIGHT_BLUE;
    }
  } else {
    if (powerState != oldPowerState) {
      if (powerState == 0) {
        LED_LIGHT_RED;
      } else {
        LED_LIGHT_GREEN;
      }
      oldPowerState = powerState;
    }
  }


  // incomming IR signal
  if (irrecv.decode(&results)) {
    checkIRCode(&results);

    // read the new button code and save it, when different from stored code
    if (readNewIrCode == true) {
      blinkIR();
      if (codeValue != sram.value) {  // check new codeValue with stored value in sram
                                      // is it not the same then store it to eeprom
        sram.type = codeType;
        sram.value = codeValue;
        sram.len = codeLen;
//      TODO: save raw codes
        saveData();                   // here we save it
        delay(10);
        restoreData();                // read back the stored data from eepreom, and check it
        if (codeValue != sram.value || codeType != sram.type) {
          blinkError();        // button code could not be stored
#ifdef ARDUIR_DEBUG
          Serial.println(F("!! New code could not be stored from remote! Something went wrong!"));
          Serial.print(F(" Got: "));
          Serial.print(codeTypeToString(codeType) + " ");
          Serial.println(codeValue, DEC);
          Serial.print(F(" Stored: "));
          Serial.print(codeTypeToString(sram.type) + " ");
          Serial.println(sram.value, DEC);
#endif
        } else {                      // button code could be stored
          blinkGood();
#ifdef ARDUIR_DEBUG
          Serial.println(F("** Stored new code from remote!"));
#endif
        }
      } else {                       // button code is the same as stored
        blinkGood();
#ifdef ARDUIR_DEBUG  
        Serial.println(F("** The new button code was not updated, same as stored!"));
#endif
      }
      readNewIrCode = false;
    // if we do not store a new button code, then we are in normal operating mode
    } else {
      if (powerState == 0) {
        if (codeType == sram.type && codeValue == sram.value && codeLen == sram.len) {
          blinkIR();
          startDevice();
#ifdef ARDUIR_DEBUG
        } else {
          Serial.println(F("!! Nothing send to the device, because device is off!"));
#endif
        }
      } else {
        sendIRCode();
        blinkIR();
        irrecv.enableIRIn(); // re-enable receiver
      }
    }
    irrecv.resume();   // receive the next value
    oldPowerState = 2; // reset RGB LED to powerstate (green or red)
  }

 
#ifdef ARDUIR_DEBUG
   // incoming commands over serial
  if (stringComplete) {
    Serial.print(F("** Received command:\t"));
    Serial.println(inputString);
    if (inputString.compareTo("setbutton") == 0 ) {
      readNewIrCode = true;
      firstNewIrCodeTime = millis();
      Serial.println(F("[] Push a button on your ir remote, that should be stored!"));
    } else if (inputString.compareTo("showdata") == 0) {
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
        if (sram.type == 1 || sram.type == 2) {  // show ToggleBit for RC5/RC6
          Serial.print(F("\tTogglebit:\t\t"));
          Serial.println(sram.togglebit);
        }
        Serial.print(F("\tLength:\t\t\t"));
        Serial.println(sram.len);
        Serial.print(F("\tStart System:\t\t"));
        Serial.println(sram.startsys);
        Serial.print(F("\tBrightness LED:\t\t"));
        Serial.println(sram.bright);
      } else {
        Serial.println(F("!! No saved data available!"));
      }
    } else if (inputString.compareTo("deletedata") == 0) {
      deleteData();
    } else {
      Serial.println(F("!! Unknown command!"));
    }
    inputString = ""; // clear the string
    stringComplete = false;
  }
#endif


  // power button is pressed
  if (digitalRead(POWER_BUTTON_PIN) == LOW) {
    if (buttonActive == false) {
      buttonActive = true;
      firstTime = millis();
    } else {
      if (millis() - firstTime > menuHoldTime) {
        while (digitalRead(POWER_BUTTON_PIN) == LOW) {
          if (millis() - firstTime > warningHoldTime) {
            blinkError();
          } else {
            blinkLedPurple(2, 200);
          }
          if (millis() - firstTime > shutdownHoldTime) {
            buttonActive = false;
            LED_LIGHT_RED;
            delay(1000);
            while (digitalRead(POWER_BUTTON_PIN) == LOW) {
              blinkButtonError();
            }
            stopDevice();
          }
        }
        if (buttonActive == true) {
          buttonActive = false;
          menuMode();  // call menuMode()
        }
        oldPowerState = 2; // reset RGB LED to powerstate (green or red)
      }   
    }
  }

  // power button has released
  if ((digitalRead(POWER_BUTTON_PIN) == HIGH) && (buttonActive == true)) {
    buttonActive = false;
    if (millis() - firstTime > buttonHoldTime) {
      if (readNewIrCode == true) {
        readNewIrCode = false;
      } else {
        if (powerState == 0) {
          startDevice();
        } else {
          if (sram.avail == 147) {
            codeType = sram.type;
            codeValue = sram.value;
//          TODO: read raw codes
            codeLen = sram.len;
            sendIRCode();
            blinkGood();
            LED_OFF;
            irrecv.enableIRIn(); // re-enable receiver
          } else {
            // we have no saved data
            blinkError();
            LED_OFF;
          }
        }
      }
      oldPowerState = 2; // reset RGB LED to powerstate (green or red)
    }
  }


  delay(5);
}

