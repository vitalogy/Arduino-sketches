
#define PROGNAME    "ArduTemp"
#define PROGVERS    "0.1v"

// how many hours
#define MAX_HOURS 150

/*
#define DELAY 10000
#define MAX_SECONDS 6   // DELAY x MAX_SECONDS = 60
#define MAX_MINUTES 60
*/

#define DELAY 2000
#define MAX_SECONDS 3   // DELAY x MAX_SECONDS = 60
#define MAX_MINUTES 3

#include <SPI.h>
#include <TFT.h>
#include <dht.h>

// pin definition
#define TFT_CS_PIN   10
#define TFT_DC_PIN   9
#define TFT_RST_PIN  8
#define DHT22_PIN    5

dht DHT;

TFT TFTscreen = TFT(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);

// char array to print to the screen
char temp[5];
char hum[5];
char buf[5];

int arraySeconds[MAX_SECONDS];
int arrayMinutes[MAX_MINUTES];
int arrayHours[MAX_HOURS];

int16_t secpos = 0;
int16_t minpos = 0;
int16_t hourpos = -1;

uint8_t i = 0;
int16_t average = 0;
int8_t value = 0;
int8_t maxvalue = -100;
int8_t minvalue = 100;


void updateGraph() {
  int drawHeight = 0;
  int drawWidth = 0;
  int8_t tempvalue = 0;
  int16_t temppos = 0;

  TFTscreen.stroke(0,0,0);
  TFTscreen.fill(0,0,0);
  TFTscreen.rect(0, 40, TFTscreen.width(), TFTscreen.height()-40);

  TFTscreen.stroke(100,180,0);
  if (hourpos == -1) {
    drawWidth = (int)(TFTscreen.width()/2) + (int)(MAX_MINUTES/2);
    for (i = 0; i < MAX_MINUTES; i++) {
      temppos = minpos - i;
      if (temppos < 0) {
        temppos = MAX_MINUTES + temppos;
      }
      tempvalue = arrayMinutes[temppos];
      if (tempvalue == -100) {
        tempvalue = 0; 
      }
      drawHeight = map(tempvalue,0,40,0,80);
      TFTscreen.line(drawWidth - i, TFTscreen.height() - drawHeight, drawWidth - i, TFTscreen.height());
    }
    TFTscreen.stroke(255, 255, 255);
    TFTscreen.setTextSize(1);
    TFTscreen.text("Last minute:", 60, 45);
    if (arrayMinutes[0] != -100) {
      String lastval = String(arrayMinutes[0]);
      lastval.toCharArray(buf, 5);
      TFTscreen.text(buf, 140, 45);
    }
  } else {
    drawWidth = (int)(TFTscreen.width()/2) + (int)(MAX_HOURS/2);
    for (i = 0; i < MAX_HOURS; i++) {
      temppos = hourpos - i;
      if (temppos < 0) {
        temppos = MAX_HOURS + temppos;
      }
      tempvalue = arrayHours[temppos];
      if (tempvalue == -100) {
        tempvalue = 0; 
      } 
      drawHeight = map(tempvalue,0,40,0,80);
      TFTscreen.line(drawWidth - i, TFTscreen.height() - drawHeight, drawWidth - i, TFTscreen.height());
    }
    TFTscreen.stroke(255, 255, 255);
    TFTscreen.setTextSize(1);
    
    TFTscreen.text("Min:", 4, 45);
    String minval = String(minvalue);
    minval.toCharArray(buf, 5);
    TFTscreen.text(buf, 32, 45);
    
    TFTscreen.text("Max:", 4, 55);
    String maxval = String(maxvalue);
    maxval.toCharArray(buf, 5);
    TFTscreen.text(buf, 32, 55);

    TFTscreen.text("Last Hour:", 60, 45);
    String lastval = String(arrayHours[0]);
    lastval.toCharArray(buf, 5);
    TFTscreen.text(buf, 140, 45);
  }
}

int8_t readData() {

  // READ DATA
  int chk = DHT.read22(DHT22_PIN);

  if (chk == 0 ) {
    String tempval = doubleToString(DHT.temperature, 1);
    String humival = doubleToString(DHT.humidity, 1);
    // convert the reading to a char array
    tempval.toCharArray(temp, 5);
    humival.toCharArray(hum, 5);
    Serial.print(secpos);
    Serial.print("\tDHT22:\t");
    Serial.print(hum);
    Serial.print("\t");
    Serial.println(temp);
  //value = (int8_t)DHT.temperature;
    return (int8_t)DHT.temperature;
  } else {
    Serial.print("Error ");
    Serial.println(chk);
  }
}

void setup() {
  Serial.begin(9600);
  // Put this line at the beginning of every sketch that uses the GLCD:
  TFTscreen.begin();
  // clear the screen with a black background
  TFTscreen.background(0, 0, 0);
  TFTscreen.stroke(255, 255, 255);
  TFTscreen.setTextSize(3);
  TFTscreen.text("Feuchte     %", 2, 2);
  TFTscreen.text("Temp        C", 2, 20);
  for (i = 0; i < MAX_SECONDS; i++) {
    arraySeconds[i] = -100;
  }
  for (i = 0; i < MAX_MINUTES; i++) {
    arrayMinutes[i] = -100;
  }
  Serial.print(F("### "));
  Serial.print(F(PROGNAME));
  Serial.print(F(" "));
  Serial.print(F(PROGVERS));
  Serial.println(F(" ###"));
}

void loop() {
  
  if (secpos == MAX_SECONDS) {
    secpos = 0;
    average = 0;
    for (i = 0; i < MAX_SECONDS; i++) {
      average = average + arraySeconds[i];
      arraySeconds[i] = -100;
    }
    average = average / MAX_SECONDS;
    Serial.print("Average of ");
    Serial.print(minpos);
    Serial.print(". minute:\t");
    Serial.println(average);
    arrayMinutes[minpos] = average;
    minpos++;
  }

  TFTscreen.setTextSize(2);

  // delete old sensor value from screen
  TFTscreen.stroke(0, 0, 0);
  TFTscreen.text(hum, 90, 2);
  TFTscreen.text(temp, 90, 20);

  value = readData();

  // print the new sensor value on screen
  TFTscreen.stroke(255, 255, 255);
  TFTscreen.text(hum, 90, 2);
  TFTscreen.text(temp, 90, 20);
  
  arraySeconds[secpos] = value;
  secpos++;

  if (value > maxvalue) {
    maxvalue = value;
  }
  if (value < minvalue) {
    minvalue = value;
  }

  if (hourpos == MAX_HOURS) {
    hourpos = 0;
  }

  if (minpos == MAX_MINUTES) {
    minpos = 0;
    average = 0;
    for (i = 0; i < MAX_MINUTES; i++) {
      average = average + arrayMinutes[i];
      arrayMinutes[i] = -100;
    }
    average = average / MAX_MINUTES;
    if (hourpos == -1) {
      hourpos = 0;
    }
    Serial.print("Average of ");
    Serial.print(hourpos);
    Serial.print(". hour:\t");  
    Serial.println(average);
    arrayHours[hourpos] = average;
    hourpos++;
  }
  
  updateGraph();

  delay(DELAY);
}

String doubleToString(double input,int decimalPlaces){
  if(decimalPlaces!=0){
    String string = String((int)(input*pow(10,decimalPlaces)));
      if(abs(input)<1){
        if(input>0)
          string = "0"+string;
        else if(input<0)
          string = string.substring(0,1)+"0"+string.substring(1);
      }
      return string.substring(0,string.length()-decimalPlaces)+"."+string.substring(string.length()-decimalPlaces);
    }
  else {
    return String((int)input);
  }
}
