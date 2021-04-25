#include <LCD_I2C.h> //LCD & I2C 
#define VCC2 5        // 5V for LCD screen
//#define VCC3 6      // 5V for Humidity sensor
//#define GND3 2      // GND for Humidity sensor

LCD_I2C lcd(0x27);

const int dry = 340; // moisture threshold
const int dryMin = 100; //moisture minimum
const int dryMax = 700; //moisture maximum

const int pump = 13;    // Relay Input Pin [Digital]
const int soilPin = A0; // Soil Moisture pin [analog]

void setup() {
  pinMode(VCC2, OUTPUT);    //LCD screen voltage output
  digitalWrite(VCC2, HIGH); //set LCD screen 

  //pinMode(VCC3, OUTPUT);    //humidity  voltage output
  //digitalWrite(VCC23, HIGH); //set humidity sensor volt 
  //pinMode(GND3, OUTPUT);    //humidity sensor ground
  //digitalWrite(GND2, HIGH); //set humidity sensor ground 
  
  pinMode(pump, OUTPUT);       //set relay input pin 
  pinMode(soilPin, INPUT);     //set soil moisture pin
  Serial.begin(9600);          //begin baudrate of pump
  digitalWrite(pump, HIGH);    //set pump high
  delay(100);

  lcd.begin();                 //set up lcd screen
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Setup Complete.");//system is ready
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Ready!");
  delay(2000);
  lcd.clear();
}

void loop() {               //main loop
  int soilValue = analogRead(soilPin);  //obtain moisture reading
  float soil = soilValue;    //command line to output on LCD
  lcd.setCursor(0, 0); 
  lcd.print("Reading...");    //LCD print reading
  lcd.print(soil);          //print soil value
  delay(5000);              //display for 5 secs
  lcd.backlight();
  lcd.clear();             //clear display

  lcd.setCursor(0, 2);
  lcd.print("M:");
  lcd.setCursor(2, 2);
  lcd.print(soil);
  lcd.setCursor(9, 2);
  lcd.print("H:");
  
  //threshold if greater than or equal to 340. 
  // if analog is A0 >= 1000 - sensor not setup right/calibrated wrong 
  // if 1000 > A0 >= 600 - soil is dry 
  // if 600 > A0 >=370 - soil is humid; verify with humidity sensor 
  // if A0 < 370 - Moisture is good
  
  if (soilValue >= dry) {
    // plant is dry - water plant
    lcd.setCursor(0, 0);  
    lcd.print("Start Watering.");
    digitalWrite(pump, HIGH); //relay is open - pump is off
    // keep watering for 5 sec
    delay(500);
    lcd.backlight();
    lcd.clear();
    digitalWrite(pump, LOW);//relay is closed - pump is ON

    lcd.print("Watering Now.");
    digitalWrite(pump, HIGH);//relay is open - pump is off
    delay(5000);              // pump is on for 5 seconds
    lcd.backlight();
    lcd.clear();
    lcd.print("Watering Done.");
    delay(500);
    lcd.clear();
    
    lcd.setCursor(0, 2);
    lcd.print("M:");
    lcd.setCursor(2, 2);
    lcd.print(soil);
    lcd.setCursor(9, 2);
    lcd.print("H:");
  } else {
    digitalWrite(pump, HIGH);//relay is open - pump is off
    lcd.setCursor(0, 0); 
    lcd.print("Moisture Good.");
    delay(5000);
    lcd.backlight();
    lcd.clear();
    delay(500);
    lcd.setCursor(0, 2);
    lcd.print("M:");
    lcd.setCursor(2, 2);
    lcd.print(soil);
    lcd.setCursor(9, 2);
    lcd.print("H:");
  }

}
