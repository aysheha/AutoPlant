#include <LCD_I2C.h> //LCD & I2C 
#include <DHT.h>     //DHT22 sensor
#include <PID_v1.h>  //PID 

#define VCC2 5       // 5V for LCD screen
#define VCC3 6       // 5V for Humidity sensor
#define GND3 7       // GND for Humidity sensor
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

LCD_I2C lcd(0x27);

const int soilDry = 600;  //start watering indicator
const int soilValue = 340; // moisture threshold
const int soilMin = 100; //moisture minimum for system
const int soilMax = 1000; //moisture maximum for system
const int soilPin = A0; // Soil Moisture pin [analog]

const int pump = 13;    // Relay output Pin [Digital]
int pumpTmin = 1000; //pump timer min
int pumpTmax = 5000; //pump timer max
int pumpTime;

double setPoint, soil, Output;  //PID controller Threshold, Input, Output
int kp = 2;
int ki = 0.025;
int kd = 1;
int T = 5000;
int previousTime;
int error;
int total_error;
int delta_error;
int last_error;
unsigned long control_signal;
int maxSignal = 10000;
int minSignal = 40;

unsigned long currentTime = millis();

PID myPID(&soil, &Output, &setPoint, kp, ki, kd, DIRECT);

void setup() {
  pinMode(VCC2, OUTPUT);    //LCD screen voltage output
  digitalWrite(VCC2, HIGH); //set LCD screen 

  pinMode(VCC3, OUTPUT);    //humidity  voltage output
  digitalWrite(VCC3, HIGH); //set humidity sensor volt 
  pinMode(GND3, OUTPUT);    //humidity sensor ground
  digitalWrite(GND3, LOW); //set humidity sensor ground 
  
  pinMode(pump, OUTPUT);       //set relay input pin 
  pinMode(soilPin, INPUT);     //set soil moisture pin
  Serial.begin(9600);          //begin baudrate of pump
  setPoint = soilPin;
  digitalWrite(pump, HIGH);    //set pump high
  delay(100);

  myPID.SetOutputLimits(pumpTmin, pumpTmax);
  myPID.SetMode(AUTOMATIC);

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
  
  dht.begin();

}

void loop() {               //main loop
  int soil = analogRead(soilPin);  //obtain moisture reading
  int humid = dht.readHumidity();
  float temp = dht.readTemperature();

  int soilPrev;
  int total_errorPrev;
  unsigned long currentTime = millis();
  error = soilValue - soil;
  total_error = total_errorPrev + error; //ki error 
  
  lcd.setCursor(0, 0); 
  lcd.print("Reading...");    //LCD print reading
  delay(5000);              //display for 5 secs
  lcd.backlight();
  lcd.clear();             //clear display

  lcd.setCursor(0, 2);
  lcd.print("M:");
  lcd.setCursor(2, 2);
  lcd.print(soil);
  lcd.setCursor(9, 2);
  lcd.print("H:");
  lcd.print(humid);
  delay(5000);              //display for 5 secs
  lcd.backlight();
  lcd.clear();             //clear display
  
  //threshold if greater than or equal to 340. 
  // if analog is A0 >= 1000 - sensor not setup right/calibrated wrong 
  // if 700 > A0 >= 600 - soil is dry 
  // if 600 > A0 >=370 - soil is humid; verify with humidity sensor 
  // if A0 < 370 - Moisture is good

  if (total_error <= -260) { // plant is dry - water plant
    int soilNow = analogRead(soilPin);
    int soilPrev;
    int total_errorPrev;
    
    error = soilValue - soilNow;     //kp error
    total_error = total_errorPrev + error; //ki error 
    delta_error = soilNow - soilPrev; //kd error
    control_signal = kp*T*error + (ki*T)*total_error + (kd*T)*delta_error;
    control_signal = control_signal/100000;
    lcd.setCursor(8, 2);
    lcd.print(control_signal);
    delay(5000);
    lcd.backlight();
    lcd.clear();

    int pumpTnow;
    if(control_signal >= maxSignal){
      control_signal = maxSignal;
      pumpTnow = pumpTmax;
      lcd.setCursor(0, 2);
      lcd.print("Max");
      lcd.setCursor(8, 2);
      lcd.print(control_signal);
      delay(2000);
      lcd.backlight();
      lcd.clear(); 
    }else if(control_signal <= minSignal){
      control_signal = minSignal;
      pumpTnow = pumpTmin;
      lcd.print("Min");
      lcd.setCursor(8, 2);
      lcd.setCursor(8, 2);
      lcd.print(control_signal);
      delay(2000);
      lcd.backlight();
      lcd.clear();
    } else {
      pumpTnow = (control_signal*5)/1000; 
      //pumpTnow = pumpTnow*100;
      lcd.print("New");
      lcd.setCursor(8, 2);
      lcd.setCursor(8, 2);
      lcd.print(control_signal);
      delay(2000);
      lcd.backlight();
      lcd.clear();
    }
    
    lcd.setCursor(0, 0);  
    lcd.print("Start Watering.");
    digitalWrite(pump, HIGH); //relay is open - pump is off
    // keep pump off for 5ms
    delay(500);
    lcd.backlight();
    lcd.clear();
    lcd.print("Watering Now.");
    digitalWrite(pump, LOW);//relay is closed - pump is ON
    lcd.setCursor(0, 2);
    lcd.print(pumpTnow);
    lcd.setCursor(8, 2);
    lcd.print(control_signal);
    delay(pumpTnow);         // pump is on for 5 seconds
    digitalWrite(pump, HIGH);//relay is open - pump is off
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
    lcd.print(humid);
    delay(500);
    lcd.clear();
    
    soilPrev = soilNow;
    total_errorPrev = total_error; 
    
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
    lcd.print(humid);
    
  }
}
