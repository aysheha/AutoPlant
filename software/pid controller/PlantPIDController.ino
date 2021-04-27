#define SoilPin 13

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 1, 1, 1, DIRECT);

int soilValueDry = 600;	//desired threshold to start watering
int T = 240000;
int last_time;
unsigned long soilValue; //current soil sensor reading
unsigned long = soilValue;
void setup()

  pinMode(soilPin, OUTPUT);
  Setpoint = 340;
  myPID.SetOutputLimits(0, soilValue);
  //PID on
  myPID.SetMode(AUTOMATIC);
}

void PID_control() {
  
  double kp, ki, kd, setpoint, error, cont_var;
  
  ////////////////////////////////////////////////////
	//Get the current moisture level
	soilValue = analogRead(A0);
  //Establish time constant
  unsigned long now = millis();
  unsigned long last_time;
  int T; //time constant in ms
  //Delta time interval
  int delta_time = now - last_time;
	if (delta_time >= T) {
    error = SetPoint - soilValue;
    total_error += error
      if (total_error >= max_control) total_error = max_control;
    else if (total_error <= min_control) total_error = min_control;
    delta_error = error - last_error
  //Calculate PID
      control_signal = kp*error + (ki*T)*total_error + (kd*T)*delta_error
      if (control_signal >= max_control) control_signal = max_control;
			else if (control_signal <= min_control) control_signal = min_control;
  //Save current error as last error for next iteration
	last_error = error;
  last_time = now;

  }
	//integral = integral + error;
	//Calculate derivative
	//derivative = error - last_error;
	//Calculate the control variable 
  //PID_Control(); //computes control signal?
	//control_variable = (kp*error) + (ki*integral) + (kd*derivative) 
  //////////////////////////////////////////////////////
}

void loop() {
PID_Control(); 
}

void Sense_PID() {

  input = analogRead(A0);
  myPID.Compute();
  unsigned long now = millis();
  if (now - soilValueDry > soilValue)
  { //time to shift the Relay Window
    soilValueDry += soilValue
  }
  if (Output > now - soilValueDry) 
  	digitalWrite(13, HIGH);
  else digitalWrite(13, LOW);
  
}}
