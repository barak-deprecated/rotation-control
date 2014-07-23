/* Control Systems - Disk Apparatus
 * Barak Alon, March 2013
 * Prepared for ME330 to demonstrate position control of a rotating disk
 * Position Calibrations: 
 * 1 = 105, 2 = 245, 3 = 383, 4 = 525, 5 = 654, 6 = 797, 7 = 930
 */


///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       DEFINITIONS                                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////////
#define magPin 3        // Define the pin for motor magnitude PWM output
#define dirPin 12       // Define the pin for motor direction output
#define sensorPin A0    // Define the pin for potentiometer reading input


int setPoint = 525;     // Sets the desired disk angle --- ****ADD FUNCTION****


int sensorReading = 0;  // Variable to store the potentiometer (sensor) reading
float error;              // Variable to store the error value between the set point and sensor feedback
float previousError = 0.0;  // Variable to store the error of the last iteration
float controlSignal;      // Variable to store the signal to be sent to the motor after compensation

unsigned long thisTime = 0; // Variable to store how much time has passed, to be utilized by the PID function
unsigned long lastTime = 0; // Variable to keep track of how much time passed at last iteration

float antiWindupThreshold = 300.0;
float area = 0.0;           // Variable to contain the accumulation of area during integral compensation


///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            SETUP                                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(magPin, OUTPUT);    // Declare the magnitude pin as output
  digitalWrite(magPin, LOW);  // Ensures that the output to the motor is off at first
  pinMode(dirPin, OUTPUT);    // Declare the direction pin as output
  digitalWrite(dirPin, LOW);  // Sets the initial direction of the motor output
//  Serial.begin(115200);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        MAIN FUNCTION                                              //
///////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  sensorReading = analogRead(sensorPin);      // Read the disk position from potentiometer  
  error = (float)(setPoint - sensorReading);  // Calculate the error as the disk position relative to 
                                                  // the set point (where it should be)
  controlSignal = PID();                      // Calculate the control signal to be sent to the motor, 
                                                  // which will optimally correct the disk position
  drive(controlSignal);                       // Send the new control signal to the motor
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         DRIVE MOTOR                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////
void drive(float input) {
  int motorDirection;        // The motor driver can't interpret negative numbers - this integer will
                             // will tell the motor driver which direction to drive
  if(input < 0)              
    motorDirection = LOW;
  else
    motorDirection = HIGH;
  
  input = abs(input);        // Get rid of any negative value
  
  if(input > 255.0)          // 255 represents the maximum voltage that can be sent to the motor
    input = 255.0;           // This limits the signal being sent to the motor
  
  digitalWrite(dirPin, motorDirection);    // Set the correct direction to drive
  analogWrite(magPin,input);               // Send the signal to the motor
}



///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            PID                                                    //
///////////////////////////////////////////////////////////////////////////////////////////////////////
float PID() {
  // Declare the Proportional, Integral, and Derivative constants, respectively
  float Kp = 30.0;
  float Ki = 10.0; 
  float Kd = 100.0;
  
  float PIDoutput; // Variable to store the output of the controller, to be returned
  
  float h;         // Variable to store the increment of time since last calculation
  
  thisTime = micros();  // Store the current time in thisTime
  h = (float)((thisTime - lastTime)/1000000.0);   // Calculate the current time increment


  // PROPORTIONAL CONTROL
  // --------------------
  float pComponent;
  pComponent = Kp * error;

  // INTEGRAL CONTROL
  // ----------------
  float iComponent;
  if(abs(error) >= antiWindupThreshold)    // This prevents "windup" from too much integral buildup
    area = 0.0;
  else if(error*previousError <= 0)
    area = 0.0;
  else
    area = area + error*h;
  iComponent = Ki * area; 

  // DERIVATIVE CONTROL
  // ------------------
  float dComponent;
  if(abs(error) >= 50)    // Derivative control slows corrections down - this limits the slowing effect to when the
    dComponent = 0;       // disk is getting close to the set point
  else 
    dComponent = Kd * ((error-previousError)/h);


  PIDoutput = pComponent + iComponent + dComponent;    // Sum up all the signal components to be sent to the motor
  
//  Serial.print(thisTime);
//  Serial.print("\t");
//  Serial.println(sensorReading);
  
  // Set variables up for the next iteration
  lastTime = thisTime;
  previousError = error;
  
  return PIDoutput;    // Send the output to be processed by the motor
}
