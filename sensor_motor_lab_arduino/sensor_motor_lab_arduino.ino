#include<Servo.h>

// communication var
int slot = 0;    // digital sensor value
double flex = 0;   // analog sensor value
float ultrasonic = 0;    // analog sensor value
int inByte = 0;         // incoming serial byte
int ServoPos = 0;
int DCPos = 0;
int DCVel = 0;
int StepperPos_cur = 0;
int StepperPos_prev = 0;
int gui = 0;
int serialCount = 0;
boolean firstContact = false;
int serialOutArray[7];
int button0State = 0;
boolean PosOrVel = true;

// Sensor Pins
const int slotPin = A1;
const int flexPin = A0;
const int ultraPin = A2;
const int Button0 = 3 ;

// DC motor & encoder pins
const int cwPin = 6;
const int ccwPin = 10;
int hallA = 2; // for encoder
int hallB = 11; // for encoder
volatile long encoderPos=0; // global counter for encoder pulses
const double ppr = 180; // poles per revolution * reduction ratio

// Servo motor
const int servoPin = 9;
const int filterBufferSize = 11;  // Odd sized buffer for easier median searches
Servo myServo;

// Stepper motor
const int stepperDirPin = 4;
const int stepperStepPin = 5;

// Ultrasonic
class ultraSonicModule
{
  public:
  //Constructor initialization
  ultraSonicModule(int pinNumber)
  {
    sensorPin = pinNumber;
    //pinMode(sensorPin, INPUT);
    for(int i=0; i<filterBufferSize; ++i)
    {
      bufferArray[i] = 0;
      sortedArray[i] = 0;  
    }
  }
  float measureDistance(){
    for(int i=filterBufferSize-1; i>= 0; --i){
      if (i==0) bufferArray[i] = analogRead(sensorPin);
      else
      {
      bufferArray[i] = bufferArray[i-1];
      }
      sortedArray[i] = bufferArray[i];
    }
    qsort(sortedArray, filterBufferSize, sizeof(sortedArray[0]), comparator);
    return 0.514*sortedArray[int((filterBufferSize-1)/2)] + 1.632;
  }
  
  private:
  int sensorPin;
  double bufferArray[filterBufferSize];
  double sortedArray[filterBufferSize];
  
  static int comparator(int *cmp1, int *cmp2)
  {
    // Need to cast the void * to int *
    int a = *(cmp1);
    int b = *(cmp2);
    // The comparison
    return a < b ? -1 : (a < b ? 1 : 0);
  } 
};
ultraSonicModule myUltra(ultraPin);


void setup() 
{
  // start serial port at 9600 bps:
  Serial.begin(9600);
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) {;}

  // sensors
  pinMode(slotPin, INPUT);   // slot sensor is on digital pin #
  pinMode(flexPin, INPUT); // flex sensor is on analog pin 0
  pinMode(ultraPin, INPUT); // ultrasonic sensor is on analog pin #
  pinMode(hallA, INPUT); // encoder
  pinMode(hallB, INPUT); // encoder
  
  // motors
  pinMode(cwPin, OUTPUT); // DC motor pwm
  pinMode(ccwPin, OUTPUT);  // DC motor pwm
  pinMode(servoPin, OUTPUT); //Servo motor
  digitalWrite(servoPin, HIGH);
  pinMode(stepperDirPin, OUTPUT); // Stepper motor
  pinMode(stepperStepPin, OUTPUT); // Stepper motor

  attachInterrupt(digitalPinToInterrupt(hallA), encoderISR, RISING);
  myServo.attach(servoPin);

  // Button interrupt
  attachInterrupt(digitalPinToInterrupt(Button0), STATE_CH, RISING);
  
  establishContact();  // send a byte to establish contact until receiver responds
}

void loop() 
{
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) 
  {
    // get incoming byte:
    inByte = Serial.read();
    if (firstContact == false && inByte == 'A') // handshake purpose
    {
      firstContact = true;     // you've had first contact from the pc
      // send sensor values:
      Serial.write(slot);
      Serial.write(int(flex) / 255);
      Serial.write(int(flex) % 255);
      Serial.write(int(ultrasonic));
    }
    else 
    {
      // Add the latest byte from the serial port to array:
      serialOutArray[serialCount] = inByte;
      serialCount++;

      // If we have 7 bytes:
      if (serialCount > 6 ) 
      {
        // read GUI values
        if (button0State == 0) ServoPos = serialOutArray[0];
        int DCPos_tmp = (serialOutArray[1] << 8) | serialOutArray[2]; 
        if (DCPos != DCPos_tmp) PosOrVel = true;
        if (DCVel != serialOutArray[3]) PosOrVel = false;
        DCPos = DCPos_tmp;
        DCVel = serialOutArray[3];
        StepperPos_cur = (serialOutArray[4] << 8) | serialOutArray[5];
        gui = serialOutArray[6]; // 0: sensor; 1: gui

        // read sensor values
        slot = slot_reading(slotPin);
        flex = force_reading(flexPin);
        ultrasonic = myUltra.measureDistance();

        // actuate motor, use gui to determine commands
        if (gui == 1)
        {
          if (button0State == 0) myServo.write(ServoPos);
          if (PosOrVel) pid_position_control(encoderPos, DCPos);
          else pid_speed_control(DCVel);
          if (StepperPos_prev != StepperPos_cur) 
          {
            writeStepperPos(StepperPos_cur);
            StepperPos_prev = StepperPos_cur;
          }
        }
        else
        {
          servo_write(ultrasonic);
          force_to_motor(flexPin);
        }

        // send sensor values for displaying:
        Serial.write(slot);
        Serial.write(int(flex) / 255);
        Serial.write(int(flex) % 255);
        Serial.write(int(ultrasonic));
        
        // Reset serialCount:
        serialCount = 0;
      }
    }
  }
}

///////CONTROL FXNS//////////////
// given rpm from gui and parameters, write that speed to motor
void pid_speed_control(double input_rpm)
{
  int newposition = encoderPos;
  static int oldposition = 0;
  int newtime = millis();
  static int oldtime = 0;
  static double current_rpm;
  static double error;
  static double e_dot;
  static double old_e = 0;
  static double e_acc = 0;
  static double kp = 5;
  static double ki = 0.5;
  static double kd = 0.1;
  static double current_pwm = 0;
  current_rpm = RPM(oldposition,newposition,oldtime,newtime,ppr);
  if (isnan(current_rpm))
    current_rpm = 0;
  error = input_rpm - current_rpm;
  e_dot = (error - old_e) * 1000 / (newtime-oldtime);
  e_acc += error;
  current_pwm = error*kp + e_dot*kd + e_acc*ki;
  drive_motor(current_pwm);
  old_e = error;
  oldposition = newposition;
  oldtime = newtime;
  delay(50);
}
void pid_position_control(int enc, int angle)
{
  static const double kp = 3.7;
  static const double kd = 0.3;
  static const double ki = 0.1;
  static const int p_init = enc;
  static double error = 0;
  static double old_e = 0;
  static double e_acc = 0;
  double newtime = millis();
  static double oldtime = 0;
  static double e_dot = 0;
  static double output = 0;
  static double setpoint = 0;
  setpoint = map(angle, 0, 360, 0, ppr);
  error = (enc - (p_init-setpoint));
  e_dot = (error - old_e) * 1000 / (newtime-oldtime);
  e_acc += error;
  output = -1*(kp*error + kd*e_dot + ki*e_acc);
  drive_motor(output);
  oldtime = newtime;
  old_e = error;
}
void servo_write(float ultra) 
{
  // Map reading to servo input
  int servo_input = map(int(ultra), 10, 40, 10, 170);
  // Rotate Servo
  myServo.write(servo_input); 
}

void writeStepperPos(int degree)
{
  int motorSteps = int(degree * 1600.0/360);
  if(motorSteps > 0) 
    digitalWrite(stepperDirPin, LOW); // Set clockwise direction.
  else
  {
    digitalWrite(stepperDirPin, HIGH);         // Set counter-clockwise direction.
    motorSteps = abs(motorSteps);
  }
  advance(motorSteps);
}
//////////////////////////////////////////////

///////SENSOR FXNS///////////////////////////
// take a reading on the fsrPin and output the average-filtered value in g
double force_reading(int fsrPin)
{
  const float VCC = 5.0;
  const float R_DIV = 10000;
  int fsrADC = analogRead(fsrPin);
  // CALCULATE FORCE IN G FROM ANALOG READING/////////
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
  static float force = 0;
  if (fsrADC != 0) // If the analog reading is non-zero
  {
    // Use ADC reading to calculate voltage:
    float fsrV = fsrADC * VCC / 1023.0;
    // Use voltage and static resistor value to 
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    // Guesstimate force based on slopes in figure 3 of
    // FSR datasheet:
    float fsrG = 1.0 / fsrR; // Calculate conductance
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600) 
      force = (fsrG - 0.00075) / 0.00000032639;
    else
      force =  fsrG / 0.000000642857;
  }
  else
    force = 0;
  /////////////////////////////////
  // average filter
  double sum = 0;
  static const int size = 50;
  static double past[size];
  for (int i = size - 1; i > 0; i--)
  {
    past[i] = past[i-1];
    sum += past[i];
  }
  past[0] = force;
  sum += force;
  sum /= size;
  return sum;
}
int slot_reading(int slotPin)
{
  return (analogRead(slotPin) == 0) ? 1 : 0;
}
/////////////////////////////////////////////

///////HELPER FXNS///////////////////////////
// handshake with processing GUI
void establishContact() 
{
  while (Serial.available() <= 0) 
  {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}
// calculate RPM given encoder positions/times 
double RPM(double p0, double p1, double t0, double t1, double pulse_per_rev)
{
  double rpm = ((p1 - p0) * 1000) / (t1-t0);
  rpm /= pulse_per_rev;
  rpm *= 60;
  return rpm;
}
// maps an rpm to a pwm value [0 - 230]
double rpm_to_pwm(double rpm) {
  double result = map(rpm,0,60,0,255);
  if (rpm > 230)
    result = 0.0;
  return result;
}
// interrupt fxn for when a rising edge is detected on hallA
void encoderISR()
{
  if (digitalRead(hallB) == HIGH)
    encoderPos++;
  else
    encoderPos--;
}
// stepper motor advance
void advance(int steps)
{
  int i = 0;
  while (i < steps) // Rotate n steps.
  {     
    if (slot_reading(slotPin) == 0)
    { 
      digitalWrite(stepperStepPin, HIGH);
      delayMicroseconds(500);           // Minimum STEP Pulse Width 1.0uS
      digitalWrite(stepperStepPin, LOW);         // Step pulse (rising edge).
      delayMicroseconds(500); 
      i++;
    }
  }
}
// Button ISR
void STATE_CH()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200)
  {
    button0State = button0State + 1;
    if(button0State>1) button0State=0;
  }
  last_interrupt_time = interrupt_time;
}
void drive_motor(int pwm)
{
  int output = constrain(pwm,-255,255);
  if (output > 0){
    analogWrite(cwPin,abs(output));
    analogWrite(ccwPin,0);
  } else {
    analogWrite(ccwPin,abs(output));
    analogWrite(cwPin,0);
  }
}
void force_to_motor(int fsrPin)
{
  drive_motor(map(analogRead(fsrPin),0,1023,0,255));
}
/////////////////////////////////////////////////
