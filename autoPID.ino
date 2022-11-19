#include <PID_v1.h>

/*******************MOTOR1*******************/
#define ENCA1 3 // First Encoder Channel 
#define ENCB1 5 // Second Encoder Channel 
#define PWM1 9  // Motor ENA Pin
#define IN2 8  // Motor Direction 1 Pin
#define IN1 11  // Motor Direction 2 Pin

/*******************MOTOR2*******************/
#define ENCA2 2 // First Encoder Channel 
#define ENCB2 4 // Second Encoder Channel 
#define PWM2 10  // Motor ENA Pin
#define IN3 7 // Motor Direction 1 Pin
#define IN4 6 // Motor Direction 2 Pin


/*******************STEPPER MOTOR*******************/
const int StepZ = 12;
const int DirZ = 13;

#define LimitSwitch1 A0
#define LimitSwitch2 A1
#define LimitSwitch3 A2

volatile int target1;
volatile int target2;

//Read Input
char letter;

volatile int posi1 = 0; //  Used for Interrupt Service Routines function specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int posi2 = 0; //  Used for Interrupt Service Routines function specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

long prevT1 = 0;       // Previous Time for integral part
float eprev1 = 0;      //previous error for derivative
float eintegral1 = 0;  //error integral coefficient

long prevT2 = 0;       // Second Previous Time for integral part
float eprev2 = 0;      //Second previous error for derivative
float eintegral2 = 0;  //Second error integral coefficient

double kp1 = 10 , ki1 = 6 , kd1 = 0.02; 
double kp2 = 7 , ki2 = 2 , kd2 = 0.02;


volatile long encoderValue1 = 0; // Raw encoder value
volatile long encoderValue2 = 0; // Raw encoder value
int PPR = 120;  // Encoder Pulse per revolution.
int angle = 360; // Maximum degree of motion.
int REV = 0;          
double input1 = 0, output1 = 0, setpoint1 = 0;
PID myPID1(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);  
double input2 = 0, output2 = 0, setpoint2 = 0;
PID myPID2(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT); 


void setup() {
  Serial.begin(9600);

  //////////////////////////////////////////////////// FIRST MOTOR
  pinMode(ENCA1, INPUT_PULLUP); 
  pinMode(ENCB1, INPUT_PULLUP);
  digitalWrite(ENCA1, HIGH); //turn pullup resistor on
  digitalWrite(ENCB1, HIGH); //turn pullup resistor on
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING); // set interrupt to record every pulse of encoder when hits then excute rest of code lines after
  //attachInterrupt only for pins 2,3 and paramters 1- Pin 2- function being excuted 3- mode

  ////////////////////////////////////////////////////// SECOND MOTOR
  pinMode(ENCA2, INPUT_PULLUP); 
  pinMode(ENCB2, INPUT_PULLUP);
  digitalWrite(ENCA2, HIGH); //turn pullup resistor on
  digitalWrite(ENCB2, HIGH); //turn pullup resistor on
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING); // set interrupt to record every pulse of encoder when hits then excute rest of code lines after
  //attachInterrupt only for pins 2,3 and paramters 1- Pin 2- function being excuted 3- mode

  //////////////////////////////////////////////////////////////
  
  myPID1.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID1.SetSampleTime(1);  // refresh rate of PID controller
  myPID1.SetOutputLimits(-255, 255); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  myPID2.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID2.SetSampleTime(1);  // refresh rate of PID controller
  myPID2.SetOutputLimits(-255, 255); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  ////////////////////////////////////////////////////// Z MOTOR
  pinMode(StepZ, OUTPUT);
  pinMode(DirZ, OUTPUT);

  pinMode(PWM1, OUTPUT); //MOTOR Speed
  pinMode(IN1, OUTPUT); //Motor Direction
  pinMode(IN2, OUTPUT); //Motor Direction

  pinMode(PWM2, OUTPUT); //MOTOR Speed
  pinMode(IN3, OUTPUT); //Motor Direction
  pinMode(IN4, OUTPUT); //Motor Direction
}

void loop() {
  // ZMotor(1,200);       //Zmotor(int Direction 1 or 0, int steps)
  // set target position
  //int target1 = 0;    //set point
  //int target2 = 0;    //set point
  //int target = 250*sin(prevT/1e6);

  if (Serial.available() > 0) {
    /*******homing*******/
    /*if (LimitSwitch1 != true && LimitSwitch2 == true) {
      analogWrite(IN1, 0);
      analogWrite(IN2, 255);
      analogWrite(PWM1, 255);

      analogWrite(IN3, 0);
      analogWrite(IN4, 0);
      analogWrite(PWM1, 0);
    }
    else if (LimitSwitch1 != true && LimitSwitch2 != true) {
      analogWrite(IN1, 0);
      analogWrite(IN2, 255);
      analogWrite(PWM1, 255);

      analogWrite(IN3, 0);
      analogWrite(IN4, 255);
      analogWrite(PWM1, 255);
    }

    else if (LimitSwitch1 == true && LimitSwitch2 != true) {
      analogWrite(IN1, 0);
      analogWrite(IN2, 0);
      analogWrite(PWM1, 0);

      analogWrite(IN3, 0);
      analogWrite(IN4, 255);
      analogWrite(PWM1, 255);
    }
    else {}*/

    letter = Serial.read();
    Serial.println(letter);
    switch (letter) {
       case '+':
        xp();
        break;
       case '-':
        xn();
        break;
       case '*':
        yp();
         break;
       case '/':
        yn();
        break;
       
      case 'L':
        L(1000,1000);
        break;
      
      
      
    }
letter = "";
}

}
/////////////////////////////////////
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
 // Serial.println(pwmVal);
 // Serial.println(dir);
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (dir == -1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
/////////////////////////////////
void ZMotor(int Zdir, int steps) {
  if (Zdir == 1) {
    digitalWrite(DirZ, HIGH);
  }
  else if (Zdir == 0) {
    digitalWrite(DirZ, LOW);
  }

  for (int x = 0; x < steps; x++) { // loop for 200 steps
    digitalWrite(StepZ, HIGH);
    delayMicroseconds(500);
    digitalWrite(StepZ, LOW);
    delayMicroseconds(500);
  }
}
///////////////////////////////////////////
void readEncoder1() {                //Interrupt function on first cahnnel encoder
  int b = digitalRead(ENCB1);        //Read the second channel encoder
  if (b > 0) {                      //if changed first and it is HIGH means it preceds the A channel
    posi1++;                         //Increase postion by one degree
  }
  else {                            //Channel B before channel A means it is still lOW
    posi1--;                         //Motor moves in other direction and increase one in that direction
  }
}

void readEncoder2() {                //Interrupt function on first cahnnel encoder
  int c = digitalRead(ENCB2);        //Read the second channel encoder
  if (c > 0) {                      //if changed first and it is HIGH means it preceds the A channel
    posi2++;                         //Increase postion by one degree
  }
  else {                            //Channel B before channel A means it is still lOW
    posi2--;                         //Motor moves in other direction and increase one in that direction
  }
}
/////////////////////////////////////////////////////////////////////////

void xp() {
analogWrite(PWM1, 255);
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
delay(1000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
}

void xn() {
analogWrite(PWM1, 255);
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
delay(1000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
}

void yp() {
analogWrite(PWM2, 255);
digitalWrite(IN3, LOW);
digitalWrite(IN4, HIGH);
delay(1000);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
}

void yn() {
analogWrite(PWM2, 255);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);
delay(1000);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
}

void pid(int motor, int target){
  if (motor == 1)
  {
  //REV = map (target, 0, 360, 0,360); // mapping degree into pulse
  setpoint1 = target;                    //PID while work to achive this value consider as SET 
  while ( posi1 <= setpoint1 - 30 || posi1 >= setpoint1 +  30 ) {     //msh target wla setpoint 
  input1 = posi1 ;           // data from encoder consider as a Process value     mfrod hna posi1 or posi2
 Serial.print("encoderValue1  ");
 Serial.println(posi1);
  myPID1.Compute();                 // calculate new output
                                
  if (output1 > 0) {                         // if REV > encoderValue motor move in forward direction.    
    analogWrite(PWM1, output1);         // Enabling motor enable pin to reach the desire angle
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);                           // calling motor to move forward
  }
  else {
    analogWrite(PWM1, abs(output1));          // if REV < encoderValue motor move in forward direction.                       
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);                            // calling motor to move reverse
        }
  }
  }
  else if (motor == 2)
  {
    //REV = map (target, 0, 360, 0,360); // mapping degree into pulse
  setpoint2 = target;                    //PID while work to achive this value consider as SET value
  while ( posi2 < setpoint2 + 35 || posi2 > setpoint2 - 35 ) {
  
  input2 = posi2 ;           // data from encoder consider as a Process value
 Serial.print("encoderValue2  ");
 Serial.println(posi2);
  myPID2.Compute();                 // calculate new output
                                
  if (output2 > 0) {                         // if REV > encoderValue motor move in forward direction.    
    analogWrite(PWM2, output2);         // Enabling motor enable pin to reach the desire angle
    
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);                           // calling motor to move forward
  }
  else {
    analogWrite(PWM2, abs(output2));          // if REV < encoderValue motor move in forward direction.                      
        
digitalWrite(IN3, LOW);
digitalWrite(IN4, HIGH);                            // calling motor to move reverse
        }
    }
  }
}

/////////////////////////LETTERS FUNCTIONS///////////////////////////////
void L(int target1 , int target2) {
  
ZMotor(1,500);
target1=-1200;      //target1 -=1200;
posi1=0;
pid(1,target1);
Serial.print("posi1 = ");
Serial.print(posi1);
Serial.print("\t");
//pid(2,0);  

//delay(3000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);

target2+=2000;
posi2=0;
pid(2,target2);
Serial.print("posi2 = ");
Serial.print(posi2);
Serial.println("");  

digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
}



void T() {
target1=-2400;
posi1=0;
while(abs(target1) >= abs(posi1)){
pid(1,target1);
Serial.print("posi1 = ");
Serial.print(posi1);
Serial.print("\t");
//pid(2,0);  
}
//delay(3000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);

target1=-1800;
while(target1 >= posi1){
pid(1,target1);
Serial.print("posi1 = ");
Serial.print(posi1);
Serial.print("\t");
//pid(2,0);  
}
//delay(3000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);

target2=1500;
while(abs(target2) >= abs(posi2)){
pid(2,target2);
Serial.print("posi2 = ");
Serial.print(posi2);
Serial.println("");  
}
//pid(1,target1);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
}

void N() {

}

void E() {

}

void H() {
target1=-2400;
posi1=0;
while(abs(target1) >= abs(posi1)){
pid(1,target1);
Serial.print("posi1 = ");
Serial.print(posi1);
Serial.print("\t");
//pid(2,0);  
}
//delay(3000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);

target1=-1800;
posi1=0;
while(abs(target1) >= abs(posi1)){
pid(1,target1);
Serial.print("posi1 = ");
Serial.print(posi1);
Serial.print("\t");
//pid(2,0);  
}
//delay(3000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);

target2=1500;
posi2=0;
while(abs(target2) >= abs(posi2)){
pid(2,target2);}
//delay(3000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);

target1=1;
posi1=0;
while(abs(target1) >= abs(posi1)){
pid(1,target1);
Serial.print("posi1 = ");
Serial.print(posi1);
Serial.print("\t");
//pid(2,0);  
}
//delay(3000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);

target1=1;
posi1=-2400;
while(abs(target1) >= abs(posi1)){
pid(1,target1);
Serial.print("posi1 = ");
Serial.print(posi1);
Serial.print("\t");
//pid(2,0);  
}
//delay(3000);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);

}
