
/*******************MOTOR1*******************/
#define ENCA1 2 // First Encoder Channel 
#define ENCB1 4 // Second Encoder Channel 
#define PWM1 11  // Motor ENA Pin
#define IN2 10  // Motor Direction 1 Pin
#define IN1 12  // Motor Direction 2 Pin

/*******************MOTOR2*******************/
#define ENCA2 3 // First Encoder Channel 
#define ENCB2 5 // Second Encoder Channel 
#define PWM2 9  // Motor ENA Pin
#define IN3 8  // Motor Direction 1 Pin
#define IN4 13  // Motor Direction 2 Pin

/*******************STEPPER MOTOR*******************/
const int StepZ = A0;
const int DirZ = A1;

#define LimitSwitch1 A2
#define LimitSwitch2 A3
#define LimitSwitch3 A4

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
void setup() {
  Serial.begin(9600);

  //////////////////////////////////////////////////// FIRST MOTOR
  pinMode(ENCA1, INPUT); //first encoder channel
  pinMode(ENCB1, INPUT); //second channel
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING); // set interrupt to record every pulse of encoder when hits then excute rest of code lines after
  //attachInterrupt only for pins 2,3 and paramters 1- Pin 2- function being excuted 3- mode

  ////////////////////////////////////////////////////// SECOND MOTOR
  pinMode(ENCA2, INPUT); //first encoder channel
  pinMode(ENCB2, INPUT); //second channel
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING); // set interrupt to record every pulse of encoder when hits then excute rest of code lines after
  //attachInterrupt only for pins 2,3 and paramters 1- Pin 2- function being excuted 3- mode

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
        L();
        break;
      case 'T':
        T();
        break;
      case 'E':
        E();
        break;
      case 'H':
        H();
        break;
      case 'N':
        N();
        break;
    }
  

 /* PID constants
  float kp1 = 5;
  float kd1 = 0.25;
  float ki1 = 0.02;


  float kp2 = 5;
  float kd2 = 0.25;
  float ki2 = 0.02;
  // time difference
  long currT = micros(); //start recording time in micro seconds
  float deltaT1 = ((float) (currT - prevT1)) / ( 1.0e6 ); //compute delta time in seconds
  float deltaT2 = ((float) (currT - prevT2)) / ( 1.0e6 ); //compute delta time in seconds
  prevT1 = currT;  //store the current time
  prevT2 = currT;  //store the current time


  // Read the position
  int pos1 = 0;
  int pos2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos1 = posi1;
  pos2 = posi2;
  interrupts(); // turn interrupts back on

  // error
  int e1 = pos1 - target1;     //switch signal if didn't work
  int e2 = pos2 - target2;     //switch signal if didn't work

  // derivative
  float dedt1 = (e1 - eprev1) / (deltaT1); //compute the dervative
  float dedt2 = (e2 - eprev2) / (deltaT2); //compute the dervative

  // integral
  eintegral1 = eintegral1 + e1 * deltaT1; //compute teh integral
  eintegral2 = eintegral2 + e2 * deltaT2; //compute teh integral

  // control signal
  float u1 = kp1 * e1 + kd1 * dedt1 + ki1 * eintegral1;
  float u2 = kp2 * e2 + kd2 * dedt2 + ki2 * eintegral2;

  // motor power
  float pwr1 = fabs(u1);    //power is the absolute of the control signal form PID
  float pwr2 = fabs(u2);    //power is the absolute of the control signal form PID

  if ( pwr1 > 255 ) {
    pwr1 = 255;            // cap it to 255 as maximum value we can write
  }

  if ( pwr2 > 255 ) {
    pwr2 = 255;            // cap it to 255 as maximum value we can write
  }

  // motor1 direction
  int dir1 = 1;
  if (u1 < 0) {
    dir1 = -1;
  }

  // motor2 direction
  int dir2 = 1;
  if (u2 < 0) {
    dir2 = -1;
  }

  // signal the motor
  setMotor(dir1, pwr1, PWM1, IN1, IN2); //write to motor using the function and control it using delays
  //function parameters is 1- direction 2- PWM value 3- PWM pin 4- Direction 1 pin 5- Direction 2 Pin

  // signal the motor
  setMotor(dir2, pwr2, PWM2, IN3, IN4); //write to motor using the function and control it using delays
  //function parameters is 1- direction 2- PWM value 3- PWM pin 4- Direction 1 pin 5- Direction 2 Pin


  // store previous error
  eprev1 = e1;

  // store previous error
  eprev2 = e2;
  letter ="";
    Serial.print(target1);
    Serial.print(" ");
    Serial.print(pos1);
    Serial.println();

    Serial.print(target2);
    Serial.print(" ");
    Serial.print(pos2);
    Serial.println();*/
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
void penUp()
{
 analogWrite(DirZ, 0);
 for(int x = 0; x<300; x++) { // loop for 300 steps
  analogWrite(StepZ,255);
  delayMicroseconds(500);
  analogWrite(StepZ,0); 
  delayMicroseconds(500);
 }
}
///////////////////////////////////////
void penUp()
{
 analogWrite(DirZ, 255);
 for(int x = 0; x<300; x++) { // loop for 300 steps
  analogWrite(StepZ,255);
  delayMicroseconds(500);
  analogWrite(StepZ,0); 
  delayMicroseconds(500);
 }
}
//////////////////////////////////////////
/*void ZMotor(int Zdir, int steps) {
  if (Zdir == 1) {
    analogWrite(DirZ, 255);
  }
  else if (Zdir == 0) {
    analogWrite(DirZ, 0);
  }

  for (int x = 0; x < steps; x++) { // loop for 200 steps
    analogWrite(StepZ, 255);
    delayMicroseconds(500);
    analogWrite(StepZ, 0);
    delayMicroseconds(500);
  }
}*/
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
  if (motor ==1 )
  {
    // PID constants
  float kp1 = 5;
  float kd1 = 0.25;
  float ki1 = 0.02;

  // time difference
  long currT1 = micros(); //start recording time in micro seconds
  float deltaT1 = ((float) (currT1 - prevT1)) / ( 1.0e6 ); //compute delta time in seconds
  prevT1 = currT1;  //store the current time


  // Read the position
  int pos1 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos1 = posi1;
  interrupts(); // turn interrupts back on

  // error
  int e1 = pos1 - target;     //switch signal if didn't work

  // derivative
  float dedt1 = (e1 - eprev1) / (deltaT1); //compute the dervative

  // integral
  eintegral1 = eintegral1 + e1 * deltaT1; //compute teh integral

  // control signal
  float u1 = kp1 * e1 + kd1 * dedt1 + ki1 * eintegral1;

  // motor power
  float pwr1 = fabs(u1);    //power is the absolute of the control signal form PID

  if ( pwr1 > 255 ) {
    pwr1 = 255;            // cap it to 255 as maximum value we can write
  }

  // motor1 direction
  int dir1 = 1;
  if (u1 < 0) {
    dir1 = -1;
  }

  // signal the motor
  setMotor(dir1, pwr1, PWM1, IN1, IN2); //write to motor using the function and control it using delays
  //function parameters is 1- direction 2- PWM value 3- PWM pin 4- Direction 1 pin 5- Direction 2 Pin
  // store previous error
  eprev1 = e1;
//  Serial.print("Target = ");
//    Serial.print(target);
//    Serial.print(" ");
//    Serial.print("Pos1");
//    Serial.print(posi1);
//    Serial.println();
    delay(500);
  }
 else if (motor ==2)
 {
  // PID constants
  float kp2 = 5;
  float kd2 = 0.25;
  float ki2 = 0.02;
  // time difference
  long currT2 = micros(); //start recording time in micro seconds
  float deltaT2 = ((float) (currT2 - prevT2)) / ( 1.0e6 ); //compute delta time in seconds
  prevT2 = currT2;  //store the current time


  
  int pos2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos2 = posi2;
  interrupts(); // turn interrupts back on

  // error
  
  int e2 = pos2 - target;     //switch signal if didn't work

  // derivative
  float dedt2 = (e2 - eprev2) / (deltaT2); //compute the dervative

  // integral
  eintegral2 = eintegral2 + e2 * deltaT2; //compute teh integral

  // control signal
  float u2 = kp2 * e2 + kd2 * dedt2 + ki2 * eintegral2;

  // motor power
  float pwr2 = fabs(u2);    //power is the absolute of the control signal form PID

 

  if ( pwr2 > 255 ) {
    pwr2 = 255;            // cap it to 255 as maximum value we can write
  }

  // motor1 direction
  

  // motor2 direction
  int dir2 = 1;
  if (u2 < 0) {
    dir2 = -1;
  }

  // signal the motor
  //function parameters is 1- direction 2- PWM value 3- PWM pin 4- Direction 1 pin 5- Direction 2 Pin

  // signal the motor
  setMotor(dir2, pwr2, PWM2, IN3, IN4); //write to motor using the function and control it using delays
  //function parameters is 1- direction 2- PWM value 3- PWM pin 4- Direction 1 pin 5- Direction 2 Pin


  // store previous error

  // store previous error
  eprev2 = e2;
/*    Serial.print("Target = ");
    Serial.print(target);
    Serial.print(" ");
    Serial.print("Pos2");
    Serial.print(posi2);
    Serial.println();*/
    delay(500);
 }
}

/////////////////////////LETTERS FUNCTIONS///////////////////////////////
void L() {
  
ZMotor(1,500);
target1=-1200;
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

target2=2000;
posi2=0;
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
