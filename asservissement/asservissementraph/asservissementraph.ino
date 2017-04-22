

/*
  Motor Controller software
  This software is due to interface the computer  and the motor controller, receiving
  the signals from the encoders and doing the system control.



  //the content below is to be deleted when ...
  To test the loop function, that is, the PID itself, just set desired values for
  Target speed and Target position at the beggining of the loop.

  to test the serial communication, adapt the loop code to blink a led, and you can use
  your own computer o test it at first
*/

long temps = 0;
float coefVitesseR=1;
float coefVitesse = 0.6;
int tempsInitial = 0;
int compteurZeroDistance = 0;
int compteurZeroAngle = 0;
unsigned long currentTime_1;
unsigned long loopTime = 0;
const int pin_G_A = 2;
const int pin_G_B = 4;
const int pin_D_A = 3;
const int pin_D_B = 1;
unsigned int encoder_A;
unsigned int encoder_G_B;
unsigned int encoder_D_B;
unsigned int encoder_A_prev = 0;
int compteur = 0;
int compteur_prev = 0;

// associating input pins constants - depending on the microcontroller (ATMega328; check Arduino ATMega328 pinout for details)
#define Left_INT 1 // correspond à la voie 2
#define Right_INT 0 // correspond à la voie 3
#define Left_A A4
#define Left_B A5
#define Right_A 2
#define Right_B A1

// associating output pins constants - depending on the microcontroller (ATMega328; check Arduino ATMega328 pinout for details)
#define Status_Blue A3
#define Status_Red A2
#define EN_L 7
#define PWM_L 5
#define EN_R 8
#define PWM_R 6
#define Serial_Send 0 //mod

// communication constants
#define UART_BAUD 19200

// motor control constants
#define FORWARD 0
#define REVERSE 1
#define STOP 2
#define LEFT 0
#define RIGHT 1

// PID mode constants
#define Coord_PD 0
#define Speed_PD 1
#define Coord_PID 2
#define Speed_PID 3
#define New_Coord_PD 4

// other constants
#define m_pi 3.14159

// robot parameters variables
int trackWidth, wheelDiameter, cpr, motorWheelDiameter;
int leftWheelDiameter, rightWheelDiameter;

// ### UART communication variables
// receive
unsigned char cardType, cardIndex, cardCommand, cardArgCount;
// send
unsigned char replyCommand, replyArgCount;
unsigned char cardArg[16];
unsigned char replyArg[16];


// current location variables
volatile long leftClicks, rightClicks;
long deltaClicks;
long lastLeftClicks, lastRightClicks;
long deltaLeftClicks, deltaRightClicks;
float deltaAvgClicks;
float angle, tempAngle, lastAngle, avgAngle, tempAngleLeft, tempAngleRight;
long intAngle;
float xClicks, yClicks;
float x, y;
long xInt, yInt;
long deltaForwardLeft, deltaForwardRight, deltaForward;
long intDeltaAngle;
float deltaAngle, deltaAngleLeft, deltaAngleRight;

bool hasArrived;

// ### PID variables ###
int PIDmode;
boolean PIDautoswitch;
long lastPWM;
long errorThresholdSlow, errorThresholdStop;

// new coordinate PD

long erreurDistance, erreurAngle;
long distanceTarget, angleTarget;
long distanceTarget2 = 0;
long distanceDer, angleDer;
long lastErreurAngle, lastErreurDistance;
long distanceInt, angleInt;
bool checkSeq;



// coordinate PID
long leftTarget, rightTarget;
long leftError, rightError;
long lastLeftError, lastRightError;
long leftDer, rightDer;
long newLeftTarget, newRightTarget;

const int kPcoord = 4000; // WARNING: the value is divided by 1024   anciennes valeurs: 1100, 4000, 0.65
const int kDcoord = 3000; // WARNING: the value is divided by 1024
const int kIcoord = 0.7;
// speed PI
long leftSpeed, rightSpeed;
long leftTargetSpeed, rightTargetSpeed;
long leftSpeedError, rightSpeedError;
long lastLeftSpeedError, lastRightSpeedError;
long leftSpeedDer, rightSpeedDer;
long leftSpeedInt, rightSpeedInt;

unsigned long currentTime, lastTime, deltaTime;

const int kPspeed = 25; // WARNING: the value is divided by 1024
const int kDspeed = 1000; // WARNING: the value is divided by 1024
const int kIspeed = 1; // WARNING: the value is divided by 1024

const long maxSpeedInt = 1024 / kIspeed + 1;
const long maxIntError = 200;

// robot behaviour global variables
long leftPWM, rightPWM;
long leftBigPWM, rightBigPWM;
long maxLeftPWM, maxRightPWM;
int maxSpeedC, maxAccC;

// temporary variables
bool statusBlue = false;
unsigned char i;
float tempPWM;
float tempDelta;
int tempPWMsign;
float angleTargetTemp;
float distanceTargetTemp;


void incr_right() {
  encoder_D_B = digitalRead(pin_D_B);
  if (encoder_D_B) {
    // B is high so clockwise
    rightClicks ++;


  }
  else {
    
    // B is low so counter-clockwise

    rightClicks --;
  }

}
void incr_left() {
  encoder_G_B = digitalRead(pin_G_B);

  if (encoder_G_B) {
    // -B is high so cclockwise
    leftClicks --;


  }
  else {
    // -B is low so clockwise

    leftClicks ++;
  }
}


// ##### The initialization function, runs once at startup. #####

void setup()
{
  //capteurs
  Serial.begin(9600);

  pinMode(pin_G_A, INPUT);
  pinMode(pin_G_B, INPUT);
  pinMode(pin_D_A, INPUT);
  pinMode(pin_D_B, INPUT);

  

  // Robot construction values
  cpr = 300; // number of counts per revolution of the encoder
  wheelDiameter = 70; // ENCODER wheel diameter in milimeters
  leftWheelDiameter =70; // LEFT ENCODER wheel diameter in milimeters
  rightWheelDiameter = 70; // RIGHT ENCODER wheel diameter in milimeters
  trackWidth = 270; // the distance between the wheels in milimeters
  motorWheelDiameter = 70; //  MOTOR wheel diameter in milimeters

  // configuring I/O digital ports
  //pinMode(Left_A, INPUT);  // encoder A left input
  //pinMode(Left_B, INPUT);  // encoder B left input
  //pinMode(Right_A, INPUT);  // encoder A right input
  //pinMode(Right_B, INPUT);  // encoder B right input

  pinMode(Status_Red, OUTPUT); // UART Status LED output
  pinMode(Status_Blue, OUTPUT); // Loop Status LED output
  pinMode(Serial_Send, OUTPUT); // UART Send Enable output

  pinMode(PWM_L, OUTPUT);  // PWM left output
  pinMode(PWM_R, OUTPUT);  // PWM right output
  pinMode(EN_L, OUTPUT);  // direction left output
  pinMode(EN_R, OUTPUT);  // direction right output

  // overriding the PWM frequency; we have to use a demultiplication factor of 1 in order to achieve an ultrasonic frequency and avoid the annoying buzzing
  // !!! WARNING !!! the default prescaling factor was 64 and the use of this function influences the delay(), millis(), micros() functions
  // the delay() will have to be 64x higher, while the actual values of millis() and micros() are 64x lower
  // setPwmFreq(PWM_L, 1);
  // setPwmFreq(PWM_R, 1);

  // PID parameters - default values - DO NOT TOUCH
  leftError = 0;
  rightError = 0;
  lastLeftError = 0;
  lastRightError = 0;
  leftDer = 0;
  rightDer = 0;
  leftPWM = 0;
  rightPWM = 0;
  leftClicks = 0;
  rightClicks = 0;
  leftTarget = 0;
  rightTarget = 0;

  // attaching the interrupt functions for the wheel encoders
  attachInterrupt(Right_INT, incr_right, FALLING);
  attachInterrupt(Left_INT, incr_left, FALLING);

  

  Serial.setTimeout(1000); // nécessaire


  errorThresholdSlow = (long) (cpr / 2);//by default the thresholder is half of a circle - you can chang it
  errorThresholdStop = 200;//             by default the thresholder is one fifth of a circle
  PIDautoswitch = false;

  // setting maximum PWM values
  if (PIDmode == Speed_PD || PIDmode == Speed_PID)
  {
    maxLeftPWM = 128 + 36 * (leftTargetSpeed + rightTargetSpeed) / 40;
    maxRightPWM = maxLeftPWM;
  }
  else
  {
    maxLeftPWM = 180;
    maxRightPWM = 180;
  }

  aller(500,90);

  /*  long turnss = 2;
    leftTarget -= turnss*cpr;
    rightTarget = turnss*cpr;


    Serial.print("leftTarget: ");
    Serial.println(leftTarget);
    Serial.println();

    leftTargetSpeed  = 30;
    rightTargetSpeed = 30;

      PIDmode = Coord_PD;
      PIDmode   = Speed_PD;*/
}

void leftUpdate() // update the encoder values for the LEFT motor
{
  //Serial.println("leftupdate");
  if (PIND & B00000100) //Thats equivalent to digitalRead, only we read it inside the registers, so as it's much faster. Check Port Manipulation out
  {

    if (PIND & B00010000) leftClicks++;
    else leftClicks--;
  }
  else
  {
    if (PIND & B00010000) leftClicks--;
    else leftClicks++;
  }

}

void rightUpdate() // update the encoder values for the RIGHT motor
{

  if (PIND & B00001000) //left A//if ((PINC>>PINC0) & 1) // equivalent to "if (PINC.0 == 1)"
  {

    if (PIND & B00000010) rightClicks--; // left B // equivalent to "if (PINC.1 == 1)"
    else rightClicks++;
  }
  else
  {


    if (PIND & B00000010) {
      rightClicks++;

    } // Left B//equivalent to "if (PINC.1 == 1)"
    else {
      rightClicks--;

    }
  }
}

void setMotorDir(int motor, int dir) // set the motor direction -- REVIEWED
{
  if (motor == LEFT)
  {
    if (dir == 0) digitalWrite(EN_L, LOW);//      FORWARD
    else if (dir == 1) digitalWrite(EN_L, HIGH);//REVERSE
    else if (dir == 2) digitalWrite(EN_L, LOW);//STOP
  }
  else if (motor == RIGHT)
  {
    if (dir == 0) digitalWrite(EN_R, HIGH);
    else if (dir == 1) digitalWrite(EN_R, LOW);
    else if (dir == 2) digitalWrite(EN_R, LOW);
  }
}

void setMotor(int motor, int dir, int speed) // set the motor parameters -- REVIEWED
{
  setMotorDir(motor, dir);
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  if (motor == LEFT) analogWrite(PWM_L, speed);
  else if (motor == RIGHT) analogWrite(PWM_R, speed);
}

void setParameters(long dist, long ang)
{
  distanceTarget = 0;
  distanceTarget2 = dist;
  angleTarget = ang;

  distanceTargetTemp = (float)distanceTarget2;
  distanceTargetTemp = distanceTargetTemp * (float)cpr / ((float) m_pi * (float) wheelDiameter);
  distanceTarget2 = (long) distanceTargetTemp;

  angleTargetTemp = (float)angleTarget;
  angleTargetTemp = angleTargetTemp * trackWidth * cpr / (180 * wheelDiameter);
  angleTarget = (long) angleTargetTemp;

  PIDmode = New_Coord_PD;
  PIDautoswitch = false;
  hasArrived = false;
  checkSeq = true;
  compteurZeroAngle=0;
  compteurZeroDistance=0;
}

void Asserv()
{
  

  erreurDistance = distanceTarget - (leftClicks + rightClicks) / 2;
  erreurAngle = angleTarget - (rightClicks - leftClicks);

  Serial.println(erreurAngle);
  Serial.println(erreurDistance);
  

  distanceDer = erreurDistance - lastErreurDistance;
  angleDer = erreurAngle - lastErreurAngle;

  distanceInt += erreurDistance;
  angleInt += erreurAngle;

  lastErreurDistance = erreurDistance;
  lastErreurAngle = erreurAngle;

  leftPWM = (kPcoord * (erreurDistance - erreurAngle) + kDcoord * (distanceDer - angleDer) + kIcoord * (distanceInt - angleInt) ) / 1024;
  rightPWM = (kPcoord * (erreurDistance + erreurAngle) + kDcoord * (distanceDer + angleDer) + kIcoord * (distanceInt + angleInt) ) / 1024;

  //compensating the non-linear dependency speed = f(PWM_Value)
  tempPWM = (float) abs(leftPWM) / 255.0;
  tempPWMsign = leftPWM / abs(leftPWM);
  tempPWM = pow(tempPWM, 0.2);
  tempPWM = 255.0 * tempPWM;
  leftPWM = (int) tempPWM * tempPWMsign;

  tempPWM = (float) abs(rightPWM) / 255.0;
  tempPWMsign = rightPWM / abs(rightPWM);
  tempPWM = pow(tempPWM, 0.2);
  tempPWM = 255.0 * tempPWM;
  rightPWM = (int) tempPWM * tempPWMsign;

  if (erreurAngle < 10 && erreurAngle>-10){
    compteurZeroAngle++;
  }

  if(erreurDistance <10 && erreurDistance>-10){
    compteurZeroDistance ++ ;
  }

  if (checkSeq  && compteurZeroAngle > 100) {
    distanceTarget = distanceTarget2;
    compteurZeroDistance=0;
    compteurZeroAngle=0;
    checkSeq=false;
    setMotor(LEFT,2,0);
    setMotor(RIGHT,2,0);
    delay(1000);
    
  }

  

  if(erreurDistance <10 && erreurDistance>-10 && compteurZeroDistance >100 && checkSeq == false){
    hasArrived = true;
  }
}

void AsservSoft()
{
  
  if(temps*coefVitesse>distanceTarget && distanceTarget>=0){

  erreurDistance = distanceTarget - (leftClicks + rightClicks) / 2;
  
  }

  if(temps*coefVitesse<=distanceTarget && distanceTarget>=0){
    erreurDistance= (long) temps*coefVitesse - (leftClicks + rightClicks) / 2;
    
  }

  if(!checkSeq || (temps*coefVitesseR>angleTarget && angleTarget>=0)){
    
    erreurAngle = angleTarget - (rightClicks - leftClicks);
  }

  if(checkSeq && temps*coefVitesseR<=angleTarget && angleTarget >=0){
    erreurAngle = temps*coefVitesseR - (rightClicks - leftClicks);
  }

  if(-temps*coefVitesse<distanceTarget && distanceTarget<0){

  erreurDistance = distanceTarget - (leftClicks + rightClicks) / 2;
  
  }

  if(-temps*coefVitesse>distanceTarget && distanceTarget<0){
    erreurDistance= (long) - temps*coefVitesse - (leftClicks + rightClicks) / 2;
    
  }

  if(!checkSeq || (-temps*coefVitesseR<angleTarget && angleTarget<0)){
    
    erreurAngle = angleTarget - (rightClicks - leftClicks);
  }

  if(checkSeq && -temps*coefVitesseR>angleTarget && angleTarget <0){
    erreurAngle = -temps*coefVitesseR - (rightClicks - leftClicks);
  }


  

  
  

  distanceDer = erreurDistance - lastErreurDistance;
  angleDer = erreurAngle - lastErreurAngle;

  distanceInt += erreurDistance;
  angleInt += erreurAngle;

  lastErreurDistance = erreurDistance;
  lastErreurAngle = erreurAngle;

  leftPWM = (kPcoord * (erreurDistance - erreurAngle) + kDcoord * (distanceDer - angleDer) + kIcoord * (distanceInt - angleInt) ) / 1024;
  rightPWM = (kPcoord * (erreurDistance + erreurAngle) + kDcoord * (distanceDer + angleDer) + kIcoord * (distanceInt + angleInt) ) / 1024;

  //compensating the non-linear dependency speed = f(PWM_Value)
  tempPWM = (float) abs(leftPWM) / 255.0;
  tempPWMsign = leftPWM / abs(leftPWM);
  tempPWM = pow(tempPWM, 0.2);
  tempPWM = 255.0 * tempPWM;
  leftPWM = (int) tempPWM * tempPWMsign;

  tempPWM = (float) abs(rightPWM) / 255.0;
  tempPWMsign = rightPWM / abs(rightPWM);
  tempPWM = pow(tempPWM, 0.2);
  tempPWM = 255.0 * tempPWM;
  rightPWM = (int) tempPWM * tempPWMsign;

  if (erreurAngle < 10 && erreurAngle>-10 &&( (angleTarget>0 && temps*coefVitesseR>angleTarget) || (angleTarget<0 && -temps*coefVitesseR<angleTarget))){
    compteurZeroAngle++;
  }

  if(erreurDistance <10 && erreurDistance>-10){
    compteurZeroDistance ++ ;
  }

  if (checkSeq  && compteurZeroAngle > 1000) {
    distanceTarget = distanceTarget2;
    temps=0;
    compteurZeroDistance=0;
    compteurZeroAngle=0;
    checkSeq=false;
    setMotor(LEFT,2,0);
    setMotor(RIGHT,2,0);
    
    
  }

  

  if(erreurDistance <10 && erreurDistance>-10 && compteurZeroDistance >100 && checkSeq == false){
    hasArrived = true;
  }

  temps++;
}

void aller(long distance, long angle) {
  temps = 0;
  leftClicks = 0;
  rightClicks = 0; // réinitialiser les compteurs

  distanceTarget2 = distance;
  angleTarget = angle;
  tempsInitial = millis(); 

  setParameters(distanceTarget2, angleTarget);
}

void initializeZero(){
  compteurZeroDistance=0;
  compteurZeroAngle=0;
}



bool has_Arrived(){

  
  if(erreurDistance <10 && erreurDistance>-10 && erreurAngle < 10  && erreurAngle>-10 && compteurZeroAngle > 100 && compteurZeroDistance >100){
    hasArrived = true;
  }
}
/*
  void setPwmFreq(int pin, int divisor) // function used to override the PWM frequency setting used by default by Arduino
  {
   byte mode;
   if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
   {
     switch (divisor)
     {
       case 1: mode = 0x01; break;
       case 8: mode = 0x02; break;
       case 64: mode = 0x03; break;
       case 256: mode = 0x04; break;
       case 1024: mode = 0x05; break;
       default: return;
     }

     if (pin == 5 || pin == 6) TCCR0B = TCCR0B & 0b11111000 | mode;
     else TCCR1B = TCCR1B & 0b11111000 | mode;
   }
   else if (pin == 3 || pin == 11)
   {
     switch (divisor)
     {
       case 1: mode = 0x01; break;
       case 8: mode = 0x02; break;
       case 32: mode = 0x03; break;
       case 64: mode = 0x04; break;
       case 128: mode = 0x05; break;
       case 256: mode = 0x06; break;
       case 1024: mode = 0x7; break;
       default: return;
     }
     TCCR2B = TCCR2B & 0b11111000 | mode;
   }
  }
*/

void sendData()
{

  Serial.write(replyCommand);
  Serial.write(replyArgCount);
  for (i = 0; i < replyArgCount; i++) Serial.write(replyArg[i]);

}

void doSerial() // UART processing function
{

  if (Serial.available() >= 4)
  {
    //detachInterrupt(Left_INT);
    //detachInterrupt(Right_INT);

    digitalWrite(Status_Red, HIGH);

    cardCommand = Serial.read();
    cardArgCount = Serial.read();

    while (Serial.available() < cardArgCount) {};

    for (i = 0; i < cardArgCount; i++)
    {
      cardArg[i] = Serial.read();
    }

    switch (cardCommand)
    {
      case 6: // set new detination in Position (distance,angle) in mm and degree.
        {
          leftClicks = 0;
          rightClicks = 0; // réinitialiser les compteurs

          distanceTarget2 = 256 * (long)cardArg[1] + (long)cardArg[0];
          distanceTarget2 = distanceTarget2 - 32768;

          angleTarget = 256 * (long)cardArg[3] + (long)cardArg[2];
          angleTarget = angleTarget - 32768;

          aller(distanceTarget2, angleTarget);
        }

      case 7: // same as 6 but sequentially
        {
          leftClicks = 0;
          rightClicks = 0; // réinitialiser les compteurs

          distanceTarget2 = 256 * (long)cardArg[1] + (long)cardArg[0];
          distanceTarget2 = distanceTarget2 - 32768;

          angleTarget = 256 * (long)cardArg[3] + (long)cardArg[2];
          angleTarget = angleTarget - 32768;

          aller(distanceTarget2, angleTarget);

        }

      case 4: // set new destinations in CLICKS :: SetNewTarget() [4 args, 2 vars]
        {
          newLeftTarget = 256 * (long)cardArg[1] + (long)cardArg[0];
          newRightTarget = 256 * (long)cardArg[3] + (long)cardArg[2];

          leftTarget = newLeftTarget - 32768;
          rightTarget = newRightTarget - 32768;
          break;
        }

      case 2: // reset destination (destination = current position) :: ResetTarget() [0 args, 0 vars]
        {
          leftTarget = leftClicks;
          rightTarget = rightClicks;
          break;
        }

      case 999: //reset destination for case 6
        {
          angleTarget = 0;
          leftClicks = 0;
          distanceTarget = 0;
          rightClicks = 0;


        case 131: // set the maximum speed in clicks/second :: SetMaxSpeedC() [2 args, 1 var]
          {
            maxSpeedC = 256 * (int)cardArg[1] + (int)cardArg[0];
            break;
          }

        case 132: // set the maximum acceleration in clicks/second^2 :: SetMaxAccC() [2 args, 1 var]
          {
            maxAccC = 256 * (int)cardArg[1] + (int)cardArg[0];
            break;
          }

        case 133: // set the track width (the distance between the left and right wheels) in milimeters :: SetTrackWidth() [2 args, 1 var]
          {
            trackWidth = 256 * (int)cardArg[1] + (int)cardArg[0];
            break;
          }

        case 134: // set the number of clicks per revolution :: SetCPR() [2 args, 1 var]
          {
            cpr = 256 * (int)cardArg[1] + (int)cardArg[0];
            break;
          }

        case 135: // set the wheel diameter in millimeters :: SetWheelDiameter() [3 args, 2 var]
          {
            if (cardArg[0] == 0) leftWheelDiameter = 256 * (int)cardArg[2] + (int)cardArg[1];
            else rightWheelDiameter = 256 * (int)cardArg[2] + (int)cardArg[1];
            break;
          }

        case 1: // go straight forward/backward for a distance expressed in millimeters :: GoStraigth() [4 args, 2 var] -- old [2 args, 1 var]
          {
            /* OLD FUNCTION - 2 args, 1 var
              deltaForward = 256 * (long)cardArg[1] + (long)cardArg[0];
              deltaForward = deltaForward - 32768;

              tempDelta = (float) deltaForward;
              tempDelta = tempDelta * (float) cpr / ((float) M_PI * (float) wheelDiameter);
              deltaForward = (long) tempDelta;

              leftTarget = leftClicks + deltaForward;
              rightTarget = rightClicks + deltaForward;
              break;
            */
            deltaForward = 256 * (long)cardArg[1] + (long)cardArg[0];
            deltaForward = deltaForward - 32768;

            tempDelta = (float) deltaForward;
            tempDelta = tempDelta * (float) cpr / ((float) m_pi * (float) leftWheelDiameter);
            deltaForwardLeft = (long) tempDelta;

            tempDelta = (float) deltaForward;
            tempDelta = tempDelta * (float) cpr / ((float) m_pi * (float) rightWheelDiameter);
            deltaForwardRight = (long) tempDelta;

            leftTarget = leftClicks + deltaForwardLeft;
            rightTarget = rightClicks + deltaForwardRight;

            leftTargetSpeed = 256 * (long)cardArg[3] + (long)cardArg[2];
            leftTargetSpeed = leftTargetSpeed - 32768;
            rightTargetSpeed = leftTargetSpeed;

            leftTargetSpeed = leftTargetSpeed * (long) cpr;
            leftTargetSpeed = leftTargetSpeed * (long) motorWheelDiameter;
            leftTargetSpeed = leftTargetSpeed / 60000;
            leftTargetSpeed = leftTargetSpeed / (long) leftWheelDiameter;

            rightTargetSpeed = rightTargetSpeed * (long) cpr;
            rightTargetSpeed = rightTargetSpeed * (long) motorWheelDiameter;
            rightTargetSpeed = rightTargetSpeed / 60000;
            rightTargetSpeed = rightTargetSpeed / (long) rightWheelDiameter;

            leftBigPWM = 0;
            rightBigPWM = 0;

            maxLeftPWM = 128 + 36 * abs(leftTargetSpeed + rightTargetSpeed) / 40;
            maxRightPWM = maxLeftPWM;

            if (abs(deltaForwardLeft + deltaForwardRight) > 20000) errorThresholdSlow = (long) (cpr / 2);
            else errorThresholdSlow = abs(deltaForwardLeft + deltaForwardRight) / 4;

            PIDmode = Speed_PD;
            PIDautoswitch = true;

            break;
          }

        case 3: // turn around an angle expressed in radians (en millièmes de radians exactement):: TurnRad() [4 args, 2 var] -- old [2 args, 1 var]
          {
            /* OLD FUNCTION - 2 args, 1 var
              intDeltaAngle = 256 * (long)cardArg[1] + (long)cardArg[0];
              intDeltaAngle = intDeltaAngle - 32768;

              deltaAngle = (float) intDeltaAngle / (float) 1000;

              deltaAngle = deltaAngle * (float) trackWidth * (float) cpr;
              deltaAngle = deltaAngle / ((float) 2 * m_pi * (float) wheelDiameter);

              deltaForward = (long) deltaAngle;

              leftTarget = leftClicks - deltaForward;
              rightTarget = rightClicks + deltaForward;
            */

            intDeltaAngle = 256 * (long)cardArg[1] + (long)cardArg[0];
            intDeltaAngle = intDeltaAngle - 32768;

            deltaAngle = (float) intDeltaAngle / (float) 1000;

            deltaAngleLeft = deltaAngle * (float) trackWidth * (float) cpr;
            deltaAngleLeft = deltaAngleLeft / ((float) 2 * m_pi * (float) leftWheelDiameter);

            deltaAngleRight = deltaAngle * (float) trackWidth * (float) cpr;
            deltaAngleRight = deltaAngleRight / ((float) 2 * m_pi * (float) leftWheelDiameter);

            deltaForwardLeft = (long) deltaAngleLeft;
            deltaForwardRight = (long) deltaAngleRight;

            leftTarget = leftClicks - deltaForwardLeft;
            rightTarget = rightClicks + deltaForwardRight;

            leftTargetSpeed = 256 * (long)cardArg[3] + (long)cardArg[2];
            leftTargetSpeed = leftTargetSpeed - 32768;

            leftTargetSpeed = leftTargetSpeed * (long) cpr;
            leftTargetSpeed = leftTargetSpeed * (long) motorWheelDiameter;
            leftTargetSpeed = leftTargetSpeed / 60000;
            leftTargetSpeed = leftTargetSpeed / (long) wheelDiameter;

            if (deltaForwardLeft + deltaForwardRight >= 0)
            {
              leftTargetSpeed = -leftTargetSpeed;
            }
            else rightTargetSpeed = -rightTargetSpeed;

            leftBigPWM = 0;
            rightBigPWM = 0;

            maxLeftPWM = 128 + 36 * (abs(leftTargetSpeed) + abs(rightTargetSpeed)) / 40;
            maxRightPWM = maxLeftPWM;

            if (abs(deltaForward) > 10000) errorThresholdSlow = (long) (cpr / 2);
            else errorThresholdSlow = abs(deltaForward) / 2;

            PIDmode = Coord_PD; //modifié par Raphaël
            PIDautoswitch = true;

            break;
          }

        case 138: // set the maximum PWM parameter (0 - 255) :: SetMaxPWM() [1 arg, 1 var]
          {
            maxLeftPWM = (int) cardArg[0];
            maxRightPWM = (int) cardArg[0];
            break;
          }

        case 139: // compute and send back the current angle of the robot :: GetAngleRad() [RETURN: 2 args, 1 var]
          {
            /*deltaClicks = rightClicks - leftClicks;
              tempAngle = (float) deltaClicks * (float) m_pi * (float) wheelDiameter;
              tempAngle = tempAngle / (float) cpr;
              tempAngle = tempAngle / (float) trackWidth;*/

            tempAngleLeft = (float) leftClicks * (float) m_pi * (float) wheelDiameter;
            tempAngleLeft = tempAngleLeft / (float) cpr;
            tempAngleLeft = tempAngleLeft / (float) trackWidth;

            tempAngleRight = (float) rightClicks * (float) m_pi * (float) wheelDiameter;
            tempAngleRight = tempAngleRight / (float) cpr;
            tempAngleRight = tempAngleRight / (float) trackWidth;

            tempAngle = tempAngleRight - tempAngleLeft;

            while (tempAngle < -m_pi) tempAngle = tempAngle + 2 * m_pi;
            while (tempAngle > m_pi) tempAngle = tempAngle - 2 * m_pi;

            tempAngle = tempAngle * 1000.0;
            intAngle = (long) tempAngle;
            intAngle = intAngle + 32768;

            replyCommand = 139;
            replyArgCount = 2;
            replyArg[1] = (unsigned char) (intAngle / 256);
            replyArg[0] = (unsigned char) (intAngle - 256 * (long) replyArg[1]);

            sendData();
            break;
          }

        case 140: // compute and send back the coordinates of the robot (x,y,angle) :: GetCurrentCoord() [RETURN: 6 args, 3 var]
          {
            /*deltaClicks = rightClicks - leftClicks;
              tempAngle = (float) deltaClicks * (float) m_pi * (float) wheelDiameter;
              tempAngle = tempAngle / (float) cpr;
              tempAngle = tempAngle / (float) trackWidth;*/

            tempAngleLeft = (float) leftClicks * (float) m_pi * (float) wheelDiameter;
            tempAngleLeft = tempAngleLeft / (float) cpr;
            tempAngleLeft = tempAngleLeft / (float) trackWidth;

            tempAngleRight = (float) rightClicks * (float) m_pi * (float) wheelDiameter;
            tempAngleRight = tempAngleRight / (float) cpr;
            tempAngleRight = tempAngleRight / (float) trackWidth;

            tempAngle = tempAngleRight - tempAngleLeft;

            while (tempAngle < -m_pi) tempAngle = tempAngle + 2 * m_pi;
            while (tempAngle > m_pi) tempAngle = tempAngle - 2 * m_pi;

            tempAngle = tempAngle * 1000.0;
            intAngle = (long) tempAngle;
            intAngle = intAngle + 32768;

            x = xClicks * (float) m_pi * (float) leftWheelDiameter / (float) cpr;
            y = yClicks * (float) m_pi * (float) rightWheelDiameter / (float) cpr;

            xInt = (long) x + 32768;
            yInt = (long) y + 32768;

            replyCommand = 140;
            replyArgCount = 6;
            replyArg[1] = (unsigned char) (intAngle / 256);
            replyArg[0] = (unsigned char) (intAngle - 256 * (long) replyArg[1]);
            replyArg[3] = (unsigned char) (xInt / 256);
            replyArg[2] = (unsigned char) (xInt - 256 * (long) replyArg[3]);
            replyArg[5] = (unsigned char) (yInt / 256);
            replyArg[4] = (unsigned char) (yInt - 256 * (long) replyArg[5]);

            sendData();
            break;
          }

        case 141: // set the target speed :: SetSpeed() [4 args, 2 var]
          {
            leftTargetSpeed = 256 * (long) cardArg[1] + (long) cardArg[0];
            rightTargetSpeed = 256 * (long) cardArg[3] + (long) cardArg[2];
            leftTargetSpeed = leftTargetSpeed - 32768;
            rightTargetSpeed = rightTargetSpeed - 32768;
            leftTargetSpeed = leftTargetSpeed * (long) cpr;
            rightTargetSpeed = rightTargetSpeed * (long) cpr;
            leftTargetSpeed = leftTargetSpeed * (long) motorWheelDiameter;
            rightTargetSpeed = rightTargetSpeed * (long) motorWheelDiameter;
            leftTargetSpeed = leftTargetSpeed / 60000;
            rightTargetSpeed = rightTargetSpeed / 60000;
            leftTargetSpeed = leftTargetSpeed / (long) wheelDiameter;
            rightTargetSpeed = rightTargetSpeed / (long) wheelDiameter;
            leftBigPWM = 0;
            rightBigPWM = 0;
            break;
          }

        case 142: // set working mode :: SetWorkingMode() [1 arg, 1 var]
          {
            PIDmode = (int) (cardArg[0] - 128);
            break;
          }

        case 5: // find out if the robot has arrived to its destination :: HasArrived() [RETURN: 1 arg, 1 var]
          {
            replyCommand = 5;
            replyArgCount = 1;

            if (hasArrived == true) replyArg[0] = 1; // modifié par Raphaël
            else replyArg[0] = 0; // modifié par Raphaël

            sendData();

            break;
          }

        }

        //attachInterrupt(Left_INT, leftUpdate, CHANGE);
        //attachInterrupt(Right_INT, rightUpdate, CHANGE);
        digitalWrite(Status_Red, LOW);
    }
  }
}
// ##### The program's main loop. #####
void loop()

{ PIDmode = New_Coord_PD ;
  
  //**************************************************************************************//
  //*******************computing current coordinates (first 40 lines)*********************//
  //*****goal: to calculate x and y (in clicks) and the error relative to the targets*****//
  //**************************************************************************************//

  /*deltaClicks = rightClicks - leftClicks;
    angle = (float) deltaClicks * (float) m_pi * (float) wheelDiameter;
    angle = angle / (float) cpr;
    angle = angle / (float) trackWidth;*/ //Above we have a second way to mesure the angle
  //************************Angle mesuring**************************************//
  tempAngleLeft = (float) compteur * (float) m_pi * (float) wheelDiameter;
  tempAngleLeft = tempAngleLeft / (float) cpr;
  tempAngleLeft = tempAngleLeft / (float) trackWidth;

  tempAngleRight = (float) rightClicks * (float) m_pi * (float) wheelDiameter;
  tempAngleRight = tempAngleRight / (float) cpr;
  tempAngleRight = tempAngleRight / (float) trackWidth;

  angle = tempAngleRight - tempAngleLeft;

  //*******************fitting angle in [-pi,pi]******************************//

  while (angle < -m_pi) angle = angle + 2 * m_pi;
  while (angle > m_pi) angle = angle - 2 * m_pi;

  //*******A little filter to decrease "noise angles" (take the average)********//
  avgAngle = (angle + lastAngle) / 2.0;
  lastAngle = angle;

  //***********calculating delta clicks, since last loop iteration************//
  deltaLeftClicks = leftClicks - lastLeftClicks;
  deltaRightClicks = rightClicks - lastRightClicks;
  lastLeftClicks = leftClicks;
  lastRightClicks = rightClicks;

  deltaAvgClicks = (float) (deltaLeftClicks + ((float) (deltaRightClicks) * (float) (rightWheelDiameter) / (float) (rightWheelDiameter)) / 2.0);

  //******************calculating x and y, in Clicks units******************//
  xClicks = xClicks + deltaAvgClicks * cos(avgAngle); //this is used only to return the value to the brain. other than that, it doesn't change how the loop works
  yClicks = yClicks + deltaAvgClicks * sin(avgAngle);

  // computing the errors for each motor
  leftError = abs(leftTarget - leftClicks);
  rightError = abs(rightTarget - rightClicks);

  //****************************************************************************//
  //***********************finished here the calculations***********************//
  //****************************************************************************//

  //*********************** checking if we have arrived ************************//
  if ((leftError < errorThresholdStop) && (rightError < errorThresholdStop)) hasArrived = true;
  else hasArrived = false;//this is used only to return the value to the brain. other than that, it doesn't change how the loop works

  //********************* implementing the PID autoswitch***********************//
  if (PIDautoswitch == true)
  {
    //if too too close close, switch to coordinate mode and limit PWMmax
    if ((leftError < errorThresholdStop) && (rightError < errorThresholdStop))
    {
      PIDmode = Coord_PD;
      maxLeftPWM = 180;
      maxRightPWM = 180;
      PIDautoswitch = false;
    }
    //if too close switch to coordinate mode
    else if ((leftError < errorThresholdSlow) && (rightError < errorThresholdSlow))
    {
      PIDmode = Coord_PD;
      if (lastPWM >= 128)
      {
        maxLeftPWM = lastPWM;
        maxRightPWM = lastPWM;
      }
      else
      {
        maxLeftPWM = 128;
        maxRightPWM = 128;
      }
    }
  }


  //############################################################################//
  //############################## PID computing ###############################//
  //############################################################################//
  switch (PIDmode)
  {
    case New_Coord_PD:
      {
        AsservSoft();

        break;
      }

    case Coord_PD: // PD algorithm, target COORDINATES
      {
        // computing the errors for each motor
        leftError = leftTarget - leftClicks;
        rightError = rightTarget - rightClicks;


        //Serial.print("leftError: ");
        //Serial.print(leftError);

        // computing the derivatives for each motor
        leftDer = leftError - lastLeftError;
        rightDer = rightError - lastRightError;

        //Serial.print("  leftDer: ");
        //Serial.print(leftDer);

        // updating the last error
        lastLeftError = leftError;
        lastRightError = rightError;

        // actual PID calculus
        leftPWM = (kPcoord * leftError + kDcoord * leftDer) / 1024;
        rightPWM = (kPcoord * rightError + kDcoord * rightDer) / 1024;

        //Serial.print("  leftPWM: ");
        //Serial.print(leftPWM);

        // compensating the non-linear dependency speed = f(PWM_Value)
        tempPWM = (float) abs(leftPWM) / 255.0;
        tempPWMsign = leftPWM / abs(leftPWM);
        tempPWM = pow(tempPWM, 0.2);
        tempPWM = 255.0 * tempPWM;
        leftPWM = (int) tempPWM * tempPWMsign;

        tempPWM = (float) abs(rightPWM) / 255.0;
        tempPWMsign = rightPWM / abs(rightPWM);
        tempPWM = pow(tempPWM, 0.2);
        tempPWM = 255.0 * tempPWM;
        rightPWM = (int) tempPWM * tempPWMsign;


        //Serial.print("  leftPWM: ");
        ///Serial.println(leftPWM);

        //Serial.println();



        break;
      }
    case Coord_PID: // PID algorithm, target COORDINATES
      {
        break;
      }
    case Speed_PD: // PD algorithm, target SPEED
      {
        // computing the time interval
        currentTime = micros();//*********************it is 64 times higher****************//
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        deltaTime = deltaTime / 64;// ***********which is fixed here*************//

        // computing the current speed in clicks / millisecond
        leftSpeed = deltaLeftClicks * 1000 / (long) deltaTime;
        rightSpeed = deltaRightClicks * 1000 / (long) deltaTime;

        // computing the speed error
        leftSpeedError = leftTargetSpeed - leftSpeed;
        rightSpeedError = rightTargetSpeed - rightSpeed;

        // computing the speed error derivative
        leftSpeedDer = leftSpeedError - lastLeftSpeedError;
        rightSpeedDer = rightSpeedError - lastRightSpeedError;

        // updating the last error value
        lastLeftSpeedError = leftSpeedError;
        lastRightSpeedError = rightSpeedError;

        // computing the errors for each motor -- useful for the autoswitch
        lastLeftError = leftTarget - leftClicks;
        lastRightError = rightTarget - rightClicks;

        // actual PID calculus
        leftBigPWM = leftBigPWM + (kPspeed * leftSpeedError + kDspeed * leftSpeedDer) / 16;
        rightBigPWM = rightBigPWM + (kPspeed * rightSpeedError + kDspeed * rightSpeedDer) / 16;
        leftPWM = leftBigPWM / 64;
        rightPWM = rightBigPWM / 64;

        lastPWM = (abs(leftPWM) + abs(rightPWM)) / 2;

        break;
      }
    case Speed_PID: // PID algorithm, target SPEED -- NOT PROPERLY TUNED, might abandon
      {
        /*
          // computing the time interval
          currentTime = micros();
          deltaTime = currentTime - lastTime;
          lastTime = currentTime;
          deltaTime = deltaTime / 64;

          // computing the current speed in clicks / millisecond
          leftSpeed = deltaLeftClicks * 1000 / (long) deltaTime;
          rightSpeed = deltaRightClicks * 1000 / (long) deltaTime;

          // computing the speed error
          leftSpeedError = leftTargetSpeed - leftSpeed;
          rightSpeedError = rightTargetSpeed - rightSpeed;

          // computing the speed error derivative
          leftSpeedDer = leftSpeedError - lastLeftSpeedError;
          rightSpeedDer = rightSpeedError - lastRightSpeedError;

          // updating the last error value
          lastLeftSpeedError = leftSpeedError;
          lastRightSpeedError = rightSpeedError;

          // limiting the error that will be added to the integral error
          if ((leftSpeedError > -maxIntError) && (leftSpeedError < maxIntError)) leftSpeedInt = leftSpeedInt + leftSpeedError;
          if ((rightSpeedError > -maxIntError) && (rightSpeedError < maxIntError)) rightSpeedInt = rightSpeedInt + rightSpeedError;

          // limiting the integral error
          if (leftSpeedInt > maxSpeedInt) leftSpeedInt = maxSpeedInt;
          else if (leftSpeedInt < - maxSpeedInt) leftSpeedInt = - maxSpeedInt;
          if (rightSpeedInt > maxSpeedInt) rightSpeedInt = maxSpeedInt;
          else if (rightSpeedInt < - maxSpeedInt) rightSpeedInt = -maxSpeedInt;

          // actual PID calculus
          leftBigPWM = leftBigPWM + (kPspeed * leftSpeedError + kDspeed * leftSpeedDer + kIspeed * leftSpeedInt) / 16;
          rightBigPWM = rightBigPWM + (kPspeed * rightSpeedError + kDspeed * rightSpeedDer + kIspeed * rightSpeedInt) / 16;
          leftPWM = leftBigPWM / 64;
          rightPWM = rightBigPWM / 64;
        */
        break;
      }
  }
  //At this point the PWM value is already defined. Now it suffices to tell the motors what to do

  // speed limiting (in order to avoid PWM values higher than the maximally set values)
  if (leftPWM < -maxLeftPWM) leftPWM = -maxLeftPWM;
  if (leftPWM > maxLeftPWM) leftPWM = maxLeftPWM;
  if (rightPWM < -maxRightPWM) rightPWM = -maxRightPWM;
  if (rightPWM > maxRightPWM) rightPWM = maxRightPWM;

  // setting the speed and direction parameters for the motors
  if (leftPWM < 0) setMotor(LEFT, REVERSE, -leftPWM);
  else setMotor(LEFT, FORWARD, leftPWM);
  if (rightPWM < 0) setMotor(RIGHT, REVERSE, -rightPWM);
  else setMotor(RIGHT, FORWARD, rightPWM);

  // checking for serial input
  doSerial();

  // and, of course, blinking
  if (statusBlue == false) {
    statusBlue = true;
    digitalWrite(Status_Blue, HIGH);
  }
  else {
    statusBlue = false;
    digitalWrite(Status_Blue, LOW);
  }
}
