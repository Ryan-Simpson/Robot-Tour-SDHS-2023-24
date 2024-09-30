// Include the (Motor) library
#include <L298NX2.h>
#include <Vector.h>

// Left Motor
const unsigned int EN_A = 10;   
const unsigned int IN1_A = 22;
const unsigned int IN2_A = 23;

// Right Motor
const unsigned int EN_B = 11;   
const unsigned int IN1_B = 24;
const unsigned int IN2_B = 25;

//Button Pin Code
const int buttonPin = 40;                 
int buttonState = 0;                     // variable for reading the pushbutton status
int lastbuttonState = 0;                 // variable for reading the last pushbutton status

//Encoder Pin Code
const byte encoderLpinA = 2;            //A pin -> the interrupt pin 0
const byte encoderLpinB = 26;           //B pin -> the digital pin 3
byte encoderLPinALast;
long encLpulses;                        //the number of the pulses
boolean encLdir;                        //the rotation direction

// Right Motor Encoder
const byte encoderRpinA = 3;            //A pin -> the interrupt pin 0
const byte encoderRpinB = 28;           //B pin -> the digital pin 3
byte encoderRPinALast;
long encRpulses;                        //the number of the pulses
boolean encRdir;                        //the rotation direction

const int encLend = 3*1920;                 // pulses for left motor
int Run_Motors_Forward = 0;                 // If set to 1 the motors will drive forward for interval
int Im_Moving_forward = 0;                  // If i'm actually moving forward
const long print_time = 200;                // Time in ms for printing updates to the console 
unsigned long previousMillis = millis();    // will store last time Motor was run
String last_command = "No Command Sent";    // Place holder for last command


//movement variavbles 
const double travelDist = 100;           // travel distance in CM  
const double ENCperCMS = 93.7;                //Number of encoder counts per cm 
const double ENCperCMB = -93.7;                //Number of encoder counts per cm 
int motor_A_speed = 50;                // motor A speed (left Motor)
int motor_B_speed = 50;                // motor B speed (right Motor)
int run_forward_cnt = 0;               // howmany times we have run forward

//*** CHANGE THESE VARIABLES ***'
//target time in seconds
int targetTime = 58;

//facing: 1North 2East 3South 4West
int currentFacing = 1;
// You always face north to start...

//input path for robot, this is the path we want the robot to go
// remeber to set pathing equal to a path and length in the setup

int allRounder[] = {21, 22, 23, 24, 23, 33, 34, 44, 43, 42, 32, 22, 21};
int allRounderLength = 13;

int backwardsTest[] = {21, 22, 23, 24, 23, 22, 21};
int backwardsLength = 7;

int weirdTest[] = {21, 22, 12, 13, 14, 24, 23, 33, 32};
int weirdLength = 9;

int farTest[] = {21, 31, 41, 31, 21, 22, 12, 13, 12, 11, 12, 13, 14, 24, 23, 33, 32};
int farLength = 17;

int closeTest[] = {21, 22, 32};
int closeLength = 3;

/* Course Grid

  {14 24 34 44}
  {13 23 33 43}
  {12 22 32 42}
  {11 21 31 41}
*/

//length for the path array


// variables
const int pushButton = 40;
int numOfF, numOfT;
String currentMove = "nothing";

//vectors for output path
char movementArray[500];
int numArray[500];
Vector<char> movementType(movementArray);
Vector<int> numMovement(numArray);
int vectorIndex = -1;
char preMove;

//Data arrays
//arrays for speed from distance
double cmOverSeconds[] = {5.50, 3.38};
int speedDistance[] = {70, 250};




// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

//Begining 
  void setup() 
{
  // Run pathing to parse course path
  // Chosse a path and a path length
  pathing(farTest, farLength);
  
  // Used to display information
  Serial.begin(115200);
  // Wait for Serial Monitor to be opened
  while (!Serial)
  {
    //do nothing
  }
  pinMode(buttonPin, INPUT);
  encLpulses = 0;
  encRpulses = 0;
  EncodersInit();
}


  void loop() 
{
  printSomeInfo();
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  // check if the  pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH && lastbuttonState == LOW) 
  {
    delay(1000);
    executePathing(numMovement, movementType);  
    //Run_Motors_Forward = 1;
  }  
    lastbuttonState = buttonState;

  // Run Motors Forward
  // If run motors forward is true -> start checking time
  // Once we have run for set time -> stop motors and reint varibles
  /*if (Run_Motors_Forward == 1 && encLpulses < encLend) 
  {
    if (Im_Moving_forward == 0) 
    {
      // Start Moving Make robot drive forward 
      //moveStraight(int speedLeftWheel, int speedRightWheel, long encoderLeftCount, long encoderRightCount, bool straightOrBack )
      //true == Straight and false == backwards    
      Im_Moving_forward = 1;
    }
    if (Im_Moving_forward == 1) 
    {
      // Start Moving Make robot drive forward 
      motors.stop();
      Im_Moving_forward = 0;
      Run_Motors_Forward = 0;
    }
    // Stop moving and Clean up
   }
  //SpeedMatch();
  */
  printSomeInfo();
}



//Setting Speeds for the turning movments 


//Setting speed for Right movement
  void speedRight()
{
  motors.stop();
  motors.setSpeedA(90);
  motors.setSpeedB(90);
}



//Moving Foward
//Set speeds and Set encoder counts 
//Are we going fowards or backwards?
//If true move fowards If false move backwards
  void moveStraight(int speedLeftWheel, double distance, bool forwardOrBack)
{
  //make sure we are stopped then zero encoders
  motors.stop();
  delay(200);
  encLpulses = 0;
  encRpulses = 0;

  // Calc End pulses
  const long encEndStraight = distance * ENCperCMS ;
  const long encEndBack = distance * ENCperCMB ;

  motors.setSpeed(speedLeftWheel);
  if(forwardOrBack == true)
  {
    motors.forward();
    while(encLpulses < encEndStraight) 
    {
      SpeedMatch();
      printSomeInfo();
    }
    hitBreakStraight(speedLeftWheel);

  } else {

    motors.backward();
    while(encLpulses > encEndBack) 
    {
      SpeedMatchReverse();
      printSomeInfo();
    }

     motors.stop();
     hitBreakBackwards();
  }
}

//Moving Left
  void moveLeft(){
  motors.stop();
  delay(200); //lets make sure we are not moving before we zero out
  encLpulses = 0;
  encRpulses = 0;
  motors.setSpeed(80);
  motors.backwardA();
  motors.forwardB();
    while(encRpulses < 960) 
  {
    printSomeInfo();
  }
motors.stop();
hitBreakLeft();
}

//Moving Right
void moveRight() {
  motors.stop();
  delay(200); //lets make sure we are not moving before we zero out
  encLpulses = 0;
  encRpulses = 0;
  motors.setSpeed(80);
  motors.backwardB();
  motors.forwardA();
  while(encRpulses > -960) 
  {
    printSomeInfo();
  }
  motors.stop();
  hitBreakRight();

}

void Stop_Slow_Right(){
  //Stop the right motor and slow it down the next time you turn it on
  int tmp_speedB = motors.getSpeedB();
  motors.stopB();
  motors.setSpeedB(tmp_speedB - 1); // slow down the right motor since it was to speedy
}

void Stop_Slow_Left(){
  //Stop the left motor and slow it down the next time you turn it on
  int tmp_speedA = motors.getSpeedA();
  motors.stopA();
  motors.setSpeedA(tmp_speedA - 1); // slow down the right motor since it was to speedy
}

//Code to match the speed of both motors to prevent one overpowering the other 
void SpeedMatch_Turn(bool turn_right) {
  //This should work to match the speed of the right motor with the speed of the left
  int tmp_speed = 0; // temp holder for speed setting
  if(abs( abs(encLpulses) - abs(encRpulses) ) > 10) {
    // we got ahead of ourselves
    if(abs(encLpulses) > abs(encRpulses)) {
      // left motor has gone further than the right motor
      // stop the left motor
      Stop_Slow_Left();
      while(abs(encLpulses) > abs(encRpulses)){
       // let the right motor catch up
       printSomeInfo();
      }
        // OK right caught up - make left go again
      if(turn_right){
        motors.forwardA();
      } else {
        motors.backwardA();
      }

    } else {
      // Right motor went further than the left
      // Stop the right motor
      
      Stop_Slow_Right();
      
      while(abs(encRpulses) > abs(encLpulses)){
       // let the left motor catch up
       printSomeInfo();
      }
      if(turn_right){
        motors.backwardB();
      } else {
        motors.forwardB();
      }
      
    }
  }
}


void SpeedMatch() {
  //This should work to match the speed of the right motor with the speed of the left

  if(abs(encLpulses-encRpulses)>10) {
    // so the encoders are off a little
    if(encLpulses > encRpulses) {
      // left motor went to far
      
      // Stop left motor
      Stop_Slow_Left();

      while(encLpulses > encRpulses){
        // Wait for right to catch up
        printSomeInfo();
      }
      // Turn left motor back on
      motors.forwardA();
    } else {
      
      // right motor went to far
      Stop_Slow_Right();
      
      while(encRpulses > encLpulses){
        // wait for left to catch up
        printSomeInfo();
      }
      // turn right motor back on
      motors.forwardB(); 
    }
  }
}

void SpeedMatchReverse() {
  //This should work to match the speed of the right motor with the speed of the left
  
  if(abs(encLpulses-encRpulses)>10) {
    // so the encoders are off a little
    if(encLpulses > encRpulses) {
      // Right motor went to far
      // stop the right mnotor
      Stop_Slow_Right();
      while(encLpulses > encRpulses){
        //wait for the left motor to catch up
        printSomeInfo();
      }
      // start right motor again
      motors.backwardB();
    } else {
      // Left Motor went to far
      // stop the left motor
      Stop_Slow_Left();
      while(encRpulses > encLpulses){
        // wait for right motor to catch up
        printSomeInfo();
      }
      // good now, turn on left motor 
      motors.backwardA(); 
    }
  }
}

//Brakes for the car if going foward or turning or baccwards 
void hitBreakStraight(int speedLeftWheel){
  motors.stop();
  motors.setSpeed(150);
  motors.backward();
  delay(100);
  motors.stop();
}
void hitBreakBackwards(){
  motors.stop();
  motors.setSpeed(150);
  motors.forward();
  delay(100);
  motors.stop();
}

void hitBreakLeft(){
  motors.stop();
  motors.setSpeed(100);
  motors.backwardB();
  motors.forwardA();
  delay(100);
  motors.stop();
}
void hitBreakRight(){
  motors.stop();
  motors.setSpeed(100);
  motors.backwardA();
  motors.forwardB();
  delay(100);
  motors.stop();
}


//Encoder counting code
void EncodersInit() {
  encLdir = true;                           //default -> Forward
  encRdir = true;                           //default -> Forward

  pinMode(encoderLpinB,INPUT);
  pinMode(encoderRpinB,INPUT);

  attachInterrupt(0, EncoderLCounter, CHANGE);
  attachInterrupt(1, EncoderRCounter, CHANGE);
}


void EncoderLCounter() {
  int encLLstate = digitalRead(encoderLpinA);
  if((encoderLPinALast == LOW) && encLLstate == HIGH)
  {
    int val = digitalRead(encoderLpinB);
    if(val == LOW && encLdir)
    {
      encLdir = false;                      //Reverse
    }
    else if(val == HIGH && !encLdir)
    {
      encLdir = true;                       //Forward
    }
  }
  encoderLPinALast = encLLstate;

  if(encLdir)  encLpulses++;                // NOTE This is opposite of the right encoder since they are inverted
  else  encLpulses--;
}

void EncoderRCounter() {
  int encRLstate = digitalRead(encoderRpinA);
  if((encoderRPinALast == LOW) && encRLstate==HIGH)
  {
    int val = digitalRead(encoderRpinB);
    if(val == LOW && encRdir)
    {
      encRdir = false;                      //Reverse
    }
    else if(val == HIGH && !encRdir)
    {
      encRdir = true;                       //Forward
    }
  }
  encoderRPinALast = encRLstate;

  if(!encRdir)  encRpulses++;
  else  encRpulses--;
}



//takes in a path array and length for that array and alters two existing vectors that will be used by executePath function to call movement functions
void pathing(int input[], int length) {
    int x1, x2, y1, y2, nextFacing;
    for (int i = 0; i < length - 1; i++) {
        x1 = input[i] / 10;
        y1 = input[i] % 10;
        x2 = input[i + 1] / 10;
        y2 = input[i + 1] % 10;
        nextFacing = correctFacing(x1, x2, y1, y2);
        if (x1 == x2 || y1 == y2) {
            if (i == 0) {
                if (currentFacing == nextFacing) {
                    vectorIndex++;
                    preMove = 'f';
                    movementType.push_back('f');
                    numMovement.push_back(1);
                    numOfF++;
                }
                else {
                    if (currentFacing < nextFacing && changeFacing("right", currentFacing) == nextFacing) {
                        vectorIndex++;
                        preMove = 'r';
                        movementType.push_back('r');
                        numMovement.push_back(1);
                        currentFacing = changeFacing("right", currentFacing);
                        numOfT++;
                    }
                    else {
                        vectorIndex++;
                        preMove = 'l';
                        movementType.push_back('l');
                        numMovement.push_back(1);
                        currentFacing = changeFacing("left", currentFacing);
                        numOfT++;
                    }
                }
            }
            else {
                if (currentFacing == nextFacing) {
                  if(preMove == 'f'){
                    numMovement[vectorIndex]++;
                    numOfF++;
                  }
                  else {
                    vectorIndex++;
                    preMove = 'f';
                    movementType.push_back('f');
                    numMovement.push_back(1);
                    numOfF++;
                  }
                }
                else if (changeFacing("right", currentFacing) != nextFacing && changeFacing("left", currentFacing) != nextFacing) {
                  if(preMove == 'b'){
                    numMovement[vectorIndex]++;
                    numOfF++;
                  }
                  else {
                    vectorIndex++;
                    preMove = 'b';
                    movementType.push_back(preMove);
                    numMovement.push_back(1);
                    numOfF++;
                  }
                }
                else {
                  if (changeFacing("right", currentFacing) == nextFacing) {
                    vectorIndex++;
                    preMove = 'r';
                    movementType.push_back(preMove);
                    numMovement.push_back(1);
                    currentFacing = changeFacing("right", currentFacing);
                    numOfT++;
                  }
                  else {
                    vectorIndex++;
                    preMove = 'l';
                    movementType.push_back(preMove);
                    numMovement.push_back(1);
                    currentFacing = changeFacing("left", currentFacing);
                    numOfT++;
                  }
                  preMove = 'f';
                  vectorIndex++;
                  movementType.push_back(preMove);
                  numMovement.push_back(1);
                  numOfF++;
                }
            }
        }
        else {
            Serial.print("FUDGE ERROR");
            break;
        }
    }
}



//gets the correct facing for the next move
int correctFacing(int currentX, int nextX, int currentY, int nextY) {
  int nFacing;                            //creates a variable for output facing
  if (currentX == nextX) {
    if (currentY < nextY) {
      nFacing = 1;
    }
    else {
      nFacing = 3;
    }
  }
  else {
    if (currentX < nextX) {
      nFacing = 2;
    }
    else {
      nFacing = 4;
    }
  }
  return nFacing;
}



//returns an int depending on input parameters, one being the direction of turn and other being a facing
int changeFacing(String direction, int facing) {
    if (direction == "right") {              //if direction input is right, increases facing output by one unless facing is 4, else changes facing to 1
      if (facing != 4) {
        facing++;
      }
      else {
        facing = 1;
      }
    }
    else {
      if (facing != 1) {                     //if direction input is left, decreases facing output by one unless facing is 1, else changes facing to 4
        facing--;
      }
      else {
        facing = 4;
      }
  }
  return facing;                             //returns facing integer
}



//takes the output vectors from pathing function and calls correct movement functions based on vectors
void executePathing(Vector<int> num, Vector<char> type) {
  long tmp;
  moveStraight(150, 25, true);              //moves forward 25 cm for first move to get to middle of square
    for (int i = 0; i <= vectorIndex; i++) {          //iterates through each index of vector
      tmp = num[i]*50;
      if (type[i] == 'f') {
        currentMove = "forward";                  //calls movement function with numMovement*distance for one moveForward
        last_command = "moveStraight(150," + tmp;
        last_command = last_command + ", true";
        moveStraight(100, num[i]*50, true);
        
      }
      else if (type[i] == 'l') {
        currentMove = "left";
        moveLeft();
        last_command = "moveLeft()";
      }
      else if (type[i] == 'r') {
        currentMove = "right";
        moveRight();
        last_command = "moveRight()";
      }
      else {
        currentMove = "back";
        last_command = tmp;
        moveStraight(100, num[i]*50, false);
        //last_command = last_command + ", false";
      }
      printSomeInfo();
      currentMove = "nothing";
    }
}



//make function to output speed of forward and backward movements // equation: time_for_forward/backward = end_time - (time_for_turn * num_of_turns) / num_of_forwards/backwards
int getSpeed(int target, int forwards, int turns){
  int time = target - turns*1.2;
  int dist = 25 + 50*forwards;


}



//takes an x value, xArray, and yArray for specific graphs and interpolates a y value for the x value
int interpolate(int x, int xArray[], int yArray[]){
  int output, x1, x2, y1, y2; 
  bool check = false;
  int sizeOfArray = sizeof(xArray)/sizeof(int);
  
  for(int i = 0; i < sizeOfArray-2; i++){           //iterates through xArray to find values X is inbetween, if no values are found, uses last value of yArray for output
    if(xArray[i] < x && xArray[i+1] > x){
      x1 = xArray[i];
      x2 = xArray[i+1];
      y1 = yArray[i];
      y2 = yArray[i+1];
      check = true;
      break;
    }
  }
  
  if(check == true){                                //uses interpolation equation to interpolate for y using previously found values from arrays
    output = y1 + ((x-x1)/(x2-x1))*(y2-y1);
  } else {
    output = yArray[sizeOfArray-1];
  }

  return output;  
}



//Printing out all information
void printSomeInfo() {
  if(millis() - previousMillis > print_time)
    {
    // Print motor info in Serial Monitor
      Serial.print("\033[0H\033[0J");       //Clear terminal window
      Serial.print("Last Command = ");
      Serial.println(last_command);
      Serial.print("Motor A direction = ");
      Serial.print(motors.getDirectionA());
      Serial.print(", Moving = ");
      Serial.print(motors.isMovingA() ? "YES" : "NO");
      Serial.print(", Speed = ");
      Serial.println(motors.getSpeedA());
      // Start New Line
      Serial.print("Motor B direction = ");
      Serial.print(motors.getDirectionA());
      Serial.print(", Moving = ");
      Serial.print(motors.isMovingB() ? "YES" : "NO");
      Serial.print(", Speed = ");
      Serial.println(motors.getSpeedB());
      // Start new Line
      Serial.print("Right Encoder Count = ");
      Serial.print(encRpulses);
      Serial.print("     Left Encoder Count = ");
      Serial.println(encLpulses);
      // Start new Line
      //Serial.print(currentMove);
      previousMillis = millis();                  // update the time we last printed
  }
}