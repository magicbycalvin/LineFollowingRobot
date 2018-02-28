
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define DEBUG // Uncomment this for serial debugging
#define DRIVE // Uncomment this to drive the motors

#ifdef DEBUG
  #define dbPrint( x ) Serial.print( x )
  #define dbPrintln( x ) Serial.println( x )
#else
  #define dbPrint( x )
  #define dbPrintln( x )
#endif

#define ROT_GAIN 1 // Rotational gain to multiply the difference of the measured distances to determine how aggresive the turn should be
#define CTRL_PER 1 // Period at which the control system is run [default is 40 Hz, or 25 ms] (ms)
#define DEFAULT_SPEED 100 // Default motor speed [0-255] (DN)
#define ROT_DELAY 250


// Since we don't have a feedback loop, we may need to tune the motor speeds to each other (offsets are in DN [0-255])
#define M1_OFFSET 13 // Back Left
#define M2_OFFSET 0 // Back Right
#define M3_OFFSET 0 // Front Right
#define M4_OFFSET 13 // Front Left

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Create 4 motor objects
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

//define infrared sensor pin
#define leftIF 8
#define rightIF 9

int leftIFVal = 0;
int rightIFVal = 0;

unsigned long curTime;
unsigned long lastTime = 0;

void setup() {
  // put your setup code here, to run once:

  pinMode(leftIF, INPUT);
  pinMode(rightIF, INPUT);

    // Create with the default frequency 1.6KHz
  AFMS.begin();
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor1->setSpeed(DEFAULT_SPEED);
  motor2->setSpeed(DEFAULT_SPEED);
  motor3->setSpeed(DEFAULT_SPEED);
  motor4->setSpeed(DEFAULT_SPEED);

  // Turn all four motors off
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);

  // Delay for a second so that we can set the robot down if it was being programmed
  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  curTime = millis();

  // Run the control system at a known frequency
  if ( curTime - lastTime >= CTRL_PER ) {

    leftIFVal = digitalRead(leftIF);
    rightIFVal = digitalRead(rightIF);

    if(!leftIFVal && !rightIFVal){
  
      dbPrintln("Normal");
      driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, FORWARD );
      
    }else if (leftIFVal && !rightIFVal) {

       dbPrintln("Left");
       //backUp();
       //rotateR();
       driveMotors( DEFAULT_SPEED, BACKWARD, DEFAULT_SPEED, FORWARD );
      
    }else if (!leftIFVal && rightIFVal){

      dbPrintln("Right");
      //backUp();
      //rotateL();
      driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, BACKWARD );

      
    } else {
      driveMotors( 0, FORWARD, 0, FORWARD );
    }
    
  }

}


void driveMotors( uint8_t speedL, uint8_t dirL, uint8_t speedR, uint8_t dirR ) {

  #ifdef DRIVE
  
    // For the robot, motors 1 and 4 are on the left, motors 2 and 3 are on the right
    // Set left speed
    motor1->setSpeed(speedL+M1_OFFSET);
    motor4->setSpeed(speedL+M4_OFFSET);
    // Set right speed
    motor2->setSpeed(speedR+M2_OFFSET);
    motor3->setSpeed(speedR+M3_OFFSET);
  
    // Set direction
    // Left motors
    motor1->run(dirL);
    motor4->run(dirL);
    // Right motors
    motor2->run(dirR);
    motor3->run(dirR);
    
  #else
  
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
    motor4->run(RELEASE);
    
  #endif

  return;
  
}

void backUp() {
  driveMotors( DEFAULT_SPEED, BACKWARD, DEFAULT_SPEED, BACKWARD );
  delay(1250);
  return;
}

void rotateR() {
  driveMotors( DEFAULT_SPEED, FORWARD, DEFAULT_SPEED, BACKWARD );
  delay(ROT_DELAY);
  return;
}

void rotateL() {
  driveMotors( DEFAULT_SPEED, BACKWARD, DEFAULT_SPEED, FORWARD );
  delay(ROT_DELAY);
  return;
}


