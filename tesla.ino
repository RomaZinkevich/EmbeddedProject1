#include <LiquidCrystal.h>
#include <Wire.h>

#define CMPS14_address 0x60 // I2C slave address for CMPS compass module 
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10



LiquidCrystal lcd(37, 36, 35, 34, 33, 32);

const int joystickXPin = A8; // X-axis pin
const int joystickYPin = A9; // Y-axis pin
const int joystickButtonPin = 19; //Joystick's button pin
const int encoderPinRight = 3; //Pin for right wheel
const int encoderPinLeft = 2; //Pin for left wheel

boolean buttonPressed = false; //Flag to determine if button is pressed
volatile long pulseCountRight = 0; //Pulse counter for right wheel
volatile long pulseCountLeft = 0;
volatile unsigned long lastDebounceTime = 0; // The last time the button was pressed
const unsigned long debounceDelay = 50; 
int offset=220;//offset for compass


void setup() {
  Wire.begin(); //Setup for compass

  //Setup for wheels
  pinMode(encoderPinRight, INPUT);
  pinMode(encoderPinLeft, INPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinRight), updatePulseCountRight, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinLeft), updatePulseCountLeft, RISING);
  //Setuo for Joystick
  pinMode(joystickXPin, INPUT);
  pinMode(joystickYPin, INPUT);
  pinMode(joystickButtonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(joystickButtonPin), buttonInterrupt, FALLING);

  lcd.begin(20, 4);
  Serial.begin(9600);
}

void loop() {
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
  if (buttonPressed){ //Driving car from Webpage mode
    lcd.setCursor(18,3);
    lcd.print("W");
    webMode();
  }
  else{ //Driving car from Joystick mode
    lcd.setCursor(18,3);
    lcd.print("J");
    joystickMode();
  }
  printData();//Printing all required data to LCD
}

void printData(){
  lcd.setCursor(0,0);
  printPulses();
  printDegrees();
  delay(100);
  lcd.clear();
}

void webMode(){//Mode when car can be driven from webpage
  if (Serial.available() > 0) {//If some message arrived
    String message = Serial.readStringUntil('\n');
    int pos_s = message.indexOf(":");
    //Gets index of Command (Can be [Move, Turn, DIr])
    int movecommand = message.indexOf("Move");
    int steercommand = message.indexOf("Turn");
    int degreescommand = message.indexOf("Dir");
    if(pos_s > -1) {//If ":" was in the message then we have right command
      if (movecommand==0){
        int stat = message.substring(pos_s + 1).toInt();//Gets number from the message
        if (stat > -21 && stat < 21) {//Can be moved 20 forward and 20 back
          Serial.print("Drive: ");
          Serial.println(stat);
          drive(0, stat);
        } else {
          Serial.println("Put proper number -20 - 20");
        }
      }
      else if (steercommand==0){
        int stat = message.substring(pos_s + 1).toInt();//Gets number from the message
        if (stat > -181 && stat < 181) {//Can turn 180 to the right and 180 to the left
          Serial.print("Turn: ");
          Serial.println(stat);
          turn(0, stat);
        } else {
          Serial.println("Put proper number -180 - 180");
        }
      }
      else if (degreescommand==0){
        int stat = message.substring(pos_s + 1).toInt();//Gets number from the message
        if (stat >= 0 && stat <= 360) {//Can rotate to degrees from 0 to 360
          Serial.print("Dir: ");
          Serial.println(stat);
          rotate(0, stat);
        } else {
          Serial.println("Put proper number 0 - 360");
        }
      }
    }
  }
}

void joystickMode(){//Mode when car is driven by joystick
    int left,right;
    int analogValue1 = analogRead(A9); // y
    int analogValue2 = analogRead(A8); // x
    analogValue1 = (analogValue1 - 1023) * -1; //invert the Y-axis direction
    int joystickPositionY = map(analogValue1, 0, 1023, -100, 100); // Invert Y-axis
    int joystickPositionX = map(analogValue2, 0, 1023, -100, 100); // Invert X-axis

    // Calculate motor speeds based on joystick positions
    int baseSpeed = map(abs(joystickPositionY), 0, 100, 0, 255);
    int steering = map(joystickPositionX, -100, 100, -255, 255);

    int pwm_L = baseSpeed + steering;
    int pwm_R = baseSpeed - steering;

    // Ensure that the motor speeds are within the valid range (0-255)
    pwm_L = constrain(pwm_L, 0, 255);
    pwm_R = constrain(pwm_R, 0, 255);

    // Set the direction pins based on joystick position
    if (joystickPositionY < 0) {
      left = 1;
      right = 1;
    } else if (joystickPositionY > 0) {
      left = 0;
      right = 0;
    }

    //Apply the motor control signals
    digitalWrite(Motor_R_dir_pin, right);
    digitalWrite(Motor_L_dir_pin, left);
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);

    // Add a delay for control loop stability (adjust as needed)
    delay(20); // 50 Hz update rate for control

    //Stop the motors if the joystick is centered
    if (joystickPositionY == 0 && joystickPositionX == 0) {
      analogWrite(Motor_L_pwm_pin, 0);
      analogWrite(Motor_R_pwm_pin, 0);
    }
}

void printDegrees(){
  int degrees;
  //Calculates degrees from the compass
  Wire.beginTransmission(CMPS14_address);    
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(CMPS14_address, 2, true); 
  if (Wire.available() >= 2) { 
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    int heading = (highByte << 8) + lowByte;
    degrees = ((heading / 10) + offset) % 360;
  }
  else degrees = 404;
  lcd.setCursor(0,2);
  lcd.print("Dgrs: ");
  lcd.print(degrees);
  lcd.setCursor(0,3);
  lcd.print("Dir: ");
  lcd.print(compassDirection(degrees));//Gets exact direction of the car (North, West, South)
}

void printPulses(){
  int leftDistance, rightDistance;
  //Calculate how many cm car moved according to pulses from the wheel
  //200 pulses = full rotation
  //wheel's radius = 3 cm, so circumference = 18.85 cm
  //so 200 pulses = 18.85 cm
  leftDistance = round(pulseCountLeft / 200.0 * 18.85); 
  rightDistance = round(pulseCountRight / 200.0 * 18.85);
  lcd.setCursor(0,0);
  lcd.print("Pulse L:");
  lcd.print(pulseCountLeft);
  lcd.print(" R:");
  lcd.print(pulseCountRight);
  lcd.setCursor(0, 1);
  lcd.print("Dist L:");
  lcd.print(leftDistance);
  lcd.print("cm");
  lcd.print(" R:");
  lcd.print(rightDistance);
  lcd.print("cm");
}

String compassDirection(int degrees){//Converts degrees to direction
  if (degrees >= 0 && degrees < 22.5) {
    return "North";
  } else if (degrees >= 22.5 && degrees < 67.5) {
    return "NorthEast";
  } else if (degrees >= 67.5 && degrees < 112.5) {
    return "East";
  } else if (degrees >= 112.5 && degrees < 157.5) {
    return "SouthEast";
  } else if (degrees >= 157.5 && degrees < 202.5) {
    return "South";
  } else if (degrees >= 202.5 && degrees < 247.5) {
    return "SouthWest";
  } else if (degrees >= 247.5 && degrees < 292.5) {
    return "West";
  } else if (degrees >= 292.5 && degrees < 337.5) {
    return "NorthWest";
  } else if (degrees >= 337.5 && degrees <= 360) {
    return "North";
  } else {
    return "Invalid input";
  }
}

void turn(int encoderValue, int extraDegrees) {//"Turn" command
  int fin, initDegrees;
  //calculates position of the car before rotating
  //also calculates position it supposed to be in
  Wire.beginTransmission(CMPS14_address);    
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(CMPS14_address, 2, true); 
  if (Wire.available() >= 2) { 
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    int heading = (highByte << 8) + lowByte;
    int initDegrees = (heading / 10 + offset) % 360; 
    fin = initDegrees + extraDegrees;
  }

  //while true loop that breaks if car reached wanted position
  while (true){
    printData();
    //calculating position of the car
    Wire.beginTransmission(CMPS14_address);    
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(CMPS14_address, 2, true); 
    if (Wire.available() >= 2) { 
      byte highByte = Wire.read();
      byte lowByte = Wire.read();
      int heading = (highByte << 8) + lowByte;
      int degrees = (heading / 10 + offset) % 360; 
      // Clockwise rotation
      if (extraDegrees>0){//Looping "position", so it goes 0-359 and 360-359
        if ((degrees < fin) && (fin-degrees) < 360){
          Serial.println(degrees);
          digitalWrite(Motor_R_dir_pin, 0);
          digitalWrite(Motor_L_dir_pin, 0);
          analogWrite(Motor_L_pwm_pin, 150);
          analogWrite(Motor_R_pwm_pin, 0);
        }
        else if ((fin-degrees) > 360) {
          fin-=360;
        }
        else {//If it reached required degrees car stops
          stopMovement();
          break;
        }
      }
      //Counter-clockwise rotation
      else if (extraDegrees<0){
        if ((degrees > fin) && (degrees-fin) < 360){
          Serial.println(degrees);
          Serial.println(fin);
          digitalWrite(Motor_R_dir_pin, 0);
          digitalWrite(Motor_L_dir_pin, 0);
          analogWrite(Motor_R_pwm_pin, 150);
          analogWrite(Motor_L_pwm_pin, 0);
        }
        else if ((degrees-fin) > 360) {
          fin+=360;
        }
        else {
          stopMovement();
          break;
        }
      }
      
    }
  }
  
}

void rotate(int encoderValue, int degrees) {//"Dir" command
  int offsets[] = {-4, -3, -2, -1, 0, 1, 2, 3, 4};//"Tolerance" for car's final position, so it is not exactly facing required position
  while (true) {
    printData();
    //calculating car's initial position
    Wire.beginTransmission(CMPS14_address);    
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(CMPS14_address, 2, true); 
    if (Wire.available() >= 2) { 
      byte highByte = Wire.read();
      byte lowByte = Wire.read();
      int heading = (highByte << 8) + lowByte;
      int initDegrees = ((heading / 10) + offset) % 360; 
      boolean isValid = true;
        for (int offset : offsets) {//If car is in required position (+- one of offsets) car should stop
          if (degrees == initDegrees + offset) {
            isValid = false;
            break;
          }
        }

      if (isValid) {
        digitalWrite(Motor_R_dir_pin, 0);
        digitalWrite(Motor_L_dir_pin, 0);
        analogWrite(Motor_R_pwm_pin, 150);
        analogWrite(Motor_L_pwm_pin, 0);
      }
      else {
        stopMovement();
        break;
      }
    }
  }
}

void drive(int encoderValue, int distance) {//"Move" command
  int left, right;
  int initPulsesDist, pulsesDist=0;
  //Determines direction according to negative or positive value of "distance"
  if (distance < 0) {
    left = 1;
    right = 1;
    distance = distance*-1;
  } else if (distance > 0) {
    left = 0;
    right = 0;
  }
  digitalWrite(Motor_R_dir_pin, right);
  digitalWrite(Motor_L_dir_pin, left);
  analogWrite(Motor_L_pwm_pin, 255);
  analogWrite(Motor_R_pwm_pin, 255);
  initPulsesDist = round(pulseCountLeft / 200.0 * 18.85);//Initial distance calculated from pulses (because pulses counter are not zeroed if you call function several times)
  while (pulsesDist<(initPulsesDist+distance)){//If (current distance calculated from pulses) < (initial distance + required distance) 
    printData();
    pulsesDist = round(pulseCountLeft / 200.0 * 18.85);
  }
  stopMovement();
}

void stopMovement(){//Stops the car
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
}

//Interrupt function
void updatePulseCountRight() {//Pulse counter for right wheel
  pulseCountRight++;
}

//Interrupt function
void updatePulseCountLeft() {//Pulse counter for left wheel
  pulseCountLeft++;
}


void buttonInterrupt() {
  // This function is called when the button is pressed
  if (millis() - lastDebounceTime > debounceDelay) {//debounce delay so that function is called only once when you press it once
    buttonPressed = !buttonPressed;
    lastDebounceTime = millis();
  }
}
