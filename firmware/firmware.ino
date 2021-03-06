// https://github.com/tinkerlog/Kritzler/blob/master/firmware/KritzASketch/KritzASketch.ino
#include <Wire.h>
#include <Servo.h> 
#include <Adafruit_MotorShield.h>

// 0, 0 -> Mitte vom Sketch

// =============================
// Distance
// =============================
#define AXIS_DISTANCE 6000 // mm = 150cm
#define START_X 3000 // 75cm
#define START_Y 4750 // 75cm

// =============================
// MIN / MAX X/Y
// =============================
#define MIN_X 2000 // 40cm
#define MIN_Y 3000 // 40cm
#define MAX_X 5000 // 110cm
#define MAX_Y 6000 // 110cm

// =============================
// PEN STATS
// =============================
#define PEN_UP 0
#define PEN_DOWN 1
// Servo Settings
#define PEN_SERVO_UP 55
#define PEN_SERVO_DOWN 90

// =============================
// Commands
// =============================
#define CMD_LINIAR 1
#define CMD_PAUSE 4
#define CMD_HOMING 28
#define CMD_ABSOLUTE 90
#define CMD_RELATIVE 91

// =============================
// Set Motor Radius
// =============================
#define PULLEY_R 96
#define PI 3.14159
#define STEPS_PER_ROT 300

// =============================
// Set Motors
// =============================
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *stepperOne = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *stepperTwo = AFMS.getStepper(200, 2);

// =============================
// Set Serial Input
// =============================
String serialInputString = "";
boolean serialInputComplete = false;

// =============================
// Current Stats
// =============================
long currentX = 0;
long currentY = 0;
int penState = PEN_DOWN;
long m2s = 0;

// =============================
// Commands
// =============================
typedef struct {
  int cmd;
  long x;
  long y;
  long targetM1Steps;
  long targetM2Steps;
  long directionM1;
  long directionM2;
} command;

#define MAX_COMMANDS 10
command cmdBuffer = {};

// =============================
// Calc Kable length
// thx: https://github.com/tinkerlog/Kritzler/blob/master/firmware/KritzASketch/KritzASketch.ino
// =============================
int computeA(long x, long y) {
  return sqrt(x * x + y * y);
}
  
int computeB(long x, long y) {
  long distanceX = AXIS_DISTANCE - x;
  return sqrt(y * y + (distanceX * distanceX));
}

float getRestPerBasisStep(int basisValue, long stepsPerStep, int targetM1Steps, int targetM2Steps) {
   float rest = 0;
    if (basisValue == targetM1Steps) {
      rest = targetM2Steps - (stepsPerStep * basisValue);

      // Debug output
      Serial.print("Rest: ");
      Serial.println(rest);
    } else if (basisValue == targetM2Steps) {
      rest = targetM1Steps - (stepsPerStep * basisValue);
      
      Serial.print("Rest: ");
      Serial.println(rest);
    } else {
      Serial.println("Error: getRestPerBasisStep: Base value is not matched.");
			return;
    }

    return rest / basisValue;
}

void drawLineToTargetPoint(int stepsPerStep, int basisValue, float restPerStep, command cmdBuffer) {
  // start drawing
  float currentRest = 0;
  int targetSteps = stepsPerStep;

  for (int i = 0; i < basisValue; i++) {
    // calculate current rest
    currentRest = currentRest + restPerStep;

    if (currentRest >= 1) {
      // add one step if rest is larger than one
      currentRest = 1 - currentRest;
      targetSteps = targetSteps + 1;
    }

    //Serial.print("Current Rest ");
    //Serial.println(currentRest);

    if (basisValue == cmdBuffer.targetM1Steps) {
      // Debug output
      //Serial.print("Do ");
      //Serial.print(targetSteps);
      //Serial.println(" Steps per one M1 step");

      // Do x Steps on M2 per one step on M1
      stepperTwo->step(1, cmdBuffer.directionM1, SINGLE);
      stepperOne->step(targetSteps, cmdBuffer.directionM2, SINGLE);
    } else if (basisValue == cmdBuffer.targetM2Steps) {
      // Debug output
      //Serial.print("Do ");
      //Serial.print(targetSteps);
      //Serial.println(" Steps per one M2 step");

      // Do x Steps on M1 per one step on M2
      stepperOne->step(1, cmdBuffer.directionM2, SINGLE);
      stepperTwo->step(targetSteps, cmdBuffer.directionM1, SINGLE);
    } else {
      // Unexpected error
      Serial.println("Error: drawLineToTargetPoint: Base value is not matched.");
			return;
    }

    // reset targetSteps
    if (targetSteps != stepsPerStep) {
      targetSteps = stepsPerStep;
    }
  }
}

// =============================
// Setup
// =============================
void setup() {
  Serial.begin(9600);
  Serial.println("***** BOOT *****");

  // compute mm to steps
  m2s = (2 * PI * PULLEY_R) / STEPS_PER_ROT;

  // compute starting pos
  currentX = START_X;
  currentY = START_Y;
  
  // Init Stepper
  AFMS.begin();

  stepperOne->setSpeed(10);  // 10 rpm
  stepperTwo->setSpeed(10);  // 10 rpm
}

// =============================
// Parse Commands by Spaces
// =============================
void parseCommand(String command) {
  // add "end"
  command += " ";

  // length
  long length = command.length();

  // result
  long x = -1, y = -1, e = -1;
  int cmd = -1; // #define to start

  // current
  char prop; // G / X / Y
  String value;

  char c = "";
  for (long i = 0; i < length; i++) {
    c = command.charAt(i);

    if (c == ' ') {
      // next
      switch(prop) {
        case 'G':
          cmd = value.toInt();  
        break;
        case 'X':
          x = value.toInt();
        break;
        case 'Y':
          y = value.toInt();
        break;
      }
      value = "";
    } else if (c == 'G' || c == 'X' || c == 'Y') {
      // new
      prop = c;
    } else {
      value += c;
    }
  }
  
  if (cmd == -1) {
    Serial.println("> Invalid Command: Code not found.");
    return;
  }

  // Debug Output
  Serial.print("#cmd ");
  Serial.print(cmd);
  Serial.print(" - x: ");
  Serial.print(x);
  Serial.print(", y: ");
  Serial.print(y);
  
  // reset min / max if nesseccary
  if (x < MIN_X) x = MIN_X;
  if (y < MIN_Y) y = MIN_Y;
  if (x > MAX_X) x = MAX_X;
  if (y > MAX_Y) y = MAX_Y;
  
  // Debug Output
  Serial.print("#cmd ");
  Serial.print(cmd);
  Serial.print(" - x: ");
  Serial.print(x);
  Serial.print(", y: ");
  Serial.println(y);


  // Compute current (old) targetM1-steps and targetM2-steps
  long targetM1Old = computeA(currentX, currentY);
  long targetM2Old = computeB(currentX, currentY);
  Serial.print("Old M1 ");
  Serial.println(targetM1Old);
  Serial.print("Old M2 ");
  Serial.println(targetM2Old);
  long targetM1OldSteps = targetM1Old / m2s;
  long targetM2OldSteps = targetM2Old / m2s;


  // Compute new targetM1-steps and targetM2-steps
  long targetM1 = computeA(x, y);
  long targetM2 = computeB(x, y);
  Serial.print("New M1 ");
  Serial.println(targetM1);
  Serial.print("New M2 ");
  Serial.println(targetM2);
  long targetM1Steps = targetM1 / m2s;
  long targetM2Steps = targetM2 / m2s;

  targetM1Steps = targetM1OldSteps - targetM1Steps;
  targetM2Steps = targetM2OldSteps - targetM2Steps;
  
  // Direction
  int directionM1 = BACKWARD;
  int directionM2 = BACKWARD;

  // Debug output
  Serial.print("Moving m1 ");
  Serial.print(targetM1Steps);
  Serial.print(" steps ");
  Serial.println(directionM1 == 1 ? "längen" : "kürzen");

  Serial.print("Moving m2 ");
  Serial.print(targetM2Steps);
  Serial.print(" steps ");
  Serial.println(directionM2 == 1 ? "längen" : "kürzen");

  if (targetM1Steps < 0) {
    targetM1Steps = -targetM1Steps;
    directionM1 = FORWARD;
  }
  if (targetM2Steps < 0) {
    targetM2Steps = -targetM2Steps;
    directionM2 = FORWARD;
  }

  // Debug output
  Serial.print("Moving m1 ");
  Serial.print(targetM1Steps);
  Serial.print(" steps ");
  Serial.println(directionM1 == 1 ? "längen" : "kürzen");

  Serial.print("Moving m2 ");
  Serial.print(targetM2Steps);
  Serial.print(" steps ");
  Serial.println(directionM2 == 1 ? "längen" : "kürzen");
  
  // add to struct
	cmdBuffer.cmd = cmd;
	cmdBuffer.x = x;
	cmdBuffer.y = y;
	cmdBuffer.targetM1Steps = targetM1Steps;
  cmdBuffer.directionM1 = directionM1;
	cmdBuffer.targetM2Steps = targetM2Steps;
  cmdBuffer.directionM2 = directionM2;
}

// =============================
// Read Serial Commands
// =============================
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    serialInputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      serialInputComplete = true;
    }
  }
}

// =============================
// Run
// =============================
void loop () {
  if (serialInputComplete) {
		// Debug ouput
    Serial.println(serialInputString);

    parseCommand(serialInputString);

		// Debug ouput
    Serial.println(cmdBuffer.targetM1Steps);
    Serial.println(cmdBuffer.targetM2Steps);

    // Get target values
    int targetM1Steps = cmdBuffer.targetM1Steps;
    int targetM2Steps = cmdBuffer.targetM2Steps;
    // default base value
    int basisValue = targetM2Steps;
    
    // calculate stepsPerStep
    long stepsPerStep = targetM1Steps / targetM2Steps;
    if (stepsPerStep < 1) {
      // invert and set base to targetM1Steps
      stepsPerStep = targetM2Steps / targetM1Steps; // 1 / stepsPerStep;
      basisValue = targetM1Steps;
    }

    float restPerStep = getRestPerBasisStep(basisValue, stepsPerStep, targetM1Steps, targetM2Steps);

    // Steps
    Serial.print("Steps Basis: ");
    Serial.println(basisValue);

    // steps per BasisStep
    Serial.print("Steps per BasisStep: ");
    Serial.println(stepsPerStep);

    // start drawing
		drawLineToTargetPoint(stepsPerStep, basisValue, restPerStep, cmdBuffer);

    currentX = cmdBuffer.x;
    currentY = cmdBuffer.y;
    Serial.println("finish");
    serialInputString = "";
    serialInputComplete = false;
  }
}
