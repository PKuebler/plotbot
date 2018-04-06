// https://github.com/tinkerlog/Kritzler/blob/master/firmware/KritzASketch/KritzASketch.ino
#include <Wire.h>
#include <Servo.h> 
#include <Adafruit_MotorShield.h>

// 0, 0 -> Mitte vom Sketch

// =============================
// Distance
// =============================
#define AXIS_DISTANCE 6300 // mm = 150cm

#define START_X 3150 // 75cm
#define START_Y 3150 // 75cm

// =============================
// MIN / MAX X/Y
// =============================
#define MIN_X 2000 // 40cm
#define MIN_Y 2000 // 40cm
#define MAX_X 5000 // 110cm
#define MAX_Y 5000 // 110cm

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
#define STEPS_PER_ROT 200

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
long stepsM1 = 0;
long stepsM2 = 0;
int penState = PEN_DOWN;
long m2s = 0;

// =============================
// Commands
// =============================
typedef struct {
  int cmd;
  long x;
  long y;
  long targetM1;
  long targetM2;
  long directionM1;
  long directionM2;
} command;

#define MAX_COMMANDS 10
command cmdBuffer[MAX_COMMANDS];

// =============================
// Calc Kable length
// thx: https://github.com/tinkerlog/Kritzler/blob/master/firmware/KritzASketch/KritzASketch.ino
// =============================
int computeA(long x, long y) {
  return sqrt(x * x + y * y);
}
  
int computeB(long x, long y) {
  long distanceX = AXIS_DISTANCE - x;
  return sqrt((distanceX * distanceX) + y * y);
}

int ggt(int x, int y) {
  int c;
  if (x < 0) x = -x;
  if (y < 0) y = -y;
  while (y != 0) {
    c = x % y; x = y; y = c;
  }
  return x;
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
  stepsM1 = computeA(START_X, START_Y) / m2s;
  stepsM2 = computeB(START_X, START_Y) / m2s;

  // Init Stepper
  AFMS.begin();

  stepperOne->setSpeed(10);  // 10 rpm
  stepperTwo->setSpeed(10);  // 10 rpm

  // Init Servo
  // Move Pen UP
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

  for (long i = 0; i < length; i++) {
    char c = command.charAt(i);

    if (c == ' ') {
      // next
      if (prop == 'G') {
        cmd = value.toInt();
      } else if (prop == 'X') {
        x = value.toInt();
      } else if (prop == 'Y') {
        y = value.toInt();
      } else if (prop == 'E') {
        e = value.toInt();
      }
      value = "";
    } else if (c == 'G' || c == 'X' || c == 'Y' || c == 'E') {
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

  // Min / Max
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
  Serial.print(y);
  Serial.print(", e: ");
  Serial.println(e);

  // Compute current a and b
  long aOld = computeA(currentX, currentY) / m2s;
  long bOld = computeB(currentX, currentY) / m2s;

  // Compute new a and b
  long a = computeA(x, y) / m2s;
  long b = computeB(x, y) / m2s;

  a = a - aOld;
  b = b - bOld;

  // Direction
  int aD = FORWARD;
  int bD = BACKWARD;
  if (a < 0) {
    a = -a;
    aD = BACKWARD;
  }
  if (b < 0) {
    b = -b;
    bD = FORWARD;
  }

  Serial.print("Moving a ");
  Serial.print(a);
  Serial.print(" steps ");
  Serial.println(aD);

  Serial.print("Moving b ");
  Serial.print(b);
  Serial.print(" steps ");
  Serial.println(bD);
  
  // add to array
	cmdBuffer[0].cmd = cmd;
	cmdBuffer[0].x = x;
	cmdBuffer[0].y = y;
	//cmdBuffer[0].e = e;
	cmdBuffer[0].targetM1 = a;
  cmdBuffer[0].directionM1 = aD;
	cmdBuffer[0].targetM2 = b;
  cmdBuffer[0].directionM2 = bD;
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
    Serial.println(serialInputString);

    parseCommand(serialInputString);

    Serial.println(cmdBuffer[0].targetM1);
    Serial.println(cmdBuffer[0].targetM2);

    int a = cmdBuffer[0].targetM1;
    int b = cmdBuffer[0].targetM2;
    boolean basis = false; // false == b 
    int basisValue = b;
    
    double g = a / b;
    
    if (g < 1) {
      g = b / a;
      basis = true;
      basisValue = a;
    }
    
    Serial.println(g);
    
    for (int i = 0; i < basisValue; i++) {
       
      if (!basis) {
        Serial.print(g);
        Serial.println(" Step per b");
        // one step b
        stepperTwo->step(1, cmdBuffer[0].directionM2, SINGLE);
        // a/b steps a 
        stepperOne->step(g, cmdBuffer[0].directionM1, SINGLE);
      } else {
        Serial.print(g);
        Serial.println(" Step per a");
        // one step a 
        stepperOne->step(1, cmdBuffer[0].directionM1, SINGLE);
        // b/a steps b
        stepperTwo->step(g, cmdBuffer[0].directionM2, SINGLE);
      }
    }

    /*
    for (int i = 0; i < g; i++) {
      stepperOne->step(a, cmdBuffer[0].directionM1, SINGLE);
      stepperTwo->step(b, cmdBuffer[0].directionM2, SINGLE);
    }
    */
    currentX = cmdBuffer[0].x;
    currentY = cmdBuffer[0].y;
    serialInputString = "";
    serialInputComplete = false;
  }
}
