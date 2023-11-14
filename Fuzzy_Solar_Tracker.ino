#include <Fuzzy.h>
#include <Servo.h>

// For scope, instantiate all objects you will need to access in loop()
// It may be just one Fuzzy, but for demonstration, this sample will print
// all FuzzySet pertinence

//Functions to Read Sensor
float sensorRead(int analog)
{
  int value;
  value = analogRead(analog); // read the value from the sensor
  return value;
}

Servo servohori;  //Initialize Horizontal Servo Motor
Servo servoverti; //Initialize Vertical Servo Motor

int servoh = 0; //Store the current Angle of Horizontal Motor
int servov = 0; //Store the current Angle of Vertical Motor

int sensitivity = 1;

//Sensor Reading
int AnalogTop = A2; // select the input pin for LDR
int TopSensor = 0; // variable to store the value coming from the sensor

int AnalogBottom = A1; // select the input pin for LDR
int BottomSensor = 0; // variable to store the value coming from the sensor

int AnalogLeft = A3; // select the input pin for LDR
int LeftSensor = 0; // variable to store the value coming from the sensor

int AnalogRight = A0; // select the input pin for LDR
int RightSensor = 0; // variable to store the value coming from the sensor



// Fuzzy
Fuzzy *fuzzy = new Fuzzy();

// Top Sensor Fuzzy Input
FuzzySet *Tdim = new FuzzySet(0, 0, 100, 200);
FuzzySet *Tnormal = new FuzzySet(150, 250, 250, 350);
FuzzySet *Tbright = new FuzzySet(300, 400, 500, 500);

// Bottom Sensor Fuzzy Input
FuzzySet *Bdim = new FuzzySet(0, 0, 100, 200);
FuzzySet *Bnormal = new FuzzySet(150, 250, 250, 350);
FuzzySet *Bbright = new FuzzySet(300, 400, 500, 500);

// Left Sensor Fuzzy Input
FuzzySet *Ldim = new FuzzySet(0, 0, 100, 200);
FuzzySet *Lnormal = new FuzzySet(150, 250, 250, 350);
FuzzySet *Lbright = new FuzzySet(300, 400, 500, 500);

// Right Sensor Fuzzy Input
FuzzySet *Rdim = new FuzzySet(0, 0, 100, 200);
FuzzySet *Rnormal = new FuzzySet(150, 250, 250, 350);
FuzzySet *Rbright = new FuzzySet(300, 400, 500, 500);

// Vertical Motor Fuzzy Output
FuzzySet *Vlow = new FuzzySet(60, 60, 75, 90);
FuzzySet *Vmedium = new FuzzySet(75, 90, 90, 105);
FuzzySet *Vhigh = new FuzzySet(90, 105, 120, 120);

// Horizontal Motor Fuzzy Output
FuzzySet *Hlow = new FuzzySet(0, 0, 45, 90);
FuzzySet *Hmedium = new FuzzySet(45, 90, 90, 135);
FuzzySet *Hhigh = new FuzzySet(90, 135, 180, 180);


void setup()
{
  servohori.attach(6);  // attaches the servo on pin 9 to the servo object
  servoverti.attach(7); 
  Serial.begin(9600); //sets serial port for communication
  servoverti.write(90);
  servohori.write(90);

  // Every setup must occur in the function setup()

  // FuzzyInput Top
  FuzzyInput *sensorTop = new FuzzyInput(1);

  sensorTop->addFuzzySet(Tdim);
  sensorTop->addFuzzySet(Tnormal);
  sensorTop->addFuzzySet(Tbright);
  fuzzy->addFuzzyInput(sensorTop);

  // FuzzyInput Bottom
  FuzzyInput *sensorBottom = new FuzzyInput(2);

  sensorBottom->addFuzzySet(Bdim);
  sensorBottom->addFuzzySet(Bnormal);
  sensorBottom->addFuzzySet(Bbright);
  fuzzy->addFuzzyInput(sensorBottom);

  // FuzzyInput Left
  FuzzyInput *sensorLeft = new FuzzyInput(3);

  sensorLeft->addFuzzySet(Ldim);
  sensorLeft->addFuzzySet(Lnormal);
  sensorLeft->addFuzzySet(Lbright);
  fuzzy->addFuzzyInput(sensorLeft);

  // FuzzyInput Right
  FuzzyInput *sensorRight = new FuzzyInput(4);

  sensorRight->addFuzzySet(Rdim);
  sensorRight->addFuzzySet(Rnormal);
  sensorRight->addFuzzySet(Rbright);
  fuzzy->addFuzzyInput(sensorRight);

  // FuzzyOutput Vertical Motor
  FuzzyOutput *VMotor = new FuzzyOutput(1);

  VMotor->addFuzzySet(Vlow);
  VMotor->addFuzzySet(Vmedium);
  VMotor->addFuzzySet(Vhigh);
  fuzzy->addFuzzyOutput(VMotor);

  // FuzzyOutput Horizontal Motor
  FuzzyOutput *HMotor = new FuzzyOutput(2);

  HMotor->addFuzzySet(Hlow);
  HMotor->addFuzzySet(Hmedium);
  HMotor->addFuzzySet(Hhigh);
  fuzzy->addFuzzyOutput(HMotor);

  // Building FuzzyRule Rule 1: if Top Bright And Bottom Normal then Vertical Servo Motor Low
  FuzzyRuleAntecedent *IfTopBrightAndBottomNormal = new FuzzyRuleAntecedent();
  IfTopBrightAndBottomNormal->joinWithAND(Tbright, Bnormal);
  
  FuzzyRuleConsequent *thenVerticalMotorLow = new FuzzyRuleConsequent();
  thenVerticalMotorLow->addOutput(Vlow);
  
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, IfTopBrightAndBottomNormal, thenVerticalMotorLow);
  fuzzy->addFuzzyRule(fuzzyRule1);

  // Building FuzzyRule Rule 2: if Top Normal And Bottom Bright then Vertical Servo Motor High
  FuzzyRuleAntecedent *IfTopNormalAndBottomBright = new FuzzyRuleAntecedent();
  IfTopNormalAndBottomBright->joinWithAND(Tnormal, Bbright);
  
  FuzzyRuleConsequent *thenVerticalMotorHigh = new FuzzyRuleConsequent();
  thenVerticalMotorHigh->addOutput(Vhigh);
  
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, IfTopNormalAndBottomBright, thenVerticalMotorHigh);
  fuzzy->addFuzzyRule(fuzzyRule2);

  // Building FuzzyRule Rule 3: if Top Normal And Bottom Normal then Vertical Servo Motor Medium
  FuzzyRuleAntecedent *IfTopNormalAndBottomNormal = new FuzzyRuleAntecedent();
  IfTopNormalAndBottomNormal->joinWithAND(Tnormal, Bnormal);
  
  FuzzyRuleConsequent *thenVerticalMotorMedium = new FuzzyRuleConsequent();
  thenVerticalMotorMedium->addOutput(Vmedium);
  
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, IfTopNormalAndBottomNormal, thenVerticalMotorMedium);
  fuzzy->addFuzzyRule(fuzzyRule3);

  // Building FuzzyRule Rule 4: if Left Bright And Right Normal then Horizontal Servo Motor High
  FuzzyRuleAntecedent *IfLeftBrightAndRightNormal = new FuzzyRuleAntecedent();
  IfLeftBrightAndRightNormal->joinWithAND(Lbright, Rnormal);
  
  FuzzyRuleConsequent *thenHorizontalMotorHigh = new FuzzyRuleConsequent();
  thenHorizontalMotorHigh->addOutput(Hhigh);
  
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, IfLeftBrightAndRightNormal, thenHorizontalMotorHigh);
  fuzzy->addFuzzyRule(fuzzyRule4);

  // Building FuzzyRule Rule 5: if Left Normal And Right Bright then Horizontal Servo Motor Low
  FuzzyRuleAntecedent *IfLeftNormalAndRightBright = new FuzzyRuleAntecedent();
  IfLeftNormalAndRightBright->joinWithAND(Ldim, Rbright);
  
  FuzzyRuleConsequent *thenHorizontalMotorLow = new FuzzyRuleConsequent();
  thenHorizontalMotorLow->addOutput(Hlow);
  
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, IfLeftNormalAndRightBright, thenHorizontalMotorLow);
  fuzzy->addFuzzyRule(fuzzyRule5);

  // Building FuzzyRule Rule 6: if Left Normal And Right Normal then Horizontal Servo Motor Medium
  FuzzyRuleAntecedent *IfLeftNormalAndRightNormal = new FuzzyRuleAntecedent();
  IfLeftNormalAndRightNormal->joinWithAND(Lnormal, Rnormal);
  
  FuzzyRuleConsequent *thenHorizontalMotorMedium = new FuzzyRuleConsequent();
  thenHorizontalMotorMedium->addOutput(Hmedium);
  
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, IfLeftNormalAndRightNormal, thenHorizontalMotorMedium);
  fuzzy->addFuzzyRule(fuzzyRule6);

  // Building FuzzyRule Rule 7: if Right Bright And Bottom Bright then Vertical Servo Motor High
  FuzzyRuleAntecedent *IfRightBrightAndBottomBright = new FuzzyRuleAntecedent();
  IfRightBrightAndBottomBright->joinWithAND(Rbright, Bbright);
  
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, IfRightBrightAndBottomBright, thenVerticalMotorHigh);
  fuzzy->addFuzzyRule(fuzzyRule7);

  // Building FuzzyRule Rule 8: if Right Bright And Bottom Bright then Horizontal Servo Motor High
  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, IfRightBrightAndBottomBright, thenHorizontalMotorHigh);
  fuzzy->addFuzzyRule(fuzzyRule8);
}

void loop()
{ 
  //sensorRead(int analog)

  TopSensor = sensorRead(AnalogTop);
  BottomSensor = sensorRead(AnalogBottom);
  LeftSensor = sensorRead(AnalogLeft); 
  RightSensor = sensorRead(AnalogRight); 

  // Check and cap the sensor values at a maximum of 500
  if (TopSensor >= 500) {
      TopSensor = 499;
  }

  if (BottomSensor >= 500) {
      BottomSensor = 499;
  }

  if (LeftSensor >= 500) {
      LeftSensor = 499;
  }

  if (RightSensor >= 500) {
      RightSensor = 499;
  }


  Serial.println("\n\n\nSensor Values: ");
  Serial.print("\t\t\tTop Sensor: ");
  Serial.print(TopSensor);
  Serial.print(", Bottom Sensor: ");
  Serial.print(BottomSensor);
  Serial.print(", Left Sensor: ");
  Serial.println(LeftSensor);
  Serial.print(", Right Sensor: ");
  Serial.println(RightSensor);

  fuzzy->setInput(1, TopSensor);
  fuzzy->setInput(2, BottomSensor);
  fuzzy->setInput(3, LeftSensor);
  fuzzy->setInput(4, RightSensor);

  fuzzy->fuzzify();

  //Top Sensor Value
  Serial.println("Input: ");
  Serial.print("\tTop Sensor: Dim-> ");
  Serial.print(Tdim->getPertinence());
  Serial.print(", Normal-> ");
  Serial.print(Tnormal->getPertinence());
  Serial.print(", Bright-> ");
  Serial.println(Tbright->getPertinence());

  //Bottom Sensor Value
  Serial.println("Input: ");
  Serial.print("\tBottom Sensor: Dim-> ");
  Serial.print(Bdim->getPertinence());
  Serial.print(", Normal-> ");
  Serial.print(Bnormal->getPertinence());
  Serial.print(", Bright-> ");
  Serial.println(Bbright->getPertinence());

  //Left Sensor Value
  Serial.println("Input: ");
  Serial.print("\tLeft Sensor: Dim-> ");
  Serial.print(Ldim->getPertinence());
  Serial.print(", Normal-> ");
  Serial.print(Lnormal->getPertinence());
  Serial.print(", Bright-> ");
  Serial.println(Lbright->getPertinence());

  //Right Sensor Value
  Serial.println("Input: ");
  Serial.print("\tRight Sensor: Dim-> ");
  Serial.print(Rdim->getPertinence());
  Serial.print(", Normal-> ");
  Serial.print(Rnormal->getPertinence());
  Serial.print(", Bright-> ");
  Serial.println(Rbright->getPertinence());

  float VerticalAngle = fuzzy->defuzzify(1);
  float HorizontalAngle = fuzzy->defuzzify(2);

  //Set Limits
  if (VerticalAngle < 60){
    VerticalAngle = 60;
  }
  else if (VerticalAngle > 120){
    VerticalAngle = 120;
  }

  //Vertical Angle
  Serial.println("Output: ");
  Serial.print("\tVertical Angle: Low-> ");
  Serial.print(Vlow->getPertinence());
  Serial.print(", Medium-> ");
  Serial.print(Vmedium->getPertinence());
  Serial.print(", High-> ");
  Serial.println(Vhigh->getPertinence());

  //Horizontal Angle
  Serial.println("Output: ");
  Serial.print("\tHorizontal Angle: Low-> ");
  Serial.print(Hlow->getPertinence());
  Serial.print(", Medium-> ");
  Serial.print(Hmedium->getPertinence());
  Serial.print(", High-> ");
  Serial.println(Hhigh->getPertinence());


  Serial.println("Result: ");
  Serial.print("\t\t\tVertical Angle: ");
  Serial.print(VerticalAngle);
  Serial.print(", and Horizontal Angle: ");
  Serial.println(HorizontalAngle);

  servov = servoverti.read();
  servoh = servohori.read();

  //Add Transition to Current Motor Angle to Sun Current Angle
  if (servov != VerticalAngle) {
    if (servov < VerticalAngle) {
      for (int i = servov; i <= VerticalAngle; i=i+sensitivity) {
        servoverti.write(i);
        delay(15); // Adjust delay as needed for smooth motion
      }
    } else {
      for (int i = servov; i >= VerticalAngle; i=i-sensitivity) {
        servoverti.write(i);
        delay(15); // Adjust delay as needed for smooth motion
      }
    }
  }

  if (servoh != HorizontalAngle) {
    if (servoh < HorizontalAngle) {
      for (int i = servoh; i <= HorizontalAngle; i=i+sensitivity) {
        servohori.write(i);
        delay(15); // Adjust delay as needed for smooth motion
      }
    } else {
      for (int i = servoh; i >= HorizontalAngle; i=i-sensitivity) {
        servohori.write(i);
        delay(15); // Adjust delay as needed for smooth motion
      }
    }
  }

  // wait 12 seconds
  delay(5000);
}
