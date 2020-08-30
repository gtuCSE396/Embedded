#include <AccelStepper.h>

// The X Stepper pins
#define STEPPER1_DIR_PIN 4
#define STEPPER1_STEP_PIN 3
// The Y stepper pins
#define STEPPER2_DIR_PIN 6
#define STEPPER2_STEP_PIN 5

#define STEPPER3_DIR_PIN 8
#define STEPPER3_STEP_PIN 7

#define STEPPER4_DIR_PIN 12
#define STEPPER4_STEP_PIN 11


#define UP -550
#define DOWN -410

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, STEPPER3_STEP_PIN, STEPPER3_DIR_PIN);
AccelStepper stepper4(AccelStepper::DRIVER, STEPPER4_STEP_PIN, STEPPER4_DIR_PIN);

int i = 0;
void setup()
{ 
  pinMode(8,OUTPUT);
  digitalWrite(8,LOW);
  stepper1.setMaxSpeed(150.0);
  stepper1.setAcceleration(500.0);
  stepper1.moveTo(UP);

  stepper2.setMaxSpeed(150.0);
  stepper2.setAcceleration(500.0);
  stepper2.moveTo(UP);

  stepper3.setMaxSpeed(150.0);
  stepper3.setAcceleration(500.0);
  stepper3.moveTo(UP);

  stepper4.setMaxSpeed(150.0);
  stepper4.setAcceleration(500.0);
  stepper4.moveTo(UP);



  while (stepper1.distanceToGo() != 0 && stepper2.distanceToGo() != 0 && stepper3.distanceToGo() != 0 && stepper4.distanceToGo() != 0)
  {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }

  stepper1.setAcceleration(500.0);
  stepper2.setAcceleration(500.0);
  stepper3.setAcceleration(500.0);
  stepper4.setAcceleration(500.0);

  delay(150);

}

void loop()
{
  // Change direction at the limits
  if (stepper1.distanceToGo() == 0) {
    if (i % 2 == 0)
      stepper1.moveTo(DOWN);
    else
      stepper1.moveTo(UP);
  }
  if (stepper2.distanceToGo() == 0) {
    if (i % 2 == 0)
      stepper2.moveTo(DOWN);
    else
      stepper2.moveTo(UP);
  }
  if (stepper3.distanceToGo() == 0) {
    if (i % 2 == 0)
      stepper3.moveTo(DOWN);
    else
      stepper3.moveTo(UP);
  }
  if (stepper4.distanceToGo() == 0) {
    if (i % 2 == 0)
      stepper4.moveTo(DOWN);
    else
      stepper4.moveTo(UP);
  }
  ++i;
  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();
}
