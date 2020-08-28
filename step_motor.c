/*     Simple Stepper Motor Control Exaple Code
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
// defines pins numbers
const int stepPin = 3; 
const int dirPin = 4; 

const int stepPin2=5;
const int dirPin2=6;

const int stepPin3=7;
const int dirPin3=8;

const int stepPin4=9; 
const int dirPin4=10;

void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(stepPin2,OUTPUT);
  pinMode(dirPin2,OUTPUT);
  pinMode(stepPin3,OUTPUT);
  pinMode(dirPin3,OUTPUT);
  pinMode(stepPin4,OUTPUT);
  pinMode(dirPin4,OUTPUT);
  Serial.begin(9600);
}
void loop() {
  Serial.println("deneme1");
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  digitalWrite(dirPin2,HIGH); // Enables the motor to move in a particular direction
  digitalWrite(dirPin3,HIGH); // Enables the motor to move in a particular direction
  digitalWrite(dirPin4,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin,HIGH);
    digitalWrite(stepPin2,HIGH);
    digitalWrite(stepPin3,HIGH);
    digitalWrite(stepPin4,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW);
    digitalWrite(stepPin2,LOW);
    digitalWrite(stepPin3,LOW);
    digitalWrite(stepPin4,LOW);
    delayMicroseconds(500); 
  }
  delay(1000); // One second delay
  Serial.println("deneme2");
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  digitalWrite(dirPin2,LOW); //Changes the rotations direction
  digitalWrite(dirPin3,LOW); //Changes the rotations direction
  digitalWrite(dirPin4,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin,HIGH);
    digitalWrite(stepPin2,HIGH);
    digitalWrite(stepPin3,HIGH);
    digitalWrite(stepPin4,HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin,LOW);
    digitalWrite(stepPin2,LOW);
    digitalWrite(stepPin3,LOW);
    digitalWrite(stepPin4,LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}
