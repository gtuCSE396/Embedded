
#include <AccelStepper.h>

#define STEPX 2
#define DIRX 5

#define STEPY 3
#define DIRY 6

#define STEPZ 4
#define DIRZ 7

#define STEPA 12
#define DIRA 13

#define EN 8 
#define init_pos 400  

#define _BUFF_SIZE 1024 


char _buffer[_BUFF_SIZE];
int counter = 0 ;
AccelStepper stepperX(AccelStepper::DRIVER, STEPX, DIRX);
AccelStepper stepperY(AccelStepper::DRIVER, STEPY, DIRY);
AccelStepper stepperZ(AccelStepper::DRIVER, STEPZ, DIRZ);
AccelStepper stepperA(AccelStepper::DRIVER, STEPA, DIRA);

int x,y,z,a;
double acceleration ;
double s ;
void setup() {

  Serial.begin(9600);
  pinMode(EN,OUTPUT);
  digitalWrite(EN,LOW);
  
  go_to(init_pos,init_pos,init_pos,init_pos,5000.0,5000.0);
  
}

void loop() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if(ch == '\n'){
      _buffer[counter++] = 0 ;      
      char *temp = strtok(_buffer,":");
      x = atoi(temp) + init_pos ;
      temp = NULL ;
      temp = strtok(0,":");
      y = atoi(temp) + init_pos;
      temp = NULL ;
      temp = strtok(0,":");
      z = atoi(temp) + init_pos;
      temp = NULL ;
      temp = strtok(0,":");
      a = atoi(temp) + init_pos;
      temp=NULL;
      temp = strtok(0,":");
      acceleration = (double)atoi(temp);
      temp=NULL;
      temp = strtok(0,":");
      s = (double)atoi(temp);
      temp=NULL;
      
      go_to(x,y,z,a,s,acceleration);
      counter = 0 ;
      
    }
    else{
      _buffer[counter++] = ch ;
    }
    
    
  }
  
  

}

void go_to(int x , int y , int z , int a , double _speed , double acceleration){
  
  
  stepperX.setMaxSpeed(_speed);
  stepperX.setAcceleration(acceleration);
  stepperX.moveTo(x);

  stepperY.setMaxSpeed(_speed);
  stepperY.setAcceleration(acceleration);
  stepperY.moveTo(y);

  stepperZ.setMaxSpeed(_speed);
  stepperZ.setAcceleration(acceleration);
  stepperZ.moveTo(z);

  stepperA.setMaxSpeed(_speed);
  stepperA.setAcceleration(acceleration);
  stepperA.moveTo(a);

  run_all();

}


void run_all(){
 
   while (!(stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0 &&
           stepperZ.distanceToGo() == 0 && stepperA.distanceToGo() == 0))
  {
    stepperX.run();
    stepperY.run();
    stepperZ.run();
    stepperA.run();
  }
  
}
