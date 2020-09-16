#include <Wire.h> 
 
#define Mag1 50                    //Electromagnet-1 Pin 
#define Mag2 51                    //Electromagnet-2 Pin
#define InL1 10                    //Left Motor Pin
#define PWML 13                    //Left Motor PWM Pin
#define InL2 9                     //Left Motor Pin
#define InR1 6                     //Right Motor Pin
#define PWMR 4                     //Right Motor PWM Pin  
#define InR2 7                     //Right Motor Pin   
#define pi 3.1415926
 
#define encoder0PinA  2
#define encoder0PinB  3 
 volatile long encoder0Pos=0, encoder1Pos=0; 
 float newposition0=0, newposition1=0;
 float oldposition0 =0, oldposition1 = 0; 
 double elapsedtime,time,previoustime;
 float oldx;
            
 void motorctrl(int torque){        //torque between 0-255 
        if (torque >= 0)  {                                        // drive motors forward 
          digitalWrite(InR1, LOW); 
          digitalWrite(InR2, HIGH);  
          digitalWrite(InL1, HIGH);  
          digitalWrite(InL2, LOW);  
          torque = abs(torque); 
          }     else{                                                  // drive motors backward     
           digitalWrite(InR1, HIGH); 
          digitalWrite(InR2, LOW);  
          digitalWrite(InL1, LOW);  
          digitalWrite(InL2, HIGH); 
            torque = abs(torque);     }       
            analogWrite(PWMR,torque);  
    analogWrite(PWML,torque);  
      } 
      void setup() 
      { 
       pinMode(encoder0PinA, INPUT); 
              // turn on pullup resistor 
       pinMode(encoder0PinB, INPUT); 
            // turn on pullup resistor 
        attachInterrupt(digitalPinToInterrupt(2), doEncoder, RISING); 

      Wire.begin();  
        Serial.begin(115200);             //Terminal Serial
 pinMode(Mag1, OUTPUT);
 pinMode(Mag2, OUTPUT);
 pinMode(InL1, OUTPUT);
 pinMode(InL2, OUTPUT);
 pinMode(PWML, OUTPUT);   
 pinMode(InR1, OUTPUT);
 pinMode(InR2, OUTPUT);
 pinMode(PWMR, OUTPUT); 

      }
  void loop() { 
  
       previoustime=time;
time=micros();
elapsedtime=(time-previoustime)/1000000; 
    newposition0 = encoder0Pos;  
    float rads = (newposition0-oldposition0)*((2*3.14159*0.0325)/(270));
float  x1 = ((newposition0 )*2*pi*0.0325)/270; //displacement
float        x2 = (x1-oldx)/(time/1000000);
    Serial.println(rads);  
   oldx = x1;
         oldposition0 = newposition0;   
  // encoder0Pos=0;
     }
 
void doEncoder ()   { 
if (digitalRead(3)==HIGH){
  encoder0Pos++;
}else{
    encoder0Pos--;
}
}
   
        
      
  
