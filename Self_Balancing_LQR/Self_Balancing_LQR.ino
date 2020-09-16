/*
 * Author Name: P Sumantha Aithal
 * Functions: motorctrl(int torque),rotary(), ReadData(), setup(), loop()
 * GLobal Variables: Mag1,Mag2,InL1,InL2,PWML,PWMR,InR1,InR2,fcut,pi,compa,alpha,tau,la,acc,gyro,roll
 *                   al,temp,digital,Yaxis,Xaxis,i,PWM,a,elapsedtime,time,previoustime,encoder0PinA,
 *                   encoder0PinB,counter,previous_counter,loop_itteration,K1,K2,K3,K4,x1,x2,x3,x4
 * 
 */
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

#define Mag1 50                    //Electromagnet-1 Pin 
#define Mag2 51                    //Electromagnet-2 Pin
#define InL1 10                    //Left Motor Pin
#define PWML 13                    //Left Motor PWM Pin
#define InL2 9                     //Left Motor Pin
#define InR1 6                     //Right Motor Pin
#define PWMR 4                     //Right Motor PWM Pin  
#define InR2 7                     //Right Motor Pin  
#define fcut 5.0                   //Cut-off frequency
#define pi 3.1415926               //PI
#define compa 0.03                 //a value for complimentary filter

double alpha,tau;                  //alpha and tau
double la[5],acc,gyro,roll,a1[5],temp[5];
int digital,Yaxis,Xaxis;           //Digital,X-axis,Y-axis of Joystick
int i,PWM;
int16_t a[5];
double elapsedtime,time,previoustime;
 
int encoder0PinA = 2;              //Encoder pin
int encoder0PinB = 3;              //Encoder pin
volatile long counter=0;
uint32_t previous_counter=0; 
 
int loop_itteration =0;

float K1=1.0000,    K2= .18317,  K3=-31.79914,   K4=-1.61486; //The K-Matrix from octave

float x1, x2, x3, x4; 
/*
 * Function Name: motorctrl
 * Input: input torque to thr motors
 * Output: Controls the motor
 * Logic: if torque is greater than 0 then bot moves forward else backward
 * Example Call: motorctrl(255)
 */
 
      void motorctrl(int torque){        //torque between 0-255 
        if (torque >= 0)  {                                     
          digitalWrite(InR1, LOW); 
          digitalWrite(InR2, HIGH);  
          digitalWrite(InL1, HIGH);  
          digitalWrite(InL2, LOW);  
          torque = abs(torque); 
          }     else{                         
           digitalWrite(InR1, HIGH); 
          digitalWrite(InR2, LOW);  
          digitalWrite(InL1, LOW);  
          digitalWrite(InL2, HIGH); 
            torque = abs(torque);     }       
            analogWrite(PWMR,torque);  
    analogWrite(PWML,torque); 
    }   
/*
 * Function Name: rotory
 * Input: None
 * Output: Counter value of the encoder
 * Logic: when pin 3 is HIGH the counter is incremented else decremented  
 * Example Call: rotory()
 */
     void rotary() {
      if(digitalRead(3)==HIGH) {   
        counter++; 
        } else {  
          counter--; 
          } 
     }
/*
 * Function Name: setup
 * Input: None
 * Output: Initialization takes place
 * Logic: The instructions inside this executes only once
 * Example Call: setup()
 */     
    void setup() {
 attachInterrupt(digitalPinToInterrupt(encoder0PinA), rotary, RISING);
 pinMode (encoder0PinB,INPUT); 
 pinMode (encoder0PinA,INPUT); 
 Serial1.begin(9600);    
 Serial.begin(115200);             //Terminal Serial
 pinMode(Mag1, OUTPUT);
 pinMode(Mag2, OUTPUT);
 pinMode(InL1, OUTPUT);
 pinMode(InL2, OUTPUT);
 pinMode(PWML, OUTPUT);   
 pinMode(InR1, OUTPUT);
 pinMode(InR2, OUTPUT);
 pinMode(PWMR, OUTPUT);
      Wire.begin();
    accelgyro.initialize();  
    delay(500);
    }
/*
 * Function Name: loop
 * Input: NONE
 * Output: PWM signal to control the motors
 * Logic: All the instructions executes repeatedly
 * Example Call: loop()
 */
    void loop() {    
      previoustime=time;
time=micros();
elapsedtime=(time-previoustime)/1000000; //Loop time
if (Serial1.available()>=20)
{
  if (Serial1.read()== 0x7E)
  {
 for (i=0;i<11;i++){
  byte discard=Serial1.read();     //Discarding unwanted data
 }
 digital=Serial1.read();           //Storing required data from Xbee
 int analogMSB1=Serial1.read();
 int analogLSB1=Serial1.read();
 Yaxis=analogLSB1+(analogMSB1*256); //Combining the data from the Xbee
 int analogMSB2=Serial1.read();
 int analogLSB2=Serial1.read();
 Xaxis=analogLSB2+(analogMSB2*256); //Combining the data from the Xbee
 } 
}

ReadData(pi,fcut,elapsedtime) ;
roll=(1-compa)*(roll+la[4]*elapsedtime)+(compa*acc); //Complimentary filter
        x1 = ((counter)*2*pi*0.0325)/270; //displacement
     if (counter>=previous_counter)
     {
        x2 = (((counter-previous_counter)*2*pi*0.0325))/(270*elapsedtime); //Velocity
     }else {
      x2 = -(((-counter+previous_counter)*2*pi*0.0325))/(270*elapsedtime); //Velocity
     }
        x3 = (roll)/57.3; //tilt angle
        x4 = (la[3])/0.0174533; //angular velocity
   float X= (x1*K1+x2*K2+x3*K3+x4*K4);  
  float tiltoutput=constrain(X*(255/12),-254,254); //The PWM value
  motorctrl (tiltoutput); 
  previous_counter=counter;
    }
/*
 * Function Name: ReadData
 * Input: pi,fcut,elapsedtime
 * Output: Filterd values from the gyroscope and accelerometer
 * Logic: the data is read from the sensor and passed to a lowpass and highpass filter
 * Example Call: ReadData(pi,fcut,elapsedtime)
 */
   void ReadData(float Pi, float Fcut, double elapsedtime)
   {
    tau=1/(2*Pi*Fcut);
alpha=tau/(tau+elapsedtime);
    for (i=3;i<=5;i++)
   {
    temp[i]=a1[i]; //Temperory storage
   }
accelgyro.getMotion6(&a[0], &a[1], &a[2], &a[3], &a[4], &a[5]); //Raw Data
   for(i=0;i<=2;i++)
   {
    a1[i]=a[i]/16384.0;
    la[i]=(1-alpha)*a1[i]+alpha*la[i]; // Lowpass filter
   }
   for (i=3;i<=5;i++)
   {
    a1[i]=a[i]/131.0;
    la[i]=(1-alpha)*la[i]+(1-alpha)*(a1[i]-temp[i]); // Highpass filter
   }
   acc=(180*atan(la[0]/abs(la[2])))/pi;
   return;
   }
