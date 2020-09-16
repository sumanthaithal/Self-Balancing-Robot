#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

#define fcut 5.0
#define pi 3.14156
#define compa 0.03
double alpha,tau;
double la[5],acc,gyro,roll,a1[5],temp[5];
int  i;
int16_t a[5];
double elapsedtime,time,previoustime;

void setup() {
  
 Serial.begin(115200); 
    Wire.begin();
    accelgyro.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
previoustime=time;
time=micros();
elapsedtime=(time-previoustime)/1000000;
tau=1/(2*pi*fcut);
alpha=tau/(tau+elapsedtime);
    for (i=3;i<=5;i++)
   {
    temp[i]=a1[i];
   }
accelgyro.getMotion6(&a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
   for(i=0;i<=2;i++)
   {
    a1[i]=a[i]/16384.0;
    la[i]=(1-alpha)*a1[i]+alpha*la[i];
   }
   for (i=3;i<=5;i++)
   {
    a1[i]=a[i]/131.0;
    la[i]=(1-alpha)*la[i]+(1-alpha)*(a1[i]-temp[i]);
    
   }
   acc=(180*atan(la[0]/abs(la[2])))/pi;
   roll=(1-compa)*(roll+la[4]*elapsedtime)+(compa*acc);
   Serial.print(roll);
   Serial.println(",");
      
}
