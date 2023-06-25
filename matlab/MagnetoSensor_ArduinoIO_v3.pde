//int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
//int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13


int sensorPin = A2;    // select the input pin for the magnetorestrictive sensor
int sensorValue = 0;   // value of the sensor
int offset=0;
int last=0;
int tempSense=0;
int refSensor=0;
int flipcount=0;
int diff=0;
int tempOffset=0;
const int flipswitch = 800;

int motorPin = 11;
int dirPin = 13;
int ledPin=9;

volatile long encCount = 0;
volatile int pin1state;
volatile int pin2state;
//volatile char REGVALS;
int incomingByte = 0;

float supply_voltage = 8.0f;
float control_effort = 0.f;
float serial_input = 0;
unsigned long time;
unsigned long time_last_sent;
float time_f, last_time_f;
boolean output_enable;

void setup()
{
  Serial.begin(115200);
  Serial.flush();
  
  pinMode(motorPin, OUTPUT);
  pinMode(dirPin,OUTPUT); 
  pinMode(ledPin,OUTPUT);
  
  last_time_f = float(micros()); 
  time_f = float(micros());

  output_enable = true;
  
  refSensor = analogRead(sensorPin);
}

void loop()
{

  last=sensorValue - refSensor;
  last=last+511;
  
  sensorValue = analogRead(sensorPin);
   
  tempSense=sensorValue - refSensor;
  tempSense=tempSense+511;
 
 //offset=0;

diff=tempSense-last;
offset=abs(diff);

if (offset > 800)
{
 digitalWrite(ledPin,HIGH);
 
   if (diff>0)
   {
     flipcount--;
   }
   else
   {
     flipcount++;
   }
 
 tempSense=tempSense+flipcount*offset;
 tempOffset=offset;
}
else
{
  digitalWrite(ledPin,LOW);
  tempSense=tempSense+flipcount*tempOffset;
}

//delay(100);
//Serial.println(tempSense);

Serial.write(((uint8_t*)&tempSense),2);
  
  while (!Serial.available()) {
         ; 
       }
       (((int8_t*)(&serial_input))[0]) = Serial.read();
       while (!Serial.available()) {
         ; 
       }
       (((int8_t*)(&serial_input))[1]) = Serial.read();
       while (!Serial.available()) {
         ; 
       }
       (((int8_t*)(&serial_input))[2]) = Serial.read();
       while (!Serial.available()) {
         ; 
       }
       (((int8_t*)(&serial_input))[3]) = Serial.read();
       
 control_effort = serial_input / supply_voltage * 255.f;
  
  if (control_effort > 0) {
  digitalWrite(dirPin,HIGH); 
       } else {
         digitalWrite(dirPin,LOW);
       }
       
        //add friction compensations
       control_effort = abs(control_effort);
       if (control_effort > 255) {
         control_effort = 255.f;
       }
      
       analogWrite(motorPin,uint8_t(abs(control_effort)));
}

