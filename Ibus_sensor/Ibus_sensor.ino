#include <IBusBM.h>

#include <Adafruit_BMP085.h>
#include "MedianFilterLib2.h"


IBusBM IBus; 

Adafruit_BMP085 bmp;

#define FILTER_N 8
#define ALPHA 0.9
MedianFilter2<double> medianFilter(FILTER_N);


#define PWM_PIN A0
#define TEMPBASE 400    // base value for 0'C
  
double alt = 0;
double base_alt = 0;
double max_alt = 0;
double d_alt = 0;
uint16_t temp=TEMPBASE+200; // start at 20'C

long old_millis = 0;
double old_alt = -1;


void reset_altitude(){
  // reset base altitude
  base_alt = 0;
  for(int i=0; i<10; i++){
    base_alt += bmp.readAltitude()/10.;
    delay(100);
 
  }  
  alt = base_alt;
  old_alt = alt;
  for(int i=0; i<FILTER_N; i++){
    medianFilter.AddValue(alt);
  }
}

void reset_max_altitude(){
   max_alt = 0; 
}


void setup() {
  // initialize serial port for debug
  SerialUSB.begin(115200);
  
  pinMode(PWM_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
 
  Serial.begin(9600);
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }


  // iBUS connected to serial1
  IBus.begin(Serial1);

  // adding 3 sensors
  IBus.addSensor(IBUSS_TEMP); // temperature
  IBus.addSensor(IBUSS_TEMP);  // altitude
  IBus.addSensor(IBUSS_TEMP);  // max altitude
  IBus.addSensor(IBUSS_TEMP);  // climb rate
  

  reset_altitude();
  old_millis = millis();

}

#define NOT_PRESSED 0
#define JUST_PRESSED 1
#define PRESSED 3
int state = NOT_PRESSED;
long pressed_timer=0;

void update_state(){
   int pwm = pulseIn(PWM_PIN, HIGH);

   if(pwm > 1500){
    if(state == NOT_PRESSED){
      state = JUST_PRESSED;
      pressed_timer = millis();
    }
    else if(state == JUST_PRESSED){
      if(millis() - pressed_timer > 1000){
          state = PRESSED;
          digitalWrite(LED_BUILTIN,HIGH);
          reset_altitude();
          reset_max_altitude();
          digitalWrite(LED_BUILTIN,LOW);
          
       }
      
    }
   } else {
    state = NOT_PRESSED;
   }
  
}

void loop() {
  double alt_t = bmp.readAltitude();
  double median = medianFilter.AddValue(alt_t);
  alt = alt * ALPHA +median * (1-ALPHA);
  /*Serial.print(millis());
  Serial.print(",");
  Serial.print(alt_t);
  Serial.print(",");
  Serial.print(median);
  Serial.print(",");
  Serial.println(alt);*/
  
 
  if (alt> max_alt){
    max_alt = alt;
  }

  double delta = (millis() - old_millis);
  d_alt = (alt - old_alt)/delta*1000;

  
  old_millis = millis();
  old_alt = alt;

  


  uint16_t temp_read = bmp.readTemperature()*10 ;
  temp = TEMPBASE+temp_read;
  IBus.setSensorMeasurement(1,temp); // increase temperature by 0.1 'C every loop
  
  uint16_t alt_read = TEMPBASE+(alt - base_alt)*10;
  IBus.setSensorMeasurement(2,alt_read  );

  uint16_t max_alt_read = TEMPBASE+ (max_alt - base_alt)*10;
  IBus.setSensorMeasurement(3, max_alt_read  );

  uint16_t d_alt_read = TEMPBASE + d_alt*10;
  IBus.setSensorMeasurement(4, d_alt_read);

  update_state();
  
}
