#define echo 8
#define trig 9
#define temp A0
#define gas  A1
#define light A2
#define motion A3

// LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(13,12,11,10,7,6);

int pirSensor = 0; //initial value set to 0

void setup()
{
  pinMode(trig, OUTPUT);
  pinMode(echo,INPUT);
  pinMode(light, INPUT);
  pinMode(motion, INPUT);
  
  //lEFT MOTOR
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  
  //RIGHT MOTOR
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  
  //LCD
  lcd.begin(16, 2);
  Serial.begin(9600);
}

void temp_reading()
{
	 int reading = analogRead(temp);
     float voltage = reading * 4.68;
     voltage /= 1024.0;
 
     float tempC = (voltage - 0.5) * 100;
     Serial.print(tempC);
     Serial.print(" degrees C, ");
}

void gas_reading()
{
	int reading = analogRead(gas);
    int b =  map(reading,0,1023,0,255);
    Serial.print(b);
  	Serial.print("% Gas, ");

}

void light_level()
{
	int reading = analogRead(light);
    int light_level = 100 - (reading * 0.097);
    Serial.print(light_level);
  	Serial.print("% Light, ");

}

void motion_detect()
{
  	pirSensor = digitalRead(motion);
  	
  	if (pirSensor == HIGH)
    {
      Serial.println("Motion Detected! ");
    }
  	
  	else 
    {
      Serial.println("No Motion Detected ");
    }
}

//CONTROLLING ROVER MOVEMENT WITH MOTORS
void forward()
{
  lcd.clear();
  lcd.print("MOVING FORWARD");
 
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  
  delay(1000);
}

void backward()
{
  digitalWrite(2,LOW);
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  
  lcd.clear();
  lcd.print("MOVING BACKWARD");
  delay(2500);
}

void stop()
{
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  
  lcd.clear();
  lcd.print("STOP");
  delay(2000);
}

void right()
{
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  
  lcd.clear();
  lcd.print("TURNING RIGHT");
  delay(2500);
}

void left()
{
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  
  lcd.clear();
  lcd.print("TURNING LEFT");
  delay(2500);
}

//MEASURE DISTANCE BETWEEN OBJECT AND ROVER
//USING ULTRASONIC DISTANCE SENSOR
long sense_distance()
{
	long time,distance;
    
  	digitalWrite(trig,LOW);
    delayMicroseconds(2);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW);
    time = pulseIn(echo,HIGH);
    distance = time/2/29.1;
  	return distance;
}

void loop()
{ 
 	long distance = sense_distance();
  	temp_reading();
  	gas_reading();
  	light_level();
    motion_detect();
  	
  	if (distance>=100)
    {
    	forward();
    }
  
  	else
  	{	
      	stop();
      	backward();
      	left();	
  	}
}