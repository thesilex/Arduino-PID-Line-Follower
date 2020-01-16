//robotcu.com
//Mert ÖZTOPRAK
//mertoztoprak@gmail.com
#include <inttypes.h>
#include <stdlib.h>

//pins
int buttonState = 0;
int buttonPin = 12;
int ledPin = 13;

int CNY1 = A0;
int CNY2 = A1;
int CNY3 = A2;
int CNY4 = A3;
int CNY5 = A4;
int CNY6 = A5;
int CNY7 = A6;
int CNY8 = A7;
int leftPWMPin = 6;
int rightPWMPin = 5;
int rightMotorForwardPin = 4;
int rightMotorBackwardPin = 2;
int leftMotorForwardPin = 7;
int leftMotorBackwardPin = 3;

//variables
long value = 0;
long totalValue = 0;
double division = 0;
double avg = 0;
double sum = 0;
int on_line = 1;
double LightValue = 0;
signed long leftMotor = 0;
signed long rightMotor = 0;
double integral = 0, lastError = 0, error = 0, derivative = 0, output = 0;
//pid coefficients
double Kp = 2;    // If this value is too low, robot can not turn, if it is too high, oscillates a lot
double Kd = 17.5; // dampens oscillation, but if this value is too high, speed gets lower
double Ki = 0;
signed long Tp = 1000; //maximum speed, max limit is 1023.
signed long turn = 0;
int sensor[8];

//calibration variables
int sensorMinCalibration[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int sensorMaxCalibration[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup()
{
    pinMode(buttonPin, INPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(leftPWMPin, OUTPUT);
    pinMode(rightPWMPin, OUTPUT);
    pinMode(rightMotorForwardPin, OUTPUT);
    pinMode(rightMotorBackwardPin, OUTPUT);
    pinMode(leftMotorForwardPin, OUTPUT);
    pinMode(leftMotorBackwardPin, OUTPUT);
}

void calibration() //store minimum and maximum adc values read by each sensor
{
    int i;

    sensor[0] = analogRead(CNY1);
    sensor[1] = analogRead(CNY2);
    sensor[2] = analogRead(CNY3);
    sensor[3] = analogRead(CNY4);
    sensor[4] = analogRead(CNY5);
    sensor[5] = analogRead(CNY6);
    sensor[6] = analogRead(CNY7);
    sensor[7] = analogRead(CNY8);

    for (i = 0; i < 8; i++)
    {
        if (sensor[i] > sensorMaxCalibration[i])
            sensorMaxCalibration[i] = sensor[i];

        if (sensor[i] < sensorMinCalibration[i])
            sensorMinCalibration[i] = sensor[i];
    }
  }

void pid_8sensor()
{
    // If Lightvalue is at 3500, that means robot is just at the center of the line. In that case error should be 0.
    error = (LightValue - 3500);
    
    // integral calculation, usually not useful on line follower robots
    //  integral = integral + error; 
    //  if(integral>60000)
    //  integral=60000;
    //  if(integral<(-60000))
    //  integral=(-60000);

    //  derivative calculation
    derivative = error - lastError;

    //Calculate output of control system using Kp, Kd, Ki coefficients
    output = Kp * error + Kd * derivative + Ki * integral; 
    
    turn = (long) output; //cast double value to long

    if (turn >= Tp) //turn, should not be higher than Tp (maximum speed)
    {
        turn = Tp;
    }
    else if (turn <= (-Tp)) //turn, should not be higher than Tp (maximum speed)
    {
        turn = (-Tp);
    }

    if (turn < 0)
    {
        leftMotor = (long)((Tp + turn));
        rightMotor = Tp;
        analogWrite(leftPWMPin, leftMotor);
        analogWrite(rightPWMPin, rightMotor);
    }
    else
    {
        leftMotor = (long)(Tp);
        rightMotor = Tp - turn;
        analogWrite(leftPWMPin, leftMotor);
        analogWrite(rightPWMPin, rightMotor);
    }
    lastError = error; //save lastError to calculate derivative and integral on next cycle
}

void readLine_8sensor() //read all sensors at once
{
    int i;
    sensor[0] = analogRead(CNY1);
    sensor[1] = analogRead(CNY2);
    sensor[2] = analogRead(CNY3);
    sensor[3] = analogRead(CNY4);
    sensor[4] = analogRead(CNY5);
    sensor[5] = analogRead(CNY6);
    sensor[6] = analogRead(CNY7);
    sensor[7] = analogRead(CNY8);

    //now calculate a single value as an input value for PID
    avg = 0;
    sum = 0;
    on_line = 0;
    totalValue = 0;

    for (i = 0; i < 8; i++)
    {
        if (sensor[i] < sensorMinCalibration[i]) //if sensor value is lower than minimum value we read during calibration
        {
            sensor[i] = 0;
            value = 1023;
        }
        else if (sensor[i] > sensorMaxCalibration[i]) //if sensor value is higher than maximum value we read during calibration
        {
            sensor[i] = 1023;
            value = 0;
        }
        else
        {
            //normalize measurements using weighted average
            division = ((1023) / (double)(sensorMaxCalibration[i] - sensorMinCalibration[i]));
            sensor[i] = (long)((long)(sensor[i] - sensorMinCalibration[i]) * (division));
            value = (1023 - sensor[i]);
        }

        if (value > 300) //if any of the sensor reading is higher than this value,
                         //it means robot is on the line
        {
            on_line = 1;
        }

        avg = (avg + (double)(value) * ((double)(i)*1000)); //calculate weighted average using all sensor values
        sum = sum + (double)value;

        totalValue = totalValue + value;
    }

    if (on_line == 0) //if robot is on the line anymore
    {
        // if robot was turning left last time, keep turning left
        if (LightValue < 3500)
        {
            LightValue = 0;
            totalValue = 0;
            return;
        }
        // if robot was turning right last time, keep turning right
        else
        {
            LightValue = 7000;
            totalValue = 0;
            return;
        }
    }

    LightValue = (double)((avg) / (sum));
    LightValue = (long)LightValue; //calculated LightValue will be between 0-7000
}

void loop()
{
    int i;
    delay(1000);

    //blink leds on powerup
    for (i = 0; i < 10; i++) 
    {
        delay(100);
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
    }
    digitalWrite(ledPin, HIGH);

    buttonState = digitalRead(buttonPin); //read button state
    //Calibration without button, starts on powerup and done after a while
    //  for(i=0;i<3000;i++)
    //  {
    //   delay(1);
    //   calibration();
    //  }
    //Calibration without button

    //Calibration with button
    while (buttonState == LOW); //wait while button is unpressed, calibration should be done first
    while (buttonState == HIGH) //wait while button is pressed, start calibration now, stop if state becomes low again
    {
        buttonState = digitalRead(buttonPin);
        calibration();
    }
    //Calibration with button
    //calibration done

    digitalWrite(ledPin, LOW);
    digitalWrite(leftMotorForwardPin, HIGH);
    digitalWrite(rightMotorForwardPin, HIGH);
    digitalWrite(leftMotorBackwardPin, LOW);
    digitalWrite(rightMotorBackwardPin, LOW);

    //start line following
    while (1)
    {
       readLine_8sensor();
       pid_8sensor();
    }
}
