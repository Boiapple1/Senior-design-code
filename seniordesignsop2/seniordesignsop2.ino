
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
#include <SoftwareSerial.h>

#define tx1 10
#define rx1 11
#include "SparkFunHTU21D.h"

SoftwareSerial mySerial(tx1, rx1); // RX, TX
HTU21D myHumidity;

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

float a,b,c,d;

void setup()
{
  Serial.begin(115200);   
  Serial.println("Monitor:");

  mySerial.begin(115200);
  mySerial.println("Computer");  // initialize serial communication at 115200 bits per second:

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  myHumidity.begin();
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }


  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32 per fifo sample
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 118; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop(){

  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  float humd = myHumidity.readHumidity();
  float temp = myHumidity.readTemperature();
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  if (mySerial.available())
  
    Serial.write(mySerial.read());
    mySerial.print("SPO2= ");
    mySerial.print(spo2, DEC);
    mySerial.print(" HRvalid= ");
    mySerial.print(validHeartRate, DEC);
    mySerial.print(" HR= ");
    mySerial.print(heartRate); 
    mySerial.print(" SPO2Valid= ");
    mySerial.print(validSPO2, DEC);
    mySerial.print(" Temperature1= ");
    mySerial.print(temp, 1);
    mySerial.print(" Temperature2= ");
    mySerial.print(((temp*(1.8))+32),1);
    mySerial.print(" Humidity= ");
    mySerial.println(humd, 1);
    
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  delay(20000);
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART

      Serial.print(" HR=");
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

  Serial.print("Time:");
  Serial.print(millis());
  Serial.print(" Temperature:");
  Serial.print(temp, 1);
  Serial.print("C");
  Serial.print(" Temperature:");
  Serial.print(((temp*(1.8))+32),1);
  Serial.print("F");
  Serial.print(" Humidity:");
  Serial.print(humd, 1);
  Serial.print("%");


    
  if (mySerial.available())
  
    Serial.write(mySerial.read());
    mySerial.print("SPO2= ");
    mySerial.print(spo2, DEC);
    mySerial.print(" HRvalid= ");
    mySerial.print(validHeartRate, DEC);
    mySerial.print(" HR= ");
    mySerial.print(heartRate); 
    mySerial.print(" SPO2Valid= ");
    mySerial.print(validSPO2, DEC);
    mySerial.print(" Temperature1= ");
    mySerial.print(temp, 1);
    mySerial.print(" Temperature2= ");
    mySerial.print(((temp*(1.8))+32),1);
    mySerial.print(" Humidity= ");
    mySerial.println(humd, 1);
      delay(20000);
  if (Serial.available())
    mySerial.write(Serial.read());

  
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


 
  
}
