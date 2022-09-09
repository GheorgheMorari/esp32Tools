#include <stdint.h>
#include <stdbool.h>
#define PWM_MEASURE_PIN 4
#define CIRCULAR_BUFFER_SIZE 15 //Increase the buffer size for better averages, but it must be filled in within one second.

static uint8_t bufferCounter = 0;
static unsigned long circularBuffer[CIRCULAR_BUFFER_SIZE];

bool startHigh = false;
bool firstTrigger = false;

void recordEntry(){
  if (firstTrigger){
    startHigh = (digitalRead(PWM_MEASURE_PIN) == HIGH);
    firstTrigger = false;
  }
  circularBuffer[bufferCounter++] = micros();
  if (bufferCounter >= CIRCULAR_BUFFER_SIZE){
    bufferCounter %= CIRCULAR_BUFFER_SIZE;
    detachInterrupt(digitalPinToInterrupt(PWM_MEASURE_PIN)); 
  }
}
void setup(){
    Serial.begin(115200);
    pinMode(PWM_MEASURE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PWM_MEASURE_PIN), recordEntry, CHANGE);
}


void loop(){

  unsigned long highTimeSum = 0;
  unsigned long lowTimeSum = 0;
  bool getHigh = startHigh;
  unsigned long * sumVal;
  for (int i = 0; i < CIRCULAR_BUFFER_SIZE-1; i++){
    sumVal = getHigh ? &lowTimeSum : &highTimeSum;
    *sumVal += circularBuffer[i+1] - circularBuffer[i];
    getHigh = !getHigh;
  }
  if (!highTimeSum || !lowTimeSum){
      Serial.print("Found zeros:"); 
      Serial.print("High Time sum microseconds:"); 
      Serial.println(highTimeSum);
      Serial.print("Low Time sum microseconds:"); 
      Serial.println(lowTimeSum); 
      Serial.println(startHigh ? "Started High" : "Started Low");

  } else {
    
  //Convert from microseconds to frequency
  float averageFreq = 500000.0f/((float)(highTimeSum + lowTimeSum)/ (float)CIRCULAR_BUFFER_SIZE); // Count two changes as one event
  float combinedTime = (float)(highTimeSum + lowTimeSum);
  Serial.print("Average High Time microseconds:"); 
  Serial.println((float)highTimeSum / (float)CIRCULAR_BUFFER_SIZE * 2.0f);
  Serial.print("Average Low Time microseconds:"); 
  Serial.println((float)lowTimeSum / (float)CIRCULAR_BUFFER_SIZE * 2.0f); 
  Serial.print("Average freq:"); 
  Serial.println(averageFreq);
  Serial.print("Average duty cycle:");
  Serial.println((float)highTimeSum / combinedTime); 
  Serial.println(startHigh ? "Started High" : "Started Low");
  }

  bufferCounter = 0;
  firstTrigger = true;
  attachInterrupt(digitalPinToInterrupt(PWM_MEASURE_PIN), recordEntry, CHANGE);

  delay(1000);      
}
