// https://rudysarduinoprojects.wordpress.com/2020/09/23/fun-with-arduino-42-railway-crossing-multi-track-two-way/
// Nicu Florica (niq_ro) added 3rd led as in US
// ver.1.a: Nicu Florica (niq_ro) added 3rd led as in US
// ver.1.b: added 4th led (white led) for open gates

#define GATE_SPEED          120 // [ms] lower number is higher servo speed
#define BLINK_SPEED         400 // [ms] smaller number is faster blinking
#define GATE_DELAY         2000 // [ms] time between start blinking and gate closing
#define END_OF_TRAIN_DELAY 2000 // [ms] time to wait before deciding this was the end of the train
#define GATE_OPEN_ANGLE      10
#define GATE_CLOSED_ANGLE    90
#define SERVO_PIN            12
#define LED1_PIN              2
#define LED2_PIN              3
#define LED3_PIN              4  // 3rd led
#define LED4_PIN              5  // white led

#define NUM_SENSORS           4 // two sensors per track, one left and one right of the gate
byte sensor_pin[NUM_SENSORS]  = {8,9,10,11}; // sensor pin numbers

byte state = 1, train_counter, n;
byte led1, led2, blink_enabled;
byte    angle = GATE_OPEN_ANGLE;
byte setpoint = GATE_OPEN_ANGLE;
byte sensor_state[NUM_SENSORS];       // 0 idle, 1 detect arrival, 2 detect departure, 3 detect end of train
byte end_of_train[NUM_SENSORS];       // 0 idle, 1 end of train detected
unsigned long time_to_blink;
unsigned long time_to_close_gate;
unsigned long time_for_servo_step;
unsigned long time_end_of_train[NUM_SENSORS];

#include <Servo.h>
Servo gate_servo;

void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);  
  pinMode(LED4_PIN, OUTPUT);    
  for (byte i = 0; i < NUM_SENSORS; i++) pinMode(sensor_pin[i], INPUT_PULLUP);
  gate_servo.attach(SERVO_PIN);
  gate_servo.write(angle);
  Serial.begin(9600);
  Serial.println("  ");
  Serial.println("Railway Crossing Control Ready");
  Serial.println("original from: https://rudysarduinoprojects.wordpress.com/2020/09/23/fun-with-arduino-42-railway-crossing-multi-track-two-way/");
  Serial.println();
  Serial.println("small changes by Nicu FLORICA (niq_ro)");
  Serial.println();
  Serial.println("Waiting for train");
  for (byte i = 0; i < NUM_SENSORS; i++) sensor_state[i] = 1; // enable sensors for train detection
}

void loop() {

  for (byte i = 0; i < NUM_SENSORS; i++) {
    if(sensor_state[i] == 1) { // detect arrival of new train
      if(!digitalRead(sensor_pin[i])) { // train detected
        train_counter++;
        sensor_state[i] = 0;
        if(i%2) n = i - 1; else n = i + 1;
        sensor_state[n] = 2; // buddy sensor departure detection enabled
        Serial.print("--> Arrival, sensor no.");
        Serial.println(i);
        Serial.print("Trains:    ");
        Serial.println(train_counter);
        Serial.println("  ");
      }
    }
    else if(sensor_state[i] > 1) {
      if(!digitalRead(sensor_pin[i])) { // departure detected
        time_end_of_train[i] = millis() + (unsigned long)END_OF_TRAIN_DELAY;
        if(i%2) n = i - 1; else n = i + 1;
        sensor_state[n] = 1; // buddy sensor enabled again
        if(sensor_state[i] == 2) {
          Serial.print("<-- Departure, sensor no.");
          Serial.println(i);
        }
        sensor_state[i] = 3;
      }
      if(sensor_state[i] == 3) // decide if end of train has passed based on a timer
        if(millis() > time_end_of_train[i]) end_of_train[i] = 1; 
      if(end_of_train[i]) { // this takes care train_counter-- is executed only once
        train_counter--;
        end_of_train[i] = 0;
        sensor_state[i] = 1;
        Serial.print("Trains:    ");
        Serial.println(train_counter);
        Serial.println("  ");
      }
    }
  }

  switch (state) {
    case 1: // gate open, not blinking, waiting for train arrival
      if(train_counter) state = 12;
    break;

    case 12: // train arrival detected, blinking
      Serial.println("Binking started");
      blink_enabled = 1;
      time_to_close_gate = millis() + (unsigned long)GATE_DELAY;
      state = 2;
    break;
      
    case 2: // blinking, wait until it's time to close the gate
      if (millis() > time_to_close_gate) state = 23; // gate delay time has passed
    break;

    case 23: // close the gate
      Serial.println("Gate closing");
      gate_servo.attach(SERVO_PIN);
      setpoint = GATE_CLOSED_ANGLE;
      state = 3;
    break;

    case 3: // gate is closing, blinking
      if(angle == setpoint) {
        Serial.println("Gate closed");
        state = 4;
      }
    break;
    
    case 4: // gate fullly closed, blinking, waiting for train departure
      if(train_counter == 0) state = 45;
    break;

    case 45: // train departure detected, open the gate, blinking
      Serial.println("Gate opening");
      gate_servo.attach(SERVO_PIN);
      setpoint = GATE_OPEN_ANGLE;        
      state = 5;
    break;

    case 5: // wait until gate is fully opened, blinking
      if(train_counter) state = 23;
      if (angle == setpoint) state = 51;
    break;

    case 51: // gate is fully opened, stop blinking
//      for (byte i = 0; i < NUM_SENSORS; i++) sensor_state[i] = 1;
      blink_enabled = 0;
      led1 = 0;
      led2 = 0;
      gate_servo.detach(); // to avoid servo flutter
      state = 1; 
      Serial.println("Gate open, blinking stopped");
      Serial.println();
      Serial.println("Waiting for train");
      Serial.println("  ");
    break;
  }

  if (millis() > time_for_servo_step) {
    time_for_servo_step = millis() + (unsigned long)GATE_SPEED;
    if (angle < setpoint) angle++;
    if (angle > setpoint) angle--;
    gate_servo.write(angle);
  }
    
  if(blink_enabled == 1) {
    if(millis() > time_to_blink) {
      time_to_blink = millis() + (unsigned long)BLINK_SPEED;
      led1 = !led1;
      led2 = !led1;
    }
  }

  digitalWrite(LED1_PIN, led1);
  digitalWrite(LED2_PIN, led2);
  digitalWrite(LED3_PIN, (led1+led2)%2);
  digitalWrite(LED4_PIN, (led1+led2+1)%2);
}
