//https://rudysarduinoprojects.wordpress.com/2020/09/23/fun-with-arduino-42-railway-crossing-multi-track-two-way/
// ver.1.a: Nicu Florica (niq_ro) added 3rd led as in US
// ver.1.b: added 4th led (white led)
// ver.1.c - added simple audible warnings
// ver.1.d - added complex bell sounds, thx to mike.osborn.756




// Bell setup - sound sketch courtesy of Michael Smith <michael@hurts.ca> (See: https://playground.arduino.cc/Code/PCMAudio/ )
 
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "crossing_bell_8KHz_mono.h"  // This is file in the same folder as this sketch that contains the sound data (limited to 1-2 secs total sound due to limited Uno memory)
                                      // Sounds must be in C code which can bet obtained via editing in AUdacity and conversion from WAV files using various tools 
                                      //  (e.g. http://colinjs.com/software.htm#t_WAVToCode)
#define SAMPLE_RATE 8000   // 8 KHz sample rate; compromised between quality and limited PROGMEM space on Uno/Nano/etc.
volatile uint16_t sample;
byte lastSample;


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
#define speaker_PIN          11
#define NUM_SENSORS           4 // two sensors per track, one left and one right of the gate
byte sensor_pin[NUM_SENSORS]  = {7,8,9,10}; //{8,9,10,11}; // sensor pin numbers

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

/*
int pitch = 200;
int i = 1;
*/

int ledState = LOW;         // current state of the control variable (as well as the on-board LED as visual confirmation for bell-on state)
// This code called at 8000 Hz to get the next sound sample.
ISR(TIMER1_COMPA_vect) {
    if (sample >= sounddata_length) {
        if (sample == sounddata_length + lastSample) {sample = 0;}
        else {OCR2B = sounddata_length + lastSample - sample;  }
    }
    else {OCR2A = ledState * pgm_read_byte(&sounddata_data[sample]);} 

    ++sample;   }

void startPlayback()
{
    pinMode(speaker_PIN, OUTPUT);

    // Set up Timer 2 to do pulse width modulation on the speaker pin.

    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);

        // Do non-inverting PWM on pin OC2A (p.155)
        // On the Arduino this is pin 11.
        TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0); TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

        // Set initial pulse width to the first sample.
        OCR2A = pgm_read_byte(&sounddata_data[0]);
   
    // Set up Timer 1 to send a sample every interrupt.
    cli();

    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);

    lastSample = pgm_read_byte(&sounddata_data[sounddata_length-1]); sample = 0; sei();
}

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
        gate_servo.detach(); // to avoid servo flutter
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
  digitalWrite(LED4_PIN, (led1+led2+1)%2);  // on in free station (gate open)

/*
  if (((led1+led2)%2 == 1) and (state == 4))
  {     
  pitch = 200 + i *100;
  if (i < 20) tone(speaker_PIN, pitch);
  else noTone(speaker_PIN);
  delay(10); 
  i++;
  if (i > 100) i = 1;
  } 
 noTone(speaker_PIN);
*/
 
}  // end main loop
