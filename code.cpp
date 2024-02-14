#include <avr/io.h>
#include <avr/interrupt.h>

const int MOTION_SENSOR_PIN = 2;
const int PIR_BUTTON_PIN = 7;
const int SOIL_MOISTURE_BUTTON_PIN = 8;
const int ULTRASONIC_SIGNAL_PIN = 9; // Ultrasonic sensor signal pin
const int FORCE_SENSOR_PIN = A1;      // Force sensor pin
const int LED_PIN = 3;
const int ANALOG_SENSOR_PIN = 0;

volatile int motionSensorState = 0;
volatile int soilMoistureState = 0;
volatile int ultrasonicState = 0;
volatile int forceSensorValue = 0;
volatile bool sensorsEnabled = true;
volatile bool ledState = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SOIL_MOISTURE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(ULTRASONIC_SIGNAL_PIN, INPUT);
  
  // Attach interrupt for motion sensor pin change
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motionInterrupt, CHANGE);

  // Attach interrupt for soil moisture sensor pin change
  PCMSK0 |= (1 << PCINT0); // Enable PCINT0 for the soil moisture sensor pin
  PCICR |= (1 << PCIE0);   // Enable PCINT0 interrupts

  // Attach interrupt for ultrasonic sensor pin change
  attachInterrupt(digitalPinToInterrupt(ULTRASONIC_SIGNAL_PIN), ultrasonicInterrupt, CHANGE);

  // Start ADC and enable interrupts
  ADCSRA |= (1 << ADEN) | (1 << ADIE); // Enable ADC and ADC interrupt
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128
  ADMUX |= (1 << REFS0); // Set reference voltage to AVCC

  sei(); // Enable global interrupts

  // Configure and enable timer interrupt for LED toggle
  setupTimerInterrupt();

  Serial.begin(9600);
}

void loop() {
  int pirButtonState = digitalRead(PIR_BUTTON_PIN);
  int soilMoistureButtonState = digitalRead(SOIL_MOISTURE_BUTTON_PIN);

  if (pirButtonState == LOW || motionSensorState == HIGH || soilMoistureButtonState == LOW || ultrasonicState == HIGH || forceSensorValue > 500) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Motion detected ");
    sensorsEnabled = false;
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("No motion detected ");
    sensorsEnabled = true;
  }

  if (sensorsEnabled) {
    int level = analogRead(ANALOG_SENSOR_PIN);
    Serial.print("Analog value: ");
    Serial.println(level);

    // Read value from ultrasonic sensor
    ultrasonicState = digitalRead(ULTRASONIC_SIGNAL_PIN);
    Serial.print("Ultrasonic sensor state: ");
    Serial.println(ultrasonicState == HIGH ? "HIGH" : "LOW");
  }
  delay(1000);
}

void motionInterrupt() {
  motionSensorState = digitalRead(MOTION_SENSOR_PIN);
}

void setupTimerInterrupt() {
  // Configure timer1
  TCCR1A = 0; // Reset control register A
  TCCR1B = 0; // Reset control register B
  TCNT1 = 0;  // Reset counter value

  // Set compare match register to desired timer count
  OCR1A = 15624; // = 16MHz / (1024 * 1Hz) - 1, for 1 second interval

  // Configure timer1
  TCCR1B |= (1 << WGM12); // Set WGM12 for CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set prescaler to 1024 and start the timer

  // Enable timer1 compare match interrupt
  TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);
}

ISR(PCINT0_vect) {
  soilMoistureState = digitalRead(ANALOG_SENSOR_PIN);
}

void ultrasonicInterrupt() {
  ultrasonicState = digitalRead(ULTRASONIC_SIGNAL_PIN);
}

ISR(ADC_vect) {
  forceSensorValue = ADC; // Read ADC value from force sensor
}
