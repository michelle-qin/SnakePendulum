#include <Arduino.h>

#include <TimerOne.h>

#include <TimerThree.h>

////// Creating and assigning variables ///////////

#define TIMER_US 100000
#define TICK_COUNTS 100    //SPS1  -- check, not sure how long to keep timer for 
#define TICK_COUNTS1 200   //SPS2   

volatile long tick_count = TICK_COUNTS;
volatile long tick_count1 = TICK_COUNTS1;
volatile bool in_long_isr = false;

#define SPS1_BUTTON_RED 23
#define SPS1_BUTTON_GREEN 8
#define SPS1_BUTTON 25
#define SPS1_PISTON 10

#define SPS2_BUTTON_RED 27
#define SPS2_BUTTON_GREEN 26
#define SPS2_BUTTON 29
#define SPS2_PISTON 9   //check that this piston pin matches up with button pins for SPS2

bool SPS1State = LOW;
bool lastSPS1State = LOW;
bool SPS1Count = false;

bool SPS2State = LOW;
bool lastSPS2State = LOW;
bool SPS2Count = false;

bool pistonState = LOW;
bool piston1State = LOW;
bool timerActive = false;
bool timer1Active = false;

//variables for keeping track of the current time in serial monitor//
int i = 0;  //SPS1
int g = 0;  //SPS2

unsigned long startTime;

void setup() {
  ////// Pinmode all ports ///////////
  Serial.begin(9600);

  pinMode(SPS1_BUTTON_RED, OUTPUT);
  pinMode(SPS1_BUTTON_GREEN, OUTPUT);
  pinMode(SPS1_BUTTON, INPUT_PULLUP);
  pinMode(SPS1_PISTON, OUTPUT);

  pinMode(SPS2_BUTTON_RED, OUTPUT);
  pinMode(SPS2_BUTTON_GREEN, OUTPUT);
  pinMode(SPS2_BUTTON, INPUT_PULLUP);
  pinMode(SPS2_PISTON, OUTPUT);

  ///////// Initialize Button Lights //////////

  digitalWrite(SPS1_BUTTON_RED, LOW); //SPS1State initialized as low.
  digitalWrite(SPS1_BUTTON_GREEN, HIGH); //sets button light green
  digitalWrite(SPS2_BUTTON_RED, LOW); //SPS2State initialized as low.
  digitalWrite(SPS2_BUTTON_GREEN, HIGH); //sets button light green
  digitalWrite(SPS1_PISTON, LOW);
  digitalWrite(SPS2_PISTON, LOW);

  startTime = millis();
}

void loop() {

  ////////// Non-Latching Buttons require Button States /////////

  if (millis() - startTime > 100) {
    updateSPS1State();
    updateSPS2State();
    startTime = millis();
  }
}

void timerIsr()
{
  Serial.print("[debug] time for SPS1 ");     //prints each 0.1 second, e.g. at "[debug] time 100", 10 seconds have passed.
  Serial.println(i);
  i++;

  timerActive = true;

  if (!(--tick_count))                             // Count to 10S
  {
    tick_count = TICK_COUNTS;                      // Reload
    tick_SPS1_isr();                                 // Call the beam routine
  }

}

void timerIsr1()
{
  Serial.print("[debug] time for SPS2 ");     //prints each 0.1 second, e.g. at "[debug] time 100", 10 seconds have passed.
  Serial.println(g);
  g++;

  timer1Active = true;

  if (!(--tick_count1))                             // Count to 20S
  {
    tick_count1 = TICK_COUNTS1;                      // Reload
    tick_SPS2_isr();                                 // Call the beam routine
  }

}

void tick_SPS1_isr()
{
  if (in_long_isr)
    return;

  in_long_isr = true;

  volatile long i;

  interrupts();

  timerActive = false;
  turnOffSPS1();
  SPS1State = LOW;
  SPS1Count = false;

  noInterrupts();
  in_long_isr = false;
}

void tick_SPS2_isr()
{
  if (in_long_isr)
    return;

  in_long_isr = true;

  volatile long i;

  interrupts();

  timer1Active = false;
  turnOffSPS2();
  SPS2State = LOW;
  SPS2Count = false;

  noInterrupts();
  in_long_isr = false;
}

void updateSPS1State() {
  Serial.println(SPS1State);
  SPS1State = !digitalRead(SPS1_BUTTON);
  Serial.println(SPS1State);

  if (SPS1State != lastSPS1State && SPS1State == HIGH) {     //SPS1State is whether SPS1 button is on or off
    SPS1Count = !SPS1Count;   //if SPS1State is HIGH, make SPS1Count TRUE

    if (SPS1Count == false) {
      turnOffSPS1();
    }
    else {
      turnOnSPS1();
    }
  }

  lastSPS1State = SPS1State;
}

void turnOffSPS1() {
  if (timerActive != true)
  {
    digitalWrite(SPS1_BUTTON_GREEN, HIGH);
    digitalWrite(SPS1_BUTTON_RED, LOW);

    Serial.println("[debug] SPS1 off");
    digitalWrite(SPS1_PISTON, LOW);  //turn piston off

    i = 0;  //this restarts the print time in serial monitor for SPS1
    tick_count = TICK_COUNTS;  //this restarts the beam timer to TICK_COUNTS
    //stop timer//
    Serial.println("[debug] SPS1 timer stops");
    Timer1.stop();
    //reset timer//
  }

}

void turnOnSPS1() {
  if (timerActive != true)
  {
    //turn beam light on//
    digitalWrite(SPS1_BUTTON_RED, HIGH);
    digitalWrite(SPS1_BUTTON_GREEN, LOW);

    Serial.println("[debug] SPS1 on");
    Serial.print("PISTON: ");
    Serial.println(pistonState);
    digitalWrite(SPS1_PISTON, HIGH);  //moves piston
    pistonState = !pistonState;
    Serial.print("PISTON: ");
    Serial.println(pistonState);

    tick_count = TICK_COUNTS;
    i = 0;
    //start timer//
    Serial.println("[debug] SPS1 timer starts");
    Timer1.initialize(TIMER_US);
    Timer1.attachInterrupt(timerIsr);
    //reset timers//

    //nothing happens when the SPS1 is in action
  }
}

void updateSPS2State() {
  Serial.println(SPS2State);
  SPS2State = !digitalRead(SPS2_BUTTON);
  Serial.println(SPS2State);

  if (SPS2State != lastSPS2State && SPS2State == HIGH) {     //SPS2State is whether SPS1 button is on or off
    SPS2Count = !SPS2Count;   //if SPS2State is HIGH, make SPS2Count TRUE

    if (SPS2Count == false) {
      turnOffSPS2();
    }
    else {
      turnOnSPS2();
    }
  }

  lastSPS2State = SPS2State;
}

void turnOffSPS2() {
  if (timer1Active != true)
  {
    digitalWrite(SPS2_BUTTON_GREEN, HIGH);
    digitalWrite(SPS2_BUTTON_RED, LOW);

    Serial.println("[debug] SPS2 off");
    digitalWrite(SPS2_PISTON, LOW);  //turn piston off

    g = 0;  //this restarts the print time in serial monitor for SPS2
    tick_count1 = TICK_COUNTS1;  //this restarts the motor timer to TICK_COUNTS1
    //stop timer//
    Serial.println("[debug] SPS2 timer stops");
    Timer3.stop();
    //reset timer//
  }
}

void turnOnSPS2 () {
  if (timer1Active != true)
  {
    digitalWrite(SPS2_BUTTON_RED, HIGH);
    digitalWrite(SPS2_BUTTON_GREEN, LOW);

    Serial.println("[debug] SPS2 on");
    Serial.print("PISTON: ");
    Serial.println(piston1State);
    digitalWrite(SPS2_PISTON, HIGH);  //moves piston
    piston1State = !piston1State;
    Serial.print("PISTON: ");
    Serial.println(piston1State);

    tick_count1 = TICK_COUNTS1;
    g = 0;
    //start timer//
    Serial.println("[debug] SPS2 timer starts");
    Timer3.initialize(TIMER_US);
    Timer3.attachInterrupt(timerIsr1);
    //reset timers//

    //nothing happens when the SPS2 is in action
  }
}

