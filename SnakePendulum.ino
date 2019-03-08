#include <Arduino.h>

#include <TimerOne.h>

#include <TimerThree.h>

////// Creating and assigning variables ///////////

#define TIMER_US 100000
#define SPS1_TICK_COUNTS 700    //SPS1 = YELLOW PENDULUM
#define SPS2_TICK_COUNTS 1000   //SPS2 = RED PENDULUM
#define SPS1_TIMER_DELAY 50
#define SPS2_TIMER_DELAY 50

volatile long SPS1_tick_counts = SPS1_TICK_COUNTS;
volatile long SPS2_tick_counts = SPS2_TICK_COUNTS;
volatile long SPS1_timer_delay = SPS1_TIMER_DELAY;
volatile long SPS2_timer_delay = SPS2_TIMER_DELAY;

#define SPS1_BUTTON_RED 8
#define SPS1_BUTTON_GREEN 23
#define SPS1_BUTTON 25
#define SPS1_PISTON 10

#define SPS2_BUTTON_RED 26
#define SPS2_BUTTON_GREEN 27
#define SPS2_BUTTON 29
#define SPS2_PISTON 9 

#define OFF 0
#define ON 1
#define WAITING 2

bool SPS1State = LOW;
bool SPS2State = LOW;

bool SPS1_PistonState = LOW;
bool SPS2_PistonState = LOW;
bool timer_expired_70s = true;
bool timer_expired_100s = true;
bool SPS1delay_5s_expired = true;
bool SPS2delay_5s_expired = true;

//variables for keeping track of the current time in serial monitor//
int SPS1_Counter = 0;  //SPS1
int SPS2_Counter = 0;  //SPS2
int SPS1_delay_counter;  //SPS1 delay
int SPS2_delay_counter;  //SPS2 delay

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

  ///////// Initialize Button Lights and Piston //////////

  digitalWrite(SPS1_BUTTON_RED, LOW); //SPS1State initialized as low.
  digitalWrite(SPS1_BUTTON_GREEN, HIGH); //sets button light green
  digitalWrite(SPS2_BUTTON_RED, LOW); //SPS2State initialized as low.
  digitalWrite(SPS2_BUTTON_GREEN, HIGH); //sets button light green
  digitalWrite(SPS1_PISTON, LOW);
  digitalWrite(SPS2_PISTON, LOW);

  startTime = millis();   //millis() grabs the current time in ms
}

void loop() {
  ////////// Non-Latching Buttons require Button States /////////
  if (millis() - startTime > 100) {     //every 100 ms this loop will run
    updateSPS1State();
    updateSPS2State();
    startTime = millis();
  }
}

void SPS1_timerIsr()
{
  Serial.print("[debug] time for SPS1 ");     //prints each 0.1 second, e.g. at "[debug] time for SPS1 70", 70 seconds have passed.
  Serial.println(SPS1_Counter);
  SPS1_Counter++;

  if (!(--SPS1_tick_counts))                             // Count to 70S
  {
    SPS1_tick_counts = SPS1_TICK_COUNTS;                      // Reload
    timer_expired_70s = true;                                 // this is going to trigger the control flow world to go to next state
  }
}

void SPS1_timerDelay()
{
  Serial.print("[debug] delay for SPS1 ");     //prints each 0.1 second, e.g. at "[debug] delay for SPS1 50", 5 seconds have passed.
  Serial.println(SPS1_delay_counter);
  SPS1_delay_counter++;

  if (!(--SPS1_timer_delay))                             // Count to 5S
  {
    Serial.println(SPS1_timer_delay);
    SPS1_timer_delay = SPS1_TIMER_DELAY;                      // Reload
    SPS1delay_5s_expired = true;                                // this is going to trigger the control flow world to go to next state
  }
}

void SPS2_timerIsr()
{
  Serial.print("[debug] time for SPS2 ");     //prints each 0.1 second, e.g. at "[debug] time for SPS2 100", 10 seconds have passed.
  Serial.println(SPS2_Counter);
  SPS2_Counter++;
  //Serial.println(SPS2_tick_counts);

  if (!(--SPS2_tick_counts))                             // Count to 100S
  {
    SPS2_tick_counts = SPS2_TICK_COUNTS;                      // Reload
    timer_expired_100s = true;           // this is going to trigger the control flow world to go to next state
  }
}

void SPS2_timerDelay()
{
  Serial.print("[debug] delay for SPS2 ");     //prints each 0.1 second, e.g. at "[debug] delay fOr SPS2 50", 5 seconds have passed.
  Serial.println(SPS2_delay_counter);
  SPS2_delay_counter++;

  if (!(--SPS2_timer_delay))                             // Count to 5S (i.e. at the end of 5S the code will read the inside block)
  {
    SPS2_timer_delay = SPS2_TIMER_DELAY;                      // Reload
    SPS2delay_5s_expired = true;                                   // this is going to trigger the control flow world to go to next state
  }
}

void SPS1_ChangeToWaiting()
{
  turnOffSPS1();  // this resets the counter (for the print outputs in serial monitor), stops the timer, and turns off the piston
  SPS1State = LOW;
}

void SPS1_ChangeToOff()
{
  Serial.println("[debug] SPS1 delay stops");
  turnOffDelaySPS1();  // this stops the delay timer, resets the counter, and changes button to GREEN
}

void SPS2_ChangeToWaiting()
{
  turnOffSPS2();    // this resets the counter (for the print outputs in serial monitor), stops the timer, and turns off the piston
  SPS2State = LOW;
}

void SPS2_ChangeToOff()
{
  Serial.println("[debug] SPS2 delay stops");
  turnOffDelaySPS2();     // this resets the counter (for the print outputs in serial monitor), stops the timer, and turns off the piston
}

void updateSPS1State() {
  static int SPS1_control_state = OFF;
 
  switch (SPS1_control_state) {
    case OFF:    //in OFF state, it will read the button. If the button is pressed, it will turn the piston on, start timer, and change state to ON.
      {
        Serial.println("[debug] SPS1 in OFF state");
        SPS1State = !digitalRead(SPS1_BUTTON); //read button
        Serial.print("[debug] Button SPS1 is ");
        Serial.println(SPS1State);

        if (SPS1State == HIGH) {     //SPS1State is whether SPS1 button is on or off
          turnOnSPS1();  //changes button to RED, pushes piston and starts 70s timer
          SPS1_control_state = ON;
          Serial.println("[debug] set state to ON");
        }
        break;
      }
    case ON:      //in ON state, it will check if the 70s timer has expired. Once it has expired, it will start the delay timer and change state to WAITING. 
      {
        if (timer_expired_70s == true)
        {
          SPS1_ChangeToWaiting();   //this resets the counter, stops the 70s timer, turns off the piston, and sets SPS1State to LOW
          turnOnDelaySPS1();   //this starts the delay timer
          Serial.println("[debug] SPS1 delay timer on");
          SPS1_control_state = WAITING;
        }
        break;
      }
    case WAITING:       //in WAITING, it will change if the 5s delay timer has expired. Once it has expired, it will change button to GREEN and change state to OFF.
      {
        if (SPS1delay_5s_expired == true)
        {
          Serial.println("[debug] SPS1 5s delay timer expired");
          SPS1_ChangeToOff();    //this stops the delay timer, resets the counter, and changes button to GREEN
          SPS1_control_state = OFF;
        }
        break;
      }
  }
}

void turnOnSPS1() {
  //turn button light to RED//
  digitalWrite(SPS1_BUTTON_RED, HIGH);
  digitalWrite(SPS1_BUTTON_GREEN, LOW);

  //turn piston ON//
  Serial.println("[debug] SPS1 on");
  digitalWrite(SPS1_PISTON, HIGH);  
  SPS1_PistonState = !SPS1_PistonState;

  //START 70s timer//
  SPS1_tick_counts = SPS1_TICK_COUNTS;
  SPS1_Counter = 0;
  Serial.println("[debug] SPS1 timer starts");
  Timer1.initialize(TIMER_US);
  timer_expired_70s = false;
  Timer1.attachInterrupt(SPS1_timerIsr);

  //nothing happens when the SPS1 is in action
}

void turnOffSPS1() {
  SPS1_Counter = 0;  //this restarts the print time in serial monitor for SPS1
  SPS1_tick_counts = SPS1_TICK_COUNTS;  //this restarts the timer to SPS1_TICK_COUNTS
  
  //STOP 70s timer//
  Serial.println("[debug] SPS1 timer stops");
  Timer1.stop();

  //turn piston OFF//
  Serial.println("[debug] SPS1 off");
  digitalWrite(SPS1_PISTON, LOW);  
}

void turnOnDelaySPS1() {
  //START 5s delay timer//
  Serial.println("[debug] SPS1 delay on");
  Timer1.initialize(TIMER_US);
  SPS1delay_5s_expired = false;
  SPS1_timer_delay = 50;
  SPS1_delay_counter = 0;
  Timer1.attachInterrupt(SPS1_timerDelay);
}

void turnOffDelaySPS1() {
  SPS1_delay_counter = 0;  //this restarts the print time in serial monitor for SPS2
  SPS1_timer_delay = SPS1_TIMER_DELAY;  //this restarts the motor timer to SPS1_TIMER_DELAY
  
  //STOP 5s delay timer//
  Timer1.stop();

  //turn button light to GREEN//
  digitalWrite(SPS1_BUTTON_GREEN, HIGH);
  digitalWrite(SPS1_BUTTON_RED, LOW);
}


void updateSPS2State() {
  static int SPS2_control_state = OFF;

  switch (SPS2_control_state) {
    case OFF:    //in OFF state, it will read the button. If the button is pressed, it will turn the piston on, start timer, and change state to ON.
      {
        SPS2State = !digitalRead(SPS2_BUTTON);  //read button

        if (SPS2State == HIGH) {  //SPS2State is whether SPS1 button is on or off
          turnOnSPS2();  //changes button to RED, pushes piston and starts 100s timer
          SPS2_control_state = ON;
        }
        break;
      }
    case ON:      //in ON state, it will check if the 100s timer has expired. Once it has expired, it will start the delay timer and change state
      {
        if (timer_expired_100s == true)   
        {
          SPS2_ChangeToWaiting();   //this resets the counter, stops the 100s timer, turns off the piston, and sets SPS1State to LOW
          Serial.println("SPS2 100s timer expired. Start 5 s delay.");
          turnOnDelaySPS2();   //this starts the delay timer
          SPS2_control_state = WAITING;
        }
        break;
      }
    case WAITING:    //in WAITING, it will change if the 5s delay timer has expired. Once it has expired, it will change button to GREEN and change state to OFF.
      {
        if (SPS2delay_5s_expired == true)
        {
          SPS2_ChangeToOff();     //this stops the delay timer, resets the counter, and changes button to GREEN
          SPS2_control_state = OFF;
        }
        break;
      }
  }
}

void turnOnSPS2() {
  //change button light to RED//
  digitalWrite(SPS2_BUTTON_RED, HIGH);
  digitalWrite(SPS2_BUTTON_GREEN, LOW);

  //turn piston ON//
  Serial.println("[debug] SPS2 on");
  digitalWrite(SPS2_PISTON, HIGH);  
  SPS2_PistonState = !SPS2_PistonState;

  //START 100s timer//
  SPS2_tick_counts = SPS2_TICK_COUNTS;
  SPS2_Counter = 0;
  Serial.println("[debug] SPS2 timer starts");
  Timer3.initialize(TIMER_US);
  timer_expired_100s = false;
  Timer3.attachInterrupt(SPS2_timerIsr);

  //nothing happens when the SPS2 is in action
}

void turnOffSPS2() {
  SPS2_Counter = 0;  //this restarts the print time in serial monitor for SPS2
  SPS2_tick_counts = SPS2_TICK_COUNTS;  //this restarts the motor timer to SPS2_TICK_COUNTS
  
  //STOP 100s timer//
  Serial.println("[debug] SPS2 timer stops");
  Timer3.stop();

  //turn piston OFF//
  Serial.println("[debug] SPS2 off");
  digitalWrite(SPS2_PISTON, LOW);  //turn piston off
}

void turnOnDelaySPS2()
{
  //START 5s delay timer//
  Timer3.initialize(TIMER_US);
  SPS2delay_5s_expired = false;
  SPS1_timer_delay = 50;
  SPS2_delay_counter = 0;
  Timer3.attachInterrupt(SPS2_timerDelay);
}

void turnOffDelaySPS2() {
  SPS2_delay_counter = 0;  //this restarts the print time in serial monitor for SPS2
  SPS2_timer_delay = SPS2_TIMER_DELAY;  //this restarts the motor timer to SPS2_TIMER_DELAY
  
  //STOP 5s delay timer//
  Timer3.stop();

  //turn button light to GREEN//
  digitalWrite(SPS2_BUTTON_GREEN, HIGH);
  digitalWrite(SPS2_BUTTON_RED, LOW);
}


