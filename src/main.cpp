#include <Arduino.h>
#include <RP2040_PWM.h>


// Motor Pins
#define R_N  17 // Left  Motor Back
#define R_P  18 // Left Motor Front

#define L_P  19 // Left  Motor Back
#define L_N  20 // Left Motor Front

#define R_PWM_PIN   16 // Right Motor PWM
#define L_PWM_PIN   21 // Left Motor PWM

// Encoder Pins
#define L_enc_A 0
#define L_enc_B 1
#define R_enc_A 2
#define R_enc_B 3

//
#define R_LED_pin 9
#define L_LED_pin 22

// Sonar Pins
#define R_Trigger 6 // Sonar trigger
#define F_Trigger 7 // Sonar trigger
#define L_Trigger 8 // Sonar trigger

#define R_echo    10 // Right Sonar
#define F_echo    11 // Front Sonar
#define L_echo    12 // Left Sonar 

#define S1_PIN     15

// Motion States
#define STOPPED                 0
#define FINDING_WALL            1
#define FOLLOWING_RIGHT_WALL    2
#define FOLLOWING_LEFT_WALL     3
#define TURNING_LEFT            4
#define TURNING_RIGHT           5
#define BACKWARDS               6

// Finding wall state sub-states
#define IDLE                    0
#define FINDING                 1
#define CLOSER_TO_RIGHT         4
#define CLOSER_TO_LEFT          5

// LED states
#define blink_0 0
#define blink_1 1
#define OFF 2



#define M_PI 3.14159265358979323846

RP2040_PWM* R_PWM;
RP2040_PWM* L_PWM;

int S1, prevS1, stable_distance, max_F_dist, default_speed;

int frequency = 20000;
float Wheel_Diameter = 6.4;
float distance_between_wheels = 13.0;
float R_Kd, R_Ki, R_Kp, L_Kd, L_Ki, L_Kp, max_correction;


typedef struct {
    unsigned long tis, tes;
    int state, new_state, prev_state;
} fsm_t;

fsm_t cycle, motion, wall_finder, L_LED, R_LED;

typedef struct {
  volatile float start, end, dt, speed, mean_speed, speed_sum;
  volatile int pulse_count, calls;
} encoder_t;

encoder_t R_encoder, L_encoder;

// Motors struct
typedef struct {
  int speed;
  float error, last_error, P, I, D, correction;
} motors_t;

motors_t R_motor, L_motor;

typedef struct
{
    volatile unsigned long flight_time, time_start;
    volatile float distance;
} sonar_t;

sonar_t R_Sonar, L_Sonar, F_Sonar;

// Sonar interrupts to calculate distances
void FSonar_Interrupt() 
{
  int pin = digitalRead(F_echo);
  if (pin) {
    F_Sonar.time_start = micros();
  } if (!pin) {
    F_Sonar.flight_time = micros () - F_Sonar.time_start;
    F_Sonar.distance = float(F_Sonar.flight_time / 2.0 / 29.0);
  }
  if (F_Sonar.distance > 200) F_Sonar.distance = 200; // Can't exceed 2m.
  if (F_Sonar.distance < 2) F_Sonar.distance = 2;
} 

void RSonar_Interrupt() 
{
  int pin = digitalRead(R_echo);
  if (pin) {
    R_Sonar.time_start = micros();
  } else if (!pin) {
    R_Sonar.flight_time = micros () - R_Sonar.time_start;
    R_Sonar.distance =     sqrt(2) / 2 * float(R_Sonar.flight_time / 2.0 / 29.0) ; // pythagoras
  }
  if (R_Sonar.distance > 200) R_Sonar.distance = 200; // Can't exceed 2m.
  if (R_Sonar.distance < 2) R_Sonar.distance = 2;     // Minimum of 2cm.
  
} 

void LSonar_Interrupt() 
{
  int pin = digitalRead(L_echo);
  if (pin) {
    L_Sonar.time_start = micros();
  } else if (!pin) {
    L_Sonar.flight_time = micros () - L_Sonar.time_start;
    L_Sonar.distance =  sqrt(2) / 2 * float(L_Sonar.flight_time / 2.0 / 29.0); // pythagoras
  }
  if (L_Sonar.distance > 200) L_Sonar.distance = 200; // Can't exceed 2m.
  if (L_Sonar.distance < 2) L_Sonar.distance = 2;     // Minimum of 2cm
} 

// Send trigger to given sonar
void sendTriggerPulse(int trigger_pin) 
{
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
}

// Set the outputs for the motors controling pins.
void MoveWheels()
{   
    if (L_motor.speed > 95) L_motor.speed = 95;
    if (R_motor.speed > 95) R_motor.speed = 95;
    digitalWrite(L_P, (L_motor.speed >= 0));
    digitalWrite(L_N, (L_motor.speed < 0));

    digitalWrite(R_N, (R_motor.speed < 0));
    digitalWrite(R_P, (R_motor.speed >= 0));

    L_PWM-> setPWM(L_PWM_PIN, frequency, abs(L_motor.speed));
    R_PWM-> setPWM(R_PWM_PIN, frequency, abs(R_motor.speed));
}

void set_state(fsm_t &fsm, int new_state)
{
    if (fsm.state != new_state)
    { // if the state changed tis is reset
        fsm.state = new_state;
        fsm.tes = millis();
        fsm.tis = 0;
    }
    // sets new time in state
    fsm.tis = millis() - fsm.tes;
}

void R_enc_rising_edge() {
    int pin = digitalRead(R_enc_A);
    if (pin)  {
        R_encoder.pulse_count ++;
        R_encoder.start = micros();
    } 
    
    if (!pin) {
        R_encoder.dt =  micros() - R_encoder.start;
        R_encoder.speed = 100000.0*(M_PI * Wheel_Diameter / 960.0) / R_encoder.dt;
        R_encoder.calls ++;
        R_encoder.speed_sum += R_encoder.speed;
    }
}

void L_enc_rising_edge() {
    int pin = digitalRead(L_enc_A);
    if (pin){
        L_encoder.pulse_count ++;
        L_encoder.start = micros();
    }
    else if (!pin) {
        L_encoder.dt = micros() - L_encoder.start;
        L_encoder.speed = 100000.0*(M_PI * Wheel_Diameter / 960.0) / L_encoder.dt; 
        L_encoder.calls ++;
        L_encoder.speed_sum += L_encoder.speed;
    }
}


void init_PID(){
  L_Kp = 1.5; L_Ki = 0.00001; L_Kd = 0.00001; L_motor.I = 0.0; L_motor.D = 0.0; L_motor.error = 0.0; L_motor.last_error = 0.0;
  R_Kp = 1.5; R_Ki = 0.00001; R_Kd = 0.00001; R_motor.I = 0.0; R_motor.D = 0.0; R_motor.error = 0.0; R_motor.last_error = 0.0;
}


void L_stabilize() {
    L_motor.error = L_Sonar.distance - stable_distance;

    L_motor.I = (L_motor.error + L_motor.I);

    if(L_motor.I > 100){
        L_motor.I = 100;
    }
    else if(L_motor.I < -100) {
      L_motor.I = -100;
    }

    L_motor.D = (L_motor.error - L_motor.last_error);
    L_motor.correction = L_motor.error * L_Kp + L_motor.I * L_Ki + L_motor.D * L_Kd;

    if(L_motor.correction  > max_correction){
        L_motor.correction = max_correction;
    }
    else if(L_motor.correction < -max_correction){
        L_motor.correction = -max_correction; 
    }

    R_motor.speed = default_speed + L_motor.correction;
    L_motor.speed = default_speed - L_motor.correction * 1.1;

    L_motor.last_error = L_motor.error;
}

void R_stabilize() {
  Serial.print(" Stabilizing to Right wall. ");
  R_motor.error = R_Sonar.distance - stable_distance;
  R_motor.I = (R_motor.error + R_motor.I);

  if(R_motor.I > 100){
      R_motor.I = 100;
  } else if(R_motor.I < -100){
      R_motor.I = -100;
  }

  R_motor.D = (R_motor.error - R_motor.last_error);
  R_motor.correction = R_motor.error * R_Kp + R_motor.I * R_Ki + R_motor.D * R_Kd;

  if(R_motor.correction  > max_correction){
      R_motor.correction = max_correction;
  }
  else if(R_motor.correction < -max_correction){
      R_motor.correction = -max_correction; 
  }

  L_motor.speed = default_speed + R_motor.correction;
  R_motor.speed = default_speed - R_motor.correction;

  R_motor.last_error = R_motor.error;
    
}

void find_wall() {

    if (R_Sonar.distance < L_Sonar.distance && R_Sonar.distance < F_Sonar.distance)
    {
      Serial.print(" Wall finder says robot is closer to RIGHT wall");
      wall_finder.new_state = CLOSER_TO_RIGHT;      
    }

    else if (L_Sonar.distance < R_Sonar.distance && L_Sonar.distance < F_Sonar.distance)
    {
      Serial.print(" Wall finder says robot is closer to LEFT wall");
      wall_finder.new_state = CLOSER_TO_LEFT;      
    }
}

void set_motion_state() {
  if (wall_finder.state == CLOSER_TO_RIGHT && F_Sonar.distance < max_F_dist) 
  { 
    Serial.print(" Following RIGHT wall after LEFT TURN. ");
    motion.new_state = TURNING_LEFT;
  } 
  else if (wall_finder.state == CLOSER_TO_LEFT && F_Sonar.distance < max_F_dist) 
  { 
    Serial.print(" Following LEFT wall after RIGHT TURN. ");
    motion.new_state = TURNING_RIGHT;
  }
  else if (wall_finder.state == CLOSER_TO_LEFT) 
  {
    Serial.print(" Following LEFT wall. ");
    motion.new_state = FOLLOWING_LEFT_WALL;    
  }
  else if (wall_finder.state == CLOSER_TO_RIGHT) 
  {
    Serial.print(" Following RIGHT wall. ");
    motion.new_state = FOLLOWING_RIGHT_WALL;
  }  
}



int x_dg_turn (int deg) {
    
    float arco = 2 * M_PI * distance_between_wheels * deg / 360;
    float wheel_perimeter =  M_PI * Wheel_Diameter;
    
    int ticks_to_turn = int(960.0 * arco / wheel_perimeter);
    
    return ticks_to_turn;
} 

void work_encoders() {
  if (motion.state != TURNING_LEFT && motion.state != TURNING_RIGHT ){
    motion.prev_state = motion.state;
  }
  // Following wall resets
  if ((motion.state == FOLLOWING_LEFT_WALL || motion.state == FOLLOWING_RIGHT_WALL) && R_encoder.pulse_count > 960) {
      R_encoder.pulse_count = 0;
  }

  else if ((motion.state == FOLLOWING_LEFT_WALL || motion.state == FOLLOWING_RIGHT_WALL) && L_encoder.pulse_count > 960) {
      L_encoder.pulse_count = 0;
  }

  //Turning 90º resets
  else if (motion.state == TURNING_LEFT && (R_encoder.pulse_count > x_dg_turn(90) || F_Sonar.distance > max_F_dist)) {
          R_encoder.pulse_count = 0;
          motion.new_state = motion.prev_state;
          R_motor.speed = default_speed;
          L_motor.speed = default_speed;
          L_LED.new_state = OFF;
      }
  else if (motion.state == TURNING_RIGHT && (L_encoder.pulse_count > x_dg_turn(90) || F_Sonar.distance > max_F_dist)) {
          L_encoder.pulse_count = 0;
          motion.new_state = motion.prev_state;
          R_motor.speed = default_speed;
          L_motor.speed = default_speed;
          R_LED.new_state = OFF;
    }
}

void turn(char orientation) {
  if (orientation == 'R') {
    R_motor.speed = 0;
    L_motor.speed = default_speed;
    R_LED.new_state = blink_0;
  }

  else if (orientation == 'L') {
    R_motor.speed = default_speed;
    L_motor.speed = 0;
    L_LED.new_state = blink_0;
  }

  if (L_LED.state != OFF && L_LED.tis > 100){
    L_LED.new_state = !L_LED.state;
  } 

  if (R_LED.state != OFF && R_LED.tis > 100){
    R_LED.new_state = !R_LED.state;
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(R_P, OUTPUT);
  pinMode(R_N, OUTPUT);
  pinMode(L_P, OUTPUT);
  pinMode(L_N, OUTPUT);
  pinMode(R_PWM_PIN,  OUTPUT);
  pinMode(L_PWM_PIN,  OUTPUT);
  pinMode(L_Trigger, OUTPUT);
  pinMode(R_Trigger, OUTPUT);
  pinMode(F_Trigger, OUTPUT);
  pinMode(L_LED_pin, OUTPUT);
  pinMode(R_LED_pin, OUTPUT);
  pinMode(R_echo, INPUT_PULLUP);
  pinMode(L_echo, INPUT_PULLUP);
  pinMode(F_echo, INPUT_PULLUP);
  pinMode(S1_PIN, INPUT_PULLUP);

  R_PWM = new RP2040_PWM(R_PWM_PIN, 20000, 0);
  L_PWM = new RP2040_PWM(L_PWM_PIN, 20000, 0);

  attachInterrupt(R_echo, RSonar_Interrupt , CHANGE);
  attachInterrupt(L_echo, LSonar_Interrupt, CHANGE);
  attachInterrupt(F_echo, FSonar_Interrupt, CHANGE);
  attachInterrupt(R_enc_A, R_enc_rising_edge, CHANGE);
  attachInterrupt(L_enc_A, L_enc_rising_edge, CHANGE);

  set_state(cycle, 0);
  set_state(motion, STOPPED);
  set_state(wall_finder, IDLE);
  set_state(L_LED, OFF);
  set_state(R_LED, OFF);
  init_PID();

  stable_distance = 15;
  max_F_dist = 30;
  default_speed = 85;

  R_motor.speed = 0;
  L_motor.speed = 0;
  max_correction = 40;
}

void loop()
{
    work_encoders();
    prevS1 = S1;
    S1 = !digitalRead(S1_PIN);

    if (!prevS1 && S1) {
        Serial.println(" BUTTON PRESSED ");
        if (motion.state != STOPPED) {
            R_motor.speed = 0;
            L_motor.speed = 0;
            init_PID();
            motion.new_state = STOPPED;
            L_LED.new_state = OFF;
            R_LED.new_state = OFF;
        } else if (motion.state == STOPPED){
            motion.new_state = FINDING_WALL;
            wall_finder.new_state = FINDING;
            L_LED.new_state = OFF;
            R_LED.new_state = OFF;
        }
    }

  if (cycle.tis > 20) {

    cycle.new_state = !cycle.state;

    if (motion.state == FINDING_WALL)
    {
      wall_finder.new_state = FINDING;
    } else wall_finder.new_state = IDLE;
    
    sendTriggerPulse(F_Trigger);
    sendTriggerPulse(R_Trigger);
    sendTriggerPulse(L_Trigger);

    // 1st find closest wall
    if (wall_finder.state == FINDING) find_wall();

    // Closest wall has been found, set motion
    if (motion.state == FINDING_WALL && wall_finder.state != FINDING && wall_finder.state != IDLE) set_motion_state();


    // Following wall events

    // Following right wall.
    if (motion.state == FOLLOWING_RIGHT_WALL && F_Sonar.distance > max_F_dist) { // Normal follow
      R_stabilize();
    } else if (motion.state == FOLLOWING_RIGHT_WALL && F_Sonar.distance < max_F_dist) { // A Wall / corner appears
      motion.new_state = TURNING_LEFT;
      R_encoder.pulse_count = 0;
      L_encoder.pulse_count = 0;
    } 

    // Following left wall.
    if (motion.state == FOLLOWING_LEFT_WALL && F_Sonar.distance > max_F_dist) {
      L_stabilize();
    } else if (motion.state == FOLLOWING_LEFT_WALL && F_Sonar.distance < max_F_dist) {
      motion.new_state = TURNING_RIGHT;
      R_encoder.pulse_count = 0;
      L_encoder.pulse_count = 0;
    } 

    // Turning events
    if (motion.state == TURNING_RIGHT) turn('R');
    if (motion.state == TURNING_LEFT)  turn('L');

    // Debug prints
    Serial.print(" Motion: ");
    Serial.print(motion.state);
    Serial.print(" Wall: ");
    Serial.print(wall_finder.state);
    Serial.print(" L: ");
    Serial.print(L_Sonar.distance);
    Serial.print(" F: ");
    Serial.print(F_Sonar.distance);
    Serial.print(" R: ");
    Serial.print(R_Sonar.distance);
    Serial.print(" R speed: ");
    Serial.print(R_motor.speed);
    Serial.print(" L speed: ");
    Serial.print(L_motor.speed);
    Serial.print(" D: ");
    //Serial.print(R_motor.D);
    Serial.print(" R Correction: ");
    Serial.print(R_motor.correction);
    Serial.print(" L Correction: ");
    Serial.println(L_LED.state);
  } 



  digitalWrite(L_LED_pin, (L_LED.state == blink_1));
  digitalWrite(R_LED_pin, (R_LED.state == blink_1));

  MoveWheels();

  set_state(cycle, cycle.new_state);
  set_state(wall_finder, wall_finder.new_state);
  set_state(motion, motion.new_state);
  set_state(L_LED, L_LED.new_state);
  set_state(R_LED, R_LED.new_state);

}

