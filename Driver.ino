#include <PID_v1.h>

/** 
 * GLOBAL VARIABLE INITIALIZATION
 */

/* Joint variable structure */
struct joint {
  int servoPin; // Pin to control the power in the motor driver using PWM
  int dir; // Pin to set the direction of rotation in the motor drive
  int encA; // Encoder A pin of the quadrature encoder
  int encB; // Encoder B pin of the quadrature encoder
  volatile long enc_count; // The count variable for the encoder
  double current_angle; // The current angle of the motor shaft in degree
  double target_angle; // The target angle of the motor shaft given as input in degree
  uint8_t enc_val; // current and previous encoder value
  double encoder_factor; // The constant to be multiplied to encoder value to get the position in degrees
  bool forwardDirection; // Sets the direction to turn when positive or negative error occurs
  double pwmSpeed;
};

/* Joint 1 */
joint motor1 = {PB8, PB9, PB6, PB7, 0, 0, 0, 0, 0.005, HIGH, 0};
/* Joint 2 */
joint motor2 = {PA8, PB15, PA9, PA10, 0, 0, 0, 0, 0.005, HIGH, 0};
/* Joint 3 */
joint motor3 = {PA2, PA3, PA1, PA0, 0, 0, 0, 0, 0.0121951219512195, HIGH, 0};
/* Joint 4 */
joint motor4 = {PB0, PA5, PA6, PA7, 0, 0, 0, 0, 0.0121951219512195, HIGH, 0};


/* Initialize PID controllers */
double Kp=2, Ki=5, Kd=1;
PID myPID1(&motor1.current_angle, &motor1.pwmSpeed, &motor1.target_angle, Kp, Ki, Kd, DIRECT);
PID myPID2(&motor2.current_angle, &motor2.pwmSpeed, &motor2.target_angle, Kp, Ki, Kd, DIRECT);
PID myPID3(&motor3.current_angle, &motor3.pwmSpeed, &motor3.target_angle, Kp, Ki, Kd, DIRECT);
PID myPID4(&motor4.current_angle, &motor4.pwmSpeed, &motor4.target_angle, Kp, Ki, Kd, DIRECT);

/* String to decode the input command */
String readStr;

/** 
 * Quadrature ENCODER Interrupt Service Routine
 */

/* Quadrature decoder for PB6 and PB7 pins */
void joint1ISR()
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  motor1.enc_val = motor1.enc_val << 2;
  motor1.enc_val = motor1.enc_val | ((GPIOB->regs->IDR & 0b11000000)>>6);

  motor1.enc_count = motor1.enc_count + lookup_table[motor1.enc_val & 0b1111];
}

/* Quadrature decoder for PA9 and PA10 pins */
void joint2ISR()
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  motor2.enc_val = motor2.enc_val << 2;
  motor2.enc_val = motor2.enc_val | ((GPIOA->regs->IDR & 0b11000000000)>>9);

  motor2.enc_count = motor2.enc_count + lookup_table[motor2.enc_val & 0b1111];
}

/* Quadrature decoder for PA0 and PA1 pins */
void joint3ISR()
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  motor3.enc_val = motor3.enc_val << 2;
  motor3.enc_val = motor3.enc_val | (GPIOA->regs->IDR & 0b11);

  motor3.enc_count = motor3.enc_count + lookup_table[motor3.enc_val & 0b1111];
}

/* Quadrature decoder for PA6 and PA7 pins */
void joint4ISR()
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  motor4.enc_val = motor4.enc_val << 2;
  motor4.enc_val = motor4.enc_val | ((GPIOA->regs->IDR & 0b11000000)>>6);

  motor4.enc_count = motor4.enc_count + lookup_table[motor4.enc_val & 0b1111];
}

void setup()
{
  // Attach pins to interrupt service routine
  attachInterrupt(digitalPinToInterrupt(motor1.encA), joint1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor1.encA), joint1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2.encA), joint2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2.encA), joint2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor3.encA), joint3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor3.encA), joint3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor4.encA), joint4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor4.encA), joint4ISR, CHANGE);

  // Setup all PWM Pins and Direction pins
  pinMode(motor1.servoPin, OUTPUT);
  pinMode(motor1.dir, OUTPUT);
  pinMode(motor2.servoPin, OUTPUT);
  pinMode(motor2.dir, OUTPUT);
  pinMode(motor3.servoPin, OUTPUT);
  pinMode(motor3.dir, OUTPUT);
  pinMode(motor4.servoPin, OUTPUT);
  pinMode(motor4.dir, OUTPUT);

  // Begin Serial Communication on USB port
  Serial.begin(115200);

  // Turn on PID controllers
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);
  myPID4.SetMode(AUTOMATIC);
}

void PIDControl(joint* ob, PID* myPID){ 
  // Set the direction of motor based on the target angle
  int diff;
  diff = (*ob).current_angle - (*ob).target_angle;
  if (diff < 0)
  {
    digitalWrite((*ob).dir, (*ob).forwardDirection);
    diff = diff * -1;
  }
  else
  {
    digitalWrite((*ob).dir, !(*ob).forwardDirection);
  }

  // If the target and current angles are not equal power the motor using PID controller
  (*myPID).Compute();
  analogWrite((*ob).servoPin, (*ob).pwmSpeed);
}

void PWMControl(joint* ob){
  // Set the direction of motor based on the target angle
  int diff;
  diff = (*ob).current_angle - (*ob).target_angle;
  if (diff < 0)
  {
    digitalWrite((*ob).dir, (*ob).forwardDirection);
    diff = diff * -1;
  }
  else
  {
    digitalWrite((*ob).dir, !(*ob).forwardDirection);
  }
  
  // If the target and current angles are not equal power the motor using PWM
  if(diff > 0.1)
  {
    analogWrite((*ob).servoPin, 10);
  }
  else
  {
    analogWrite((*ob).servoPin, 0);
  }
}

void loop()
{
  /* Check for input */ 
  while (Serial.available()) // TODO: Implement timeout to prevent getting stuck
  {
    readStr = Serial.readString();
    motor1.target_angle = 10;  // TODO: Fill up for all angles from input by seperation
    motor2.target_angle = 10; 
    motor3.target_angle = 10; 
    motor4.target_angle = 10; 
  }

  /* Update all current angle using the encoder count */
  motor1.current_angle = (double)motor1.enc_count * motor1.encoder_factor;
  motor2.current_angle = (double)motor2.enc_count * motor2.encoder_factor;
  motor3.current_angle = (double)motor3.enc_count * motor3.encoder_factor;
  motor4.current_angle = (double)motor4.enc_count * motor4.encoder_factor;

  PIDControl(&motor1, &myPID1);
  PIDControl(&motor2, &myPID2);
  PIDControl(&motor3, &myPID3);
  PIDControl(&motor4, &myPID4);
  
}
