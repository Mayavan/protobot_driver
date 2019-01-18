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
};

/* Joint 1 */
joint j1 = {PB8, PB9, PB6, PB7, 0, 0, 0, 0, 0.005, HIGH};
/* Joint 2 */
joint j2 = {PA8, PB15, PA9, PA10, 0, 0, 0, 0, 0.005, HIGH};
/* Joint 3 */
joint j3 = {PA2, PA3, PA1, PA0, 0, 0, 0, 0, 0.0121951219512195, HIGH};
/* Joint 4 */
joint j4 = {PB0, PA5, PA6, PA7, 0, 0, 0, 0, 0.0121951219512195, HIGH};

/* String to decode the input command */
String readStr;

/** 
 * Quadrature ENCODER Interrupt Service Routine
 */

/* Quadrature decoder for PB6 and PB7 pins */
void joint1ISR()
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  j1.enc_val = j1.enc_val << 2;
  j1.enc_val = j1.enc_val | ((GPIOB->regs->IDR & 0b11000000)>>6);

  j1.enc_count = j1.enc_count + lookup_table[j1.enc_val & 0b1111];
}

/* Quadrature decoder for PA9 and PA10 pins */
void joint2ISR()
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  j2.enc_val = j2.enc_val << 2;
  j2.enc_val = j2.enc_val | ((GPIOA->regs->IDR & 0b11000000000)>>9);

  j2.enc_count = j2.enc_count + lookup_table[j2.enc_val & 0b1111];
}

/* Quadrature decoder for PA0 and PA1 pins */
void joint3ISR()
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  j3.enc_val = j3.enc_val << 2;
  j3.enc_val = j3.enc_val | (GPIOA->regs->IDR & 0b11);

  j3.enc_count = j3.enc_count + lookup_table[j3.enc_val & 0b1111];
}

/* Quadrature decoder for PA6 and PA7 pins */
void joint4ISR()
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  j4.enc_val = j4.enc_val << 2;
  j4.enc_val = j4.enc_val | ((GPIOA->regs->IDR & 0b11000000)>>6);

  j4.enc_count = j4.enc_count + lookup_table[j4.enc_val & 0b1111];
}

void setup()
{
  // Attach pins to interrupt service routine
  attachInterrupt(digitalPinToInterrupt(j1.encA), joint1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j1.encA), joint1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j2.encA), joint2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j2.encA), joint2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j3.encA), joint3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j3.encA), joint3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j4.encA), joint4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(j4.encA), joint4ISR, CHANGE);

  // Setup all PWM Pins and Direction pins
  pinMode(j1.servoPin, OUTPUT);
  pinMode(j1.dir, OUTPUT);
  pinMode(j2.servoPin, OUTPUT);
  pinMode(j2.dir, OUTPUT);
  pinMode(j3.servoPin, OUTPUT);
  pinMode(j3.dir, OUTPUT);
  pinMode(j4.servoPin, OUTPUT);
  pinMode(j4.dir, OUTPUT);

  // Begin Serial Communication on USB port
  Serial.begin(115200);
}

void PIDControl(joint ob){ 
  // Set the direction of motor based on the target angle
  int diff;
  diff = ob.current_angle - ob.target_angle;
  if (diff < 0)
  {
    digitalWrite(ob.dir, ob.forwardDirection);
    diff = diff * -1;
  }
  else
  {
    digitalWrite(ob.dir, !ob.forwardDirection);
  }

  // If the target and current angles are not equal power the motor using PID controller
  if(diff > 0.1)
  {
    analogWrite(servoPin, 10);
  }
  else
  {
    analogWrite(servoPin, 0);
  }
}

void PWMControl(joint ob){
  // Set the direction of motor based on the target angle
  int diff;
  diff = ob.current_angle - ob.target_angle;
  if (diff < 0)
  {
    digitalWrite(ob.dir, ob.forwardDirection);
    diff = diff * -1;
  }
  else
  {
    digitalWrite(ob.dir, !ob.forwardDirection);
  }
  
  // If the target and current angles are not equal power the motor using PWM
  if(diff > 0.1)
  {
    analogWrite(servoPin, 10);
  }
  else
  {
    analogWrite(servoPin, 0);
  }
}

void loop()
{
  /* Check for input */ 
  while (Serial.available()) // TODO: Implement timeout to prevent getting stuck
  {
    readStr = Serial.readString();
    angle = readStr.toInt();
    j1.target_angle = angle;  // TODO: Fill up for all angles from input by seperation
    
  }

  /* Update all current angle using the encoder count */
  j1.current_angle = (double)j1.enc_count * j1.encoder_factor;
  j2.current_angle = (double)j2.enc_count * j2.encoder_factor;
  j3.current_angle = (double)j3.enc_count * j3.encoder_factor;
  j4.current_angle = (double)j4.enc_count * j4.encoder_factor;

  PIDControl(j1);
  PWMControl(j2);
  PWMControl(j3);
  PIDControl(j4);
  
}
