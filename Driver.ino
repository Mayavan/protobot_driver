#include "Controller.cpp"

volatile long enc_count = 0;
int servoPin = PB6;

double angle = 0;
double current_angle = 0;

String readStr;
uint8_t enc_val = 0;

void setup()
{
  // all your normal setup code
  attachInterrupt(digitalPinToInterrupt(PB0), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PB1), encoder_isr, CHANGE);

  pinMode(servoPin, OUTPUT);
  analogWrite(servoPin, 0);

  Serial.begin(115200);
}

void encoder_isr()
{
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  enc_val = enc_val << 2;
  enc_val = enc_val | (GPIOB->regs->IDR & 0b11);

  enc_count = enc_count + lookup_table[enc_val & 0b1111];
}

void loop()
{
  while (Serial.available())
  {
    readStr = Serial.readString();
    angle = readStr.toInt();
    Serial.println(angle);
    Serial.println(current_angle);
  }
  current_angle = (double)enc_count * 0.005;
  if (current_angle < 0)
    current_angle = current_angle * -1;
  while (current_angle >= 360)
    current_angle = current_angle - 360;

  int diff = current_angle - angle;
  if (diff < 0)
    diff = diff * -1;

  while (diff > 0.1)
  {
    current_angle = (double)enc_count * 0.005;

    if (current_angle < 0)
      current_angle = current_angle * -1;
    while (current_angle >= 360)
      current_angle = current_angle - 360;

    analogWrite(servoPin, 10);
    diff = current_angle - angle;
    if (diff < 0)
      diff = diff * -1;
  }
  analogWrite(servoPin, 0);
}
