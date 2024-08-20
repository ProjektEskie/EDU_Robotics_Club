int left_direction_pin = 4;
int right_direction_pin = 3;
int left_speed_pin = 6;
int right_speed_pin = 5;

int left_speed = 0;
int right_speed = 0;

void brake_stop()
{
  // Run the motor in the reverse direction for 100ms then enter idle
  left_speed = -1 * left_speed;
  right_speed = -1 * right_speed;

  commit_speed();

  delay(50);

  left_speed = 0;
  right_speed = 0;
  commit_speed();
}

void commit_speed()
{
  int abs_left_speed;
  int abs_right_speed;
  abs_left_speed = abs(left_speed);
  abs_right_speed = abs(right_speed);

  if (left_speed >= 0)
  {
    digitalWrite(left_direction_pin, HIGH);
  }
  else
  {
    digitalWrite(left_direction_pin, LOW);
  }

  if (right_speed >= 0)
  {
    digitalWrite(right_direction_pin, LOW);
  }
  else
  {
    digitalWrite(right_direction_pin, HIGH);
  }
  analogWrite(left_speed_pin, abs_left_speed);
  analogWrite(right_speed_pin, abs_right_speed);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(left_direction_pin, OUTPUT);
  pinMode(right_direction_pin, OUTPUT);
  pinMode(left_speed_pin, OUTPUT);
  pinMode(right_speed_pin, OUTPUT);
}

void loop() {

  // advance forward
  left_speed = 150;
  right_speed = 150;
  commit_speed();
  delay(1000);
  brake_stop();
  delay(500);

  // Turn 180 in the right direction
  left_speed = 200;
  right_speed = -200;
  commit_speed();
  delay(1500);
  brake_stop();
  delay(500);

  // advance forward
  left_speed = 150;
  right_speed = 150;
  commit_speed();
  delay(1000);
  brake_stop();
  delay(500);

  // Turn 180 in the left direction
  left_speed = -200;
  right_speed = 200;
  commit_speed();
  delay(1500);
  brake_stop();
  delay(500);

  while(true){
    delay(1);
  }
}
