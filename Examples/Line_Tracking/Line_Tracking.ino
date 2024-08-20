#define PIN_BATTERY_SENSE A0
#define PIN_LINE_SENSE_LEFT  A1
#define PIN_LINE_SENSE_CENTER A2
#define PIN_LINE_SENSE_RIGHT  A3


int left_direction_pin = 4;
int right_direction_pin = 3;
int left_speed_pin = 6;
int right_speed_pin = 5;

int n_laps = 0;

int left_speed = 0;
int right_speed = 0;

int line_sense_left, line_sense_right, line_sense_center;
float battery_voltage = 0;

void stop()
{
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
  Serial.begin(9600); //set baud rate
  pinMode(PIN_LINE_SENSE_LEFT, INPUT); //
  pinMode(PIN_LINE_SENSE_RIGHT, INPUT); //
  pinMode(PIN_LINE_SENSE_CENTER, INPUT); //
  pinMode(PIN_BATTERY_SENSE, INPUT);

  Serial.println("=== Ready! ===");
}

void decision_tree()
{
  if(line_sense_left == 1) {
    if(line_sense_right == 1) 
    {
      Serial.println("Stoping");
      left_speed = 0;
      right_speed = 0;
    }
    else{
      left_speed = -255;
      right_speed = 255;
      Serial.println("Turning left");
    }
  }
  else{
    if(line_sense_right == 1){
      left_speed = 255;
      right_speed = -255;   
      Serial.println("Turning right");
    }
    else{
      if (line_sense_center == 1)
      {
        left_speed = 255;
        right_speed = 255;
        Serial.println("Going forward");
      }
      else
      {
        Serial.println("Stoping");
        left_speed = 0;
        right_speed = 0;
      }

    }
  }
}

void decision_overrides()
{
  if (battery_voltage < 5.5)
  {
    left_speed = 0;
    right_speed = 0;
    Serial.println("Override: No power, car stopped.");
  }
}

void loop() {

  // read all the sensors
  line_sense_left = digitalRead(PIN_LINE_SENSE_LEFT);
  line_sense_center = digitalRead(PIN_LINE_SENSE_CENTER);
  line_sense_right = digitalRead(PIN_LINE_SENSE_RIGHT);
  int _battery_reading = analogRead(PIN_BATTERY_SENSE);
  battery_voltage = _battery_reading / 1023.0 * 5.0 * 4.0;
  

  // make decisions
  decision_tree();
  decision_overrides();


  // Execute actions
  commit_speed();


  // This is the time step of each action
  delay(50);
  stop();

  // Allow everything to come to a complete stop
  delay(50);
  }