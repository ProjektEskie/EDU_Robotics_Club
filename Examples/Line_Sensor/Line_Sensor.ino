#define PIN_LINE_SENSE_LEFT  A1
#define PIN_LINE_SENSE_CENTER A2
#define PIN_LINE_SENSE_RIGHT  A3

int line_sense_left, line_sense_right, line_sense_center;

void setup() {
  Serial.begin(9600); //set baud rate
  pinMode(PIN_LINE_SENSE_LEFT, INPUT); //
  pinMode(PIN_LINE_SENSE_RIGHT, INPUT); //
  pinMode(PIN_LINE_SENSE_CENTER, INPUT); //

  Serial.println("=== Ready! ===");
}

void loop() {
  line_sense_left = digitalRead(PIN_LINE_SENSE_LEFT);
  line_sense_center = digitalRead(PIN_LINE_SENSE_CENTER);
  line_sense_right = digitalRead(PIN_LINE_SENSE_RIGHT);

  Serial.print("Line Sensor Value (L / M / R): ");
  Serial.print(line_sense_left);
  Serial.print('\t');
  Serial.print(line_sense_center);
  Serial.print('\t');
  Serial.print(line_sense_right);
  Serial.print('\n');

  // Decision tree
  if (line_sense_left == 1)
  {
    // >, <, >=, <=, !=
  }


  delay(100);
}
