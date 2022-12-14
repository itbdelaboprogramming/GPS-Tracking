// Initialize the variables with some randomly-chosen integers
volatile double odo_forward_velocity = 30.02;
volatile double odo_turning_velocity = 2.30;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // Print integers every 500 milliseconds
  Serial.print(odo_forward_velocity); Serial.print(",");
  Serial.println(odo_turning_velocity);
  delay(100);
}
