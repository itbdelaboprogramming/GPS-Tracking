// Initialize the variables with some randomly-chosen integers
volatile double odo_VL = -0.5;
volatile double odo_VR = 0.5;
volatile double mode = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // Print integers every 500 milliseconds
  Serial.print(odo_VL); Serial.print(",");
  Serial.print(odo_VR); Serial.print(",");
  Serial.println(mode);
  delay(100);
}
