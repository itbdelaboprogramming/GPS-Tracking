// Masukkan PIN yang mau diuji interrupt
#define PIN 2

// Wiring untuk menguji interrupt menggunakan potensiometer
// Merah terhubung ke 5V arduino dan salah satu kaki yang dipinggir dari potensiometer
// Hitam terhubung ke GND arduino dan salah satu kaki yang dipinggir dari potensiometer
// Kuning terhubung ke PIN interrupt yang mau ditest dan kaki tengah dari potensiometer

// Untuk menguji putar potensiometer hingga maksimum atau minimum
// Serial print akan muncul jika potensiometer mendapat sinyal LOW

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN), tulis, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = digitalRead(PIN);
  //Serial.println(val);
}

void tulis(){
  Serial.println("LOW");
}
