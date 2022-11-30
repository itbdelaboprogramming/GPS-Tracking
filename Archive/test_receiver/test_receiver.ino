#define PIN_1 2
#define PIN_2 3

volatile int pwm_value_1 = 0;
volatile uint64_t prev_time_1 = 0;
volatile int pwm_value_2 = 0;
volatile uint64_t prev_time_2 = 0;
 
void setup() {
  Serial.begin(9600);
  // when pin D2 goes high, call the rising function
  Serial.println("Test Start");
  attachInterrupt(digitalPinToInterrupt(PIN_1), rising1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_2), rising2, RISING);
}
 
void loop() {
  //pwm_value = pulseIn(PIN, HIGH);
  
  Serial.print(map(pwm_value_1,1000,2000,-20,20));Serial.print("\t");
  Serial.print(map(pwm_value_2,1000,2000,-20,20));Serial.print("\t");
  Serial.println();
}

void rising1(){
  //Serial.println("Rise");
  attachInterrupt(digitalPinToInterrupt(PIN_1), falling1, FALLING);
  prev_time_1 = micros();
}

void falling1(){
  //Serial.println("Fall");
  attachInterrupt(digitalPinToInterrupt(PIN_1), rising1, RISING);
  //pwm_value_1 = micros()-prev_time_1;
  pwm_value_1 = pwm_val(prev_time_1);
  //Serial.println(pwm_value);
}

void rising2(){
  //Serial.println("Rise");
  attachInterrupt(digitalPinToInterrupt(PIN_2), falling2, FALLING);
  prev_time_2 = micros();
}

void falling2(){
  //Serial.println("Fall");
  attachInterrupt(digitalPinToInterrupt(PIN_2), rising2, RISING);
  //pwm_value_2 = micros()-prev_time_2;
  pwm_value_2 = pwm_val(prev_time_2);
  //Serial.println(pwm_value);
}

int pwm_val(uint64_t t){
  if(micros()-t>2000){
    return 2000;
  } else if(micros()-t<1000){
    return 1000;
  } else {
    return micros()-t;
  }
}
