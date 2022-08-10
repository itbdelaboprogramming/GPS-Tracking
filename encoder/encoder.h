#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
    public:
    //contructor to assign encoder pin
      Encoder(int pin_a, int pin_b);
      void start(void(*userFuncA)(void),void(*userFuncB)(void));
      int getPinA(){return enc_a;};
      int getPinB(){return enc_b;};
      int getPos(){return enc_pos;};
      void doEncoderB();
      void doEncoderA();
      friend void callBackFunction(Encoder);
      void setPinA(int a);
      void setPinB(int b);
      
      
    private:
      int enc_a;
      int enc_b;
      int enc_pos;
      int data_a;
      int data_b;
      
};



#endif
