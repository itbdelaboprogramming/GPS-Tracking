/** 
  Low Pass Filter code - Header File
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]W. J. Palm, System dynamics. New York, Ny: Mcgraw-Hill, 2014.
**/

#ifndef LPF_H
#define LPF_H

class LPF{
    public:
    //
    LPF(float _fc, float _Ts);
    
    float filter(float raw);

    private:
    float fc;
    float Ts;
    float filtered;
    float last_filtered;
    float alfa;
};

#endif