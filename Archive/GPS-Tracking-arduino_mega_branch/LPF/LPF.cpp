/** 
  Code to compute PID control
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]W. J. Palm, System dynamics. New York, Ny: Mcgraw-Hill, 2014.
**/

#include "LPF.h"

// Constructor to assign PID parameters
LPF::LPF(float _fc, float _Ts){
    fc = _fc;
    Ts = _Ts;
    filtered = 0;
    last_filtered = 0;
    alfa = (2*3.14159265*fc*Ts)/(1+(2*3.14159265*fc*Ts));
};

float LPF::filter(float raw){

    filtered = alfa*raw + (1-alfa)*last_filtered;
    last_filtered = filtered;

    return filtered;
};