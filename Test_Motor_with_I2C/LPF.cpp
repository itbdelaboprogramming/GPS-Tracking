/** 
  Code Low Pass Filter
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]W. J. Palm, System dynamics. New York, Ny: Mcgraw-Hill, 2014.
**/

#include "LPF.h"
#define PI 3.141592653

// Constructor to assign LPF parameters
LPF::LPF(float _fc){
    fc = _fc;
    filtered = 0;
    last_filtered = 0;
};

float LPF::filter(float raw, float _Ts){

    alfa = (2.0*PI*fc*_Ts)/(1.0+(2*PI*fc*_Ts));
    filtered = alfa*raw + (1-alfa)*last_filtered;
    
    //if (absolute((filtered-last_filtered)) > 10.0*last_filtered) {
    //  if (last_filtered != 0) {
    //    filtered = last_filtered;
    //  }
    //}
    last_filtered = filtered;

    return filtered;
};

float LPF::absolute(float val){
    if (val > 0) {
      return val;
    } else {
      return -val;
    }
};
