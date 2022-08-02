/** 
  Code to compute PID control
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]R. C. Dorf and R. H. Bishop, Modern Control Systems. Essex: Pearson, Cop, 2017.
**/

#include "pidIr.h"

pidIr::pidIr(float p, float i, float d, float _Ts){
    Kp = p;
    Ki = i;
    Kd = d;
    Ts = _Ts;
    last_error = 0;
    last_output = 0;

};

float pidIr::compute(float setpoint, float feedback, float max_output){
    error = setpoint - feedback;
    last_error = error;

    output = (Kp + Ki*Ts + (Kd/Ts))*error - Kd*Ts*last_error + Ki*last_output;
    if (output > max_output){
        output = last_output;
    }

    last_output = output;

    return output;
};

