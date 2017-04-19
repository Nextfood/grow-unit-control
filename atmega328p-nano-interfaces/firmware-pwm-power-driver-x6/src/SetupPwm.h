#ifndef _SetupPwm_H_
#define _SetupPwm_H_

class SetupPwm {
public:
    char* id;
    const int pin[6] = {3, 5, 6, 9, 10, 12};
    const float pwm[6] = {0, 0, 0, 0, 0, 0};
};


#endif
