#ifndef _SetupRelay_H_
#define _SetupRelay_H_

class SetupRelay {
public:
    char* id;
    const int pin[1] = {2}; // Corresponds to pin D2
    const bool initState[1] = {false};
    const bool invert[1] = {true};
};


#endif
