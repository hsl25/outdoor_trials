#include <math.h>

#include "navigation.hpp"

// Constructor
Navigation::Navigation() {
    
}

float Navigation:: calc_width(uint16_t length1, int angle1, uint16_t length2, int angle2) {
    int theta_diff = angle2 - angle1;
    float x1 = (float) length1;
    float x2 = (float) length2;

    return sqrt((x1 * x1) + (x2 * x2) - (2 * x1 * x2 * cos(theta_diff)));
}


