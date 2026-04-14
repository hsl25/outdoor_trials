#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <stdio.h>
#include <cstdint>

class Navigation {
    public:
        Navigation();
        float calc_width(uint16_t length1, int angle1, uint16_t length2, int angle2);
    private:
};

#endif