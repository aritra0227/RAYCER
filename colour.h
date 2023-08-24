#ifndef COLOUR_H
#define COLOUR_H
#include "vec3.h"
#include <iostream>

using colour = vec3;

void write_color(std::ostream &out, colour pixel_color)
{
    // Write the translated [0,255] value of each color component.
    out << static_cast<int>(255.999 * pixel_color.x()) << ' '
        << static_cast<int>(255.999 * pixel_color.y()) << ' '
        << static_cast<int>(255.999 * pixel_color.z()) << '\n';
}

#endif