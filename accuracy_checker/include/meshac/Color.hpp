#ifndef MESH_ACCURACY_COLOR_H
#define MESH_ACCURACY_COLOR_H

#include <string>
#include <vector>

namespace meshac {

    typedef unsigned char byte;
    
    struct Color {
        byte r = 0;
        byte g = 0;
        byte b = 0;
        float a = 0;

        Color(byte r, byte g, byte b, float a);

        std::string to_string();
    };

    typedef Color * ColorPtr;
    typedef std::vector<Color> ColorList;

} // namespace meshac

#endif // MESH_ACCURACY_COLOR_H
