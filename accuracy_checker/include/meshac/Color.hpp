#ifndef MESH_ACCURACY_COLOR_H
#define MESH_ACCURACY_COLOR_H

#include <string>
#include <vector>

namespace meshac {
    
    struct Color {
        float r = 0;
        float g = 0;
        float b = 0;
        float a = 0;

        Color(float r, float g, float b, float a);

        std::string string();
    };

    typedef Color * ColorPtr;
    typedef std::vector<Color> ColorList;

} // namespace meshac

#endif // MESH_ACCURACY_COLOR_H
