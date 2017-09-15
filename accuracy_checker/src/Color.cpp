#include <meshac/Color.hpp>

namespace meshac {

    Color::Color(byte r, byte g, byte b, float a)
    {
        this->r = r;
        this->g = g;
        this->b = b;
        this->a = a;
    }

    std::string Color::to_string()
    {
        return std::to_string(this->r) + " " + std::to_string(this->g) + \
                " " + std::to_string(this->b) + " " + std::to_string(this->a);
    }

} // namespace meshac
