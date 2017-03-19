
#include <meshac/Color.hpp>

namespace meshac {

    Color::Color(float r, float g, float b, float a)
    {
        this->r = r;
        this->g = g;
        this->b = b;
        this->a = a;
    }

    std::string Color::string()
    {
        return std::to_string(this->r) + " " + std::to_string(this->g) + " " + std::to_string(this->b) + " " + std::to_string(this->a);
    }

} // namespace meshac
