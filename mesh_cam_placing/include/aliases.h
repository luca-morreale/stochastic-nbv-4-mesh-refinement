#ifndef ALIASES_CENTERS_READER_H
#define ALIASES_CENTERS_READER_H

#include <vector>
#include <array>

#include <glm/glm.hpp>

namespace camplacing {

    typedef std::vector<double> DoubleList;
    typedef std::vector<std::string> StringList;
    
    typedef glm::vec3 GLMVec3;
    typedef std::vector<GLMVec3> GLMVec3List;

    typedef glm::vec2 GLMVec2;
    typedef std::vector<GLMVec2> GLMVec2List;

    typedef std::array<int, 8> Cube;
    typedef std::vector<Cube> CubeList;

    typedef std::array<int, 3> Axes;
    typedef std::vector<Axes> AxesList;

    typedef glm::mat3 GLMMat3;
    typedef std::vector<GLMMat3> GLMMat3List;

} // namespace camplacing

#endif // ALIASES_CENTERS_READER_H
