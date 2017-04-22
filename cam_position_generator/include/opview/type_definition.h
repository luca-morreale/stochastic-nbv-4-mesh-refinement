#ifndef CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
#define CAM_POSITION_GENERATOR_TYPE_DEFINITION_H

#include <opview/alias_definition.h>

namespace opview {

    struct Face {
        FaceVertices face;
        PointD3 oppositeVertex;
    };

    typedef std::vector<Face> FaceList;

}

#endif // CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
