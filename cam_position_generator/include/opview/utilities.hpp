#ifndef CAM_POSITION_GENERATOR_UTILITIES_H
#define CAM_POSITION_GENERATOR_UTILITIES_H

#include <opview/alias_definition.h>
#include <opview/type_definition.h>

namespace opview {
    
    Face convertCGALFaceToFace(CGALFace triangleVertices);
    Face convertCGALFaceToFace(CGALFace triangleVertices, PointD3 oppositeVertex);

    GLMVec3 convertPoinToGLMVec(PointD3 &point);
    CGALVec3 convertPoinToCGALVec(PointD3 &point);
    CGALVec3 convertPoinToCGALVec(GLMVec3 &point);
    CGALVec3List convertListToCGALVecList(GLMVec3List &inList);

}

#endif // CAM_POSITION_GENERATOR_UTILITIES_H
