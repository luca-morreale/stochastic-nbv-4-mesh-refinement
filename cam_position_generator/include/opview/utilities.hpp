#ifndef CAM_POSITION_GENERATOR_UTILITIES_H
#define CAM_POSITION_GENERATOR_UTILITIES_H

#include <cmath>

#include <opview/alias_definition.h>
#include <opview/type_definition.h>

namespace opview {
    
    Face convertCGALFaceToFace(CGALFace triangleVertices);
    Face convertCGALFaceToFace(CGALFace triangleVertices, PointD3 oppositeVertex);

    GLMVec3 convertPoinToGLMVec(PointD3 &point);
    CGALVec3 convertPoinToCGALVec(PointD3 &point);
    CGALVec3 convertPoinToCGALVec(GLMVec3 &point);
    CGALVec3List convertListToCGALVecList(GLMVec3List &inList);

    double distanceCGALVec3(CGALVec3 &a, CGALVec3 &b);

    float deg2rad(float deg);
    float rad2deg(float rad);

    double bivariateGuassian(double x, double y, double centerx, double centery, double sigmax, double sigmay);

}

#endif // CAM_POSITION_GENERATOR_UTILITIES_H
