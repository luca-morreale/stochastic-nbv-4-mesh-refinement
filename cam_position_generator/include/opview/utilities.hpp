#ifndef CAM_POSITION_GENERATOR_UTILITIES_H
#define CAM_POSITION_GENERATOR_UTILITIES_H

#include <cmath>

#include <glm/gtx/norm.hpp>

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
    GLMVec3List concatLists(GLMVec3List &a, GLMVec3List &b);

    float deg2rad(float deg);
    float rad2deg(float rad);

    double bivariateGuassian(double x, double y, double centerx, double centery, double sigmax, double sigmay);

    double logVonMises(GLMVec3 &point, GLMVec3 &centroid, GLMVec3 &normalVector, VonMisesConfiguration &config);
    double logVonMises(GLMVec3 &v, GLMVec3 &normalVector, VonMisesConfiguration &config);
    double logVonMises(double angle, VonMisesConfiguration &vonMisesConfig);
    double logBessel0(double k);


    template<typename K, typename V>
    std::map<V, K> invertMap(std::map<K, V> &mapping)
    {
        std::map<V, K> unmapping;
        for (typename std::map<K, V>::iterator i = mapping.begin(); i != mapping.end(); ++i) {
            unmapping[i->second] = i->first;
        }
        return unmapping;
    }

}

#endif // CAM_POSITION_GENERATOR_UTILITIES_H
