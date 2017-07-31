#ifndef CAM_POSITION_GENERATOR_UTILITIES_H
#define CAM_POSITION_GENERATOR_UTILITIES_H

#include <cmath>

#include <glm/gtx/norm.hpp>

#include <opview/alias_definition.h>
#include <opview/type_definition.h>

namespace opview {
    
    GLMVec3 convertPoinToGLMVec(Point &point);
    Vector convertPoinToCGALVec(Point &point);
    Vector convertPoinToCGALVec(GLMVec3 &point);
    VectorList convertListToCGALVecList(GLMVec3List &inList);

    double distanceVector(Vector &a, Vector &b);

    OrderedPose copy(OrderedPose poses, size_t max);
    

    float deg2rad(float deg);
    float rad2deg(float rad);

    double bivariateGaussian(double x, double y, double centerx, double centery, double sigmax, double sigmay);
    double logBivariateGaussian(double x, double y, double centerx, double centery, double sigmax, double sigmay);

    double logVonMises(GLMVec3 &point, GLMVec3 &centroid, GLMVec3 &normalVector, VonMisesConfiguration &config);
    double logVonMises(GLMVec3 &v, GLMVec3 &normalVector, VonMisesConfiguration &config);
    double logVonMises(double angle, VonMisesConfiguration &vonMisesConfig);
    double logBessel0(double k);

    double modAngle(double x);
    double constrainAngle(double x);


    template<typename K, typename V>
    std::map<V, K> invertMap(std::map<K, V> &mapping)
    {
        std::map<V, K> unmapping;
        for (typename std::map<K, V>::iterator i = mapping.begin(); i != mapping.end(); ++i) {
            unmapping[i->second] = i->first;
        }
        return unmapping;
    }

    template<typename T>
    std::vector<T> concatLists(std::vector<T> &a, std::vector<T> &b)
    {
        a.insert(a.end(), b.begin(), b.end());
        return a;
    }

}

#endif // CAM_POSITION_GENERATOR_UTILITIES_H
