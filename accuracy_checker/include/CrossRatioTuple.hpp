
#ifndef MESH_ACCURACY_CROSS_RATIO_TUPLE_H
#define MESH_ACCURACY_CROSS_RATIO_TUPLE_H

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <algorithm>
#include <list>
#include <map>
#include <vector>


namespace glm {
namespace detail {

    static bool operator <(const glm::vec2 &vecA, const glm::vec2 &vecB)
    {
       const double epsilion = 0.0001;

       return fabs(vecA[0] - vecB[0]) < epsilion;
    }
}
}

namespace meshac {

    struct CrossRatioTuple {

        std::vector<glm::vec2 *> points;

        // Inserts a new point at the end of those in the array
        void append(glm::vec2 *point) {
            if (points.size() < 4) {
                points.push_back(point);
            } else {
                std::sort(points.begin(), points.end());
            }
        }

        // Computes the cross ratio of the given four points
        double crossRatio() {

            double n1 = glm::distance2(*points[0], *points[3]);
            double d1 = glm::distance2(*points[2], *points[3]);
            double n2 = glm::distance2(*points[2], *points[4]);
            double d2 = glm::distance2(*points[1], *points[4]);

            return (n1 * n2) / (d1 * d2);
        }

        // Overloading < operation for comparation
        bool operator<(const CrossRatioTuple &other) const
        {
            if (points.size() < other.points.size()) {
                return true;
            }

            glm::vec2 eps = glm::vec2();
            eps[0] = glm::epsilon<float>();
            eps[1] = glm::epsilon<float>();

            const glm::vec2 EPSILON = eps;

            // find better compare
            if (glm::epsilonEqual(*points[0], *(other.points[0]), EPSILON)[0] && 
                glm::epsilonEqual(*points[1], *(other.points[1]), EPSILON)[0] && 
                glm::epsilonEqual(*points[2], *(other.points[2]), EPSILON)[0] && 
                glm::epsilonEqual(*points[3], *(other.points[3]), EPSILON)[0]) {
                return true;
            }
            
            return false;
        }    
    };
    

}   // namespace meshacac


#endif // ACCURACY_CROSS_RATIO_TUPLE_H
