
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

#include <alias_definition.hpp>

namespace meshac {

    #define SENSIBILITY 0.1f    // or glm::epsilon<float>()

    class CrossRatioTuple {
    public:
        /*
         * Inserts a new point at the end of those in the array.
         */
        void append(glm::vec2 *point);

        /* 
         * Computes the cross ratio of the given four points.
         */
        double crossRatio();

        /*
         * Computes the jacobian of the quadruplets.
         */
        EigVector jacobian();

        /*
         * Returns the average distance between the points in the quadruplets.
         */
        double avgDistance();

        /*
         * Checks if the given point is contained in the tuple.
         */
        bool isInTuple(glm::vec2 point);

        /*
         * Overloading < operation for comparation.
         */
        bool operator<(const CrossRatioTuple &other) const;

    private:
        void precomputeDistancesBetweenPoints();

        /*
         * First point x, Second point y, Third point z, Fourth point t.
         */
        std::vector<glm::vec2 *> points;

        /*
         * Precomputed distances.
         */
        double xy, xz, xt, yz, yt, zt;

        const glm::vec2 EPSILON = glm::vec2(SENSIBILITY);
    
    };
    

}   // namespace meshacac


#endif // ACCURACY_CROSS_RATIO_TUPLE_H
