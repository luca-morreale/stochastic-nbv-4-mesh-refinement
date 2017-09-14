#ifndef MESH_ACCURACY_CROSS_RATIO_TUPLE_H
#define MESH_ACCURACY_CROSS_RATIO_TUPLE_H

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/norm.hpp>

#include <algorithm>
#include <list>
#include <map>
#include <vector>

#include <meshac/alias_definition.hpp>
#include <meshac/type_definition.hpp>

namespace meshac {

    extern const float SENSIBILITY;

    class CrossRatioTuple {
    public:
        /*
         * Inserts a new point at the end of those in the array.
         */
        void append(GLMVec2 &point);

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
        bool isInTuple(GLMVec2 &point);

        GLMVec2List getPoints();

        std::string to_string();

        /*
         * Overloading < operation for comparation.
         */
        bool operator<(const CrossRatioTuple &other) const;

    private:
        void precomputeDistancesBetweenPoints();

        static bool point2DComparator(const GLMVec2 &vecA, const GLMVec2 &vecB);

        /*
         * First point x, Second point y, Third point z, Fourth point t.
         */
        GLMVec2List points;

        /*
         * Precomputed distances.
         */
        double xy, xz, xt, yz, yt, zt;

        const GLMVec2 EPSILON = GLMVec2(1.0f, 1.0f);
    
    };

    typedef CrossRatioTuple * CrossRatioTuplePtr;
    typedef std::set<CrossRatioTuple> CrossRatioTupleSet;
    typedef std::vector<CrossRatioTupleSet> ListCrossRatioTupleSet;
    typedef std::pair<CrossRatioTupleSet, EigMatrix> CrossRatioTupleSetVariance;

}   // namespace meshacac


#endif // ACCURACY_CROSS_RATIO_TUPLE_H
