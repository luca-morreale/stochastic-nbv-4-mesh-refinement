
#ifndef MESH_ACCURACY_TYPE_DEFINITION_H
#define MESH_ACCURACY_TYPE_DEFINITION_H

#include <glm/glm.hpp>

#include <Eigen/Dense>

#include <opencv/cv.hpp>

#include <set>
#include <vector>

#include <CrossRatioTuple.hpp>

namespace meshac {
    typedef std::set<CrossRatioTuple> CrossRatioTupleSet;
    typedef std::vector<CrossRatioTupleSet> ListCrossRatioTupleSet;   
    
} // namesace meshac

#endif // MESH_ACCURACY_TYPE_DEFINITION_H
