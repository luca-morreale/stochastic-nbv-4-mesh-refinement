
#ifndef MESH_ACCURACY_TYPE_DEFINITION_H
#define MESH_ACCURACY_TYPE_DEFINITION_H

#include <glm/glm.hpp>

#include <opencv/cv.hpp>

#include <set>
#include <vector>

#include <CrossRatioTuple.hpp>

namespace meshac {
    
    typedef std::vector<int> IntList;
    typedef std::vector<IntList> IntArrayList;

    typedef std::vector<cv::Vec2f> CVList2DVec;
    typedef std::vector<glm::vec2> GLMList2DVec;

    typedef std::set<CrossRatioTuple> CrossRatioTupleSet;
    typedef std::vector<CrossRatioTupleSet> ListCrossRatioTupleSet;
    
    
} // namesace meshac

#endif // MESH_ACCURACY_TYPE_DEFINITION_H
