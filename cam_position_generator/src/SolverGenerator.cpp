#include <opview/SolverGenerator.hpp>

namespace opview {

    LabelList SolverGenerator::genStartPoint(size_t numVariables)   // FIXME point should start from bounding box
    {
        LabelList startPoint(numVariables);
        for (int i = 0; i < numVariables; i++) {
            startPoint[i] = static_cast<LabelType>(std::rand()) / RAND_MAX;
            std::cout << "startPoint[" << i << "]: " << startPoint[i] << std::endl;
        }
        return startPoint;
    }

}