#include <opview/SolverGenerator.hpp>

namespace opview {

    VarIndexList SolverGenerator::genStartPoint(size_t numVariables)   // FIXME point should start from bounding box
    {
        VarIndexList startPoint(numVariables);
        for (int i = 0; i < numVariables; i++) {
            startPoint[i] = static_cast<VariableIndexType>(std::rand()) / RAND_MAX;
            std::cout << "startPoint[" << i << "]: " << startPoint[i] << std::endl;
        }
        return startPoint;
    }

}