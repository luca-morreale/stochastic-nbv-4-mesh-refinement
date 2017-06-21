#ifndef CAM_POSITION_GENERATOR_LOC_SOLVER_GENERATOR_H
#define CAM_POSITION_GENERATOR_LOC_SOLVER_GENERATOR_H

#include <opview/SolverGenerator.hpp>

namespace opview {
    
    class LOCSolverGenerator : public SolverGenerator {
    public:
        LOCSolverGenerator() : SolverGenerator() { /*    */ }
        ~LOCSolverGenerator() { /*    */ }

        virtual AdderInferencePtr getOptimizerAlgorithm(GraphicalModelAdder &model, VarIndexList currentOptimal, size_t numVariables);
        // virtual MultiplierInferencePtr getOptimizerAlgorithm(GraphicalModelMultiplier &model, VarIndexList currentOptimal, size_t numVariables);

    private:
        const std::string solver = "ad3";
        const double phi = 0.5;
        const size_t maxBlockRadius  = 50;
        const size_t maxTreeRadius = 50;
        const double pFastHeuristic = 0.7;
        const size_t maxIterations = 100000;
        const size_t stopAfterNBadIterations = 10000;
        const size_t maxBlockSize = 0;
        const size_t maxTreeSize = 0;
        const int treeRuns = 5;
    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_LOC_SOLVER_GENERATOR_H
