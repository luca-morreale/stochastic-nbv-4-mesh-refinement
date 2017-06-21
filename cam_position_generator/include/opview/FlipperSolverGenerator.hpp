#ifndef CAM_POSITION_GENERATOR_FLIPPER_SOLVER_GENERATOR_H
#define CAM_POSITION_GENERATOR_FLIPPER_SOLVER_GENERATOR_H

#include <opview/SolverGenerator.hpp>

namespace opview {
    
    class FlipperSolverGenerator : public SolverGenerator {
    public:
        FlipperSolverGenerator() : SolverGenerator() { /*    */ }
        ~FlipperSolverGenerator() { /*    */ }

        virtual AdderInferencePtr getOptimizerAlgorithm(GraphicalModelAdder &model, VarIndexList currentOptimal, size_t numVariables);
        // virtual MultiplierInferencePtr getOptimizerAlgorithm(GraphicalModelMultiplier &model, VarIndexList currentOptimal, size_t numVariables);
    
    private:
        size_t maxSubgraphSize = 5; // works only if it is 1
    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_FLIPPER_SOLVER_GENERATOR_H
