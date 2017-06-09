#ifndef CAM_POSITION_GENERATOR_MULTIBRUTEFORCE_SOLVER_GENERATOR_H
#define CAM_POSITION_GENERATOR_MULTIBRUTEFORCE_SOLVER_GENERATOR_H

#include <opview/SolverGenerator.hpp>

namespace opview {
    
    class MultiBruteForceSolverGenerator : public SolverGenerator {
    public:
        MultiBruteForceSolverGenerator() : SolverGenerator() { /*    */ }
        ~MultiBruteForceSolverGenerator() { /*    */ }

        virtual AdderInferencePtr getOptimizerAlgorithm(GraphicalModelAdder &model, LabelList currentOptimal, size_t numVariables);
        // virtual MultiplierInferencePtr getOptimizerAlgorithm(GraphicalModelMultiplier &model, LabelList currentOptimal, size_t numVariables);
        
    };

    typedef MultiBruteForceSolverGenerator* MultiBruteForceSolverGeneratorPtr;


} // namespace opview

#endif // CAM_POSITION_GENERATOR_BRUTEFORCE_SOLVER_GENERATOR_H
