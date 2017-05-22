#ifndef CAM_POSITION_GENERATOR_ICM_SOLVER_GENERATOR_H
#define CAM_POSITION_GENERATOR_ICM_SOLVER_GENERATOR_H

#include <opview/SolverGenerator.hpp>

namespace opview {
    
    class ICMSolverGenerator : public SolverGenerator {
    public:
        ICMSolverGenerator() : SolverGenerator() { /*    */ }
        ~ICMSolverGenerator() { /*    */ }

        virtual AdderInferencePtr getOptimizerAlgorithm(GraphicalModelAdder &model, LabelList currentOptimal, size_t numVariables);
        // virtual MultiplierInferencePtr getOptimizerAlgorithm(GraphicalModelMultiplier &model, LabelList currentOptimal, size_t numVariables);
        
    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_ICM_SOLVER_GENERATOR_H
