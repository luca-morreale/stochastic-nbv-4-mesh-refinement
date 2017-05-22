#ifndef CAM_POSITION_GENERATOR_SOLVER_GENERATOR_H
#define CAM_POSITION_GENERATOR_SOLVER_GENERATOR_H

#include <cstdlib>

#include <opview/type_definition.h>

namespace opview {
    
    class SolverGenerator {
    public:
        SolverGenerator() { /*    */ }
        ~SolverGenerator() { /*    */ }

        virtual AdderInferencePtr getOptimizerAlgorithm(GraphicalModelAdder &model, LabelList currentOptimal, size_t numVariables) = 0;
        // virtual MultiplierInferencePtr getOptimizerAlgorithm(GraphicalModelMultiplier &model, LabelList currentOptimal, size_t numVariables) = 0;

    protected:
        LabelList genStartPoint(size_t numVariables);

    };

    typedef SolverGenerator* SolverGeneratorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_SOLVER_GENERATOR_H
