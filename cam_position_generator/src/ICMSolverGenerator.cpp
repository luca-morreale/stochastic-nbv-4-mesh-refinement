#include <opview/ICMSolverGenerator.hpp>

namespace opview {
    
    AdderInferencePtr ICMSolverGenerator::getOptimizerAlgorithm(GraphicalModelAdder &model, LabelList currentOptimal, size_t numVariables)
    {
        LabelList startPoint = (currentOptimal.size() > 0) ? currentOptimal : genStartPoint(numVariables);
        ICM::Parameter para(startPoint);
        ICMPtr algorithm = new ICM(model, para);
        
        return algorithm;
    }
    
    MultiplierInferencePtr ICMSolverGenerator::getOptimizerAlgorithm(GraphicalModelMultiplier &model, LabelList currentOptimal, size_t numVariables)
    {
        LabelList startPoint = (currentOptimal.size() > 0) ? currentOptimal : genStartPoint(numVariables);
        ICM::Parameter para(startPoint);
        //ICMPtr algorithm = new ICM(model, para);    // this is not for the multiplier
        
        return NULL;
    }

} // namespace opview
