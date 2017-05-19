#include <opview/ICMSolverGenerator.hpp>

namespace opview {
    
    AdderInferencePtr ICMSolverGenerator::getOptimizerAlgorithm(GraphicalModelAdder &model, size_t numVariables)
    {
        LabelList startPoint = genStartPoint(numVariables);
        ICM::Parameter para(startPoint);
        ICMPtr algorithm = new ICM(model, para);
        
        return algorithm;
    }
    
    MultiplierInferencePtr ICMSolverGenerator::getOptimizerAlgorithm(GraphicalModelMultiplier &model, size_t numVariables)
    {
        LabelList startPoint = genStartPoint(numVariables);
        ICM::Parameter para(startPoint);
        //ICMPtr algorithm = new ICM(model, para);    // this is not for the multiplier
        
        return NULL;
    }

} // namespace opview
