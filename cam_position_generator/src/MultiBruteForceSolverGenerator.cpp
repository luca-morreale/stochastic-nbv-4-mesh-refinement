#include <opview/MultiBruteForceSolverGenerator.hpp>

namespace opview {
    
    AdderInferencePtr MultiBruteForceSolverGenerator::getOptimizerAlgorithm(GraphicalModelAdder &model, LabelList currentOptimal, size_t numVariables)
    {
        MultiBruteforcePtr algorithm = new MultiBruteforce(model);  // NOTE missing number of points, should be set manually
        
        return algorithm;
    }
    
    /*MultiplierInferencePtr BruteForceSolverGenerator::getOptimizerAlgorithm(GraphicalModelMultiplier &model, LabelList currentOptimal, size_t numVariables)
    {
        LabelList startPoint = (currentOptimal.size() > 0) ? currentOptimal : genStartPoint(numVariables);
        ICM::Parameter para(startPoint);
        //ICMPtr algorithm = new ICM(model, para);    // this is not for the multiplier
        
        return NULL;
    }*/

} // namespace opview
