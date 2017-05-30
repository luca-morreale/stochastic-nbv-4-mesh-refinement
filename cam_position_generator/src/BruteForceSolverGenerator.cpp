#include <opview/BruteForceSolverGenerator.hpp>

namespace opview {
    
    AdderInferencePtr BruteForceSolverGenerator::getOptimizerAlgorithm(GraphicalModelAdder &model, LabelList currentOptimal, size_t numVariables)
    {
        BruteforcePtr algorithm = new Bruteforce(model);
        
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
