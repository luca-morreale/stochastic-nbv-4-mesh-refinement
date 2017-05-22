#include <opview/LOCSolverGenerator.hpp>

namespace opview {
    
    AdderInferencePtr LOCSolverGenerator::getOptimizerAlgorithm(GraphicalModelAdder &model, LabelList currentOptimal, size_t numVariables)
    {
        LabelList startPoint = (currentOptimal.size() > 0) ? currentOptimal : genStartPoint(numVariables);
        LOC::Parameter parameter(solver, phi, maxBlockRadius, maxTreeRadius, pFastHeuristic, maxIterations, 
                                    stopAfterNBadIterations, maxBlockSize, maxTreeSize, treeRuns);
        LOCPtr algorithm = new LOC(model, parameter);
        algorithm->setStartingPoint(startPoint.begin());
        
        return algorithm;
    }
    
   /* MultiplierInferencePtr LOCSolverGenerator::getOptimizerAlgorithm(GraphicalModelMultiplier &model, LabelList currentOptimal, size_t numVariables)
    {
        LabelList startPoint = (currentOptimal.size() > 0) ? currentOptimal : genStartPoint(numVariables);
        LOCParameter parameter(solver, phi, maxBlockRadius, maxTreeRadius, pFastHeuristic, maxIterations, 
                                    stopAfterNBadIterations, maxBlockSize, maxTreeSize, treeRuns);
        //LOCPtr algorithm = new LOC(model, parameter);
        
        return NULL;
    }*/

} // namespace opview
