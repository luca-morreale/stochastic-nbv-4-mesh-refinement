#include <opview/FlipperSolverGenerator.hpp>

namespace opview {
    
    AdderInferencePtr FlipperSolverGenerator::getOptimizerAlgorithm(GraphicalModelAdder &model, LabelList currentOptimal, size_t numVariables)
    {
        size_t maxSubgraphSize = 3; // works only if it is 1

        LabelList startPoint = (currentOptimal.size() > 0) ? currentOptimal : genStartPoint(numVariables);
        LazyFlipperParameter para(maxSubgraphSize, startPoint.begin(), startPoint.end());
        LazyFlipperPtr algorithm = new LazyFlipper(model, para);
        return algorithm;
    }
    
    /*MultiplierInferencePtr FlipperSolverGenerator::getOptimizerAlgorithm(GraphicalModelMultiplier &model, LabelList currentOptimal, size_t numVariables)
    {
        size_t maxSubgraphSize = 1; // works only if it is 1

        LabelList startPoint = (currentOptimal.size() > 0) ? currentOptimal : genStartPoint(numVariables);
        LazyFlipperParameter para(maxSubgraphSize, startPoint.begin(), startPoint.end());
        //LazyFlipperPtr algorithm = new LazyFlipper(model, para);

        return NULL;
    }*/

} // namespace opview
