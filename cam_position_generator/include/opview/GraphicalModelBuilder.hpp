#ifndef CAM_POSITION_GENERATOR_GRAPHICAL_MODEL_BUILDER_H
#define CAM_POSITION_GENERATOR_GRAPHICAL_MODEL_BUILDER_H

#include <cmath>

#include <glm/gtx/norm.hpp>

#include <opengm/inference/inference.hxx>

#include <opview/type_definition.h>
#include <opview/SolverGenerator.hpp>

namespace opview {
    
    class GraphicalModelBuilder {
    public:
        GraphicalModelBuilder(SolverGeneratorPtr solverGen);
        ~GraphicalModelBuilder();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

        virtual size_t numVariables() = 0;
        virtual size_t numLabels() = 0;

        SolverGeneratorPtr solverGenerator();
        void setSolverGenerator(SolverGeneratorPtr solver);

    protected:
        virtual LabelList extractResults(AdderInferencePtr algorithm);
        

        virtual void fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector) = 0;

        void addFunctionTo(GMExplicitFunction &fun, GraphicalModelAdder &model, VarIndexList &variableIndices);
        void addFunctionTo(GMSparseFunction &fun, GraphicalModelAdder &model, VarIndexList &variableIndices);  

    private:
        SolverGeneratorPtr solver;
        
    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_MODEL_BUILDER_H
