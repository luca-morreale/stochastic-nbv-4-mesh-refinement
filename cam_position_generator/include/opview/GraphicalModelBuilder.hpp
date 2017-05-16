#ifndef CAM_POSITION_GENERATOR_MODEL_BUILDER_H
#define CAM_POSITION_GENERATOR_MODEL_BUILDER_H

#include <cmath>
#include <cstdlib>

#include <glm/gtx/norm.hpp>

#include <opview/type_definition.h>

namespace opview {


    #define coordinatecycles(X0, Xn, Y0, Yn, Z0, Zn) for(int x = X0; x < Xn; x++) for(int y = Y0; y < Yn; y++) for(int z = Z0; z < Zn; z++)

    
    class GraphicalModelBuilder {
    public:
        GraphicalModelBuilder(GLMVec3List &cams, double goalAngle = 45, double dispersion = 1);
        ~GraphicalModelBuilder();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

    protected:

        virtual void buildModel(GraphicalModelAdder &model, GMExplicitFunction &vonMises, GMSparseFunction &constraints, GMSparseFunction &distances, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void fillConstraintFunction(GMSparseFunction &constraints, GMSparseFunction &distances, GLMVec3 &centroid);
        virtual void addValueToConstraintFunction(GMSparseFunction &function, GLMVec3 point, GLMVec3 &cam, GLMVec3 &centroid);

        virtual LabelType logVonMises(GLMVec3 point, GLMVec3 &centroid, GLMVec3 &normalVector, VonMisesConfigurationPtr config);
        virtual LabelType logVonMises(GLMVec3 &point, GLMVec3 &centroid, GLMVec3 &normalVector, VonMisesConfigurationPtr config);
        virtual LabelType logVonMises(GLMVec3 &v, GLMVec3 &normalVector, VonMisesConfigurationPtr config);
        virtual LabelType logVonMises(double angle, VonMisesConfigurationPtr config);
        virtual LabelType logBessel0(double K);
        

    private:
        GLMVec3List cams;
        VonMisesConfiguration vonMisesConfig;

        const int numVariables = 3;
        std::vector<size_t> variableIndices;
        const int numLabels = 200;
        std::vector<size_t> shape;

        const int startX = 0;
        const int endX = 200;
        const int startY = 0;
        const int endY = 200;
        const int startZ = 0;
        const int endZ = 200;

        void initShapes();
        void addFunctionTo(GMExplicitFunction &fun, GraphicalModelAdder &model);
        void addFunctionTo(GMSparseFunction &fun, GraphicalModelAdder &model);
        
    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_MODEL_BUILDER_H
