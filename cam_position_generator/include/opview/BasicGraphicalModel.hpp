#ifndef CAM_POSITION_BASIC_GRAPHICAL_MODEL_H
#define CAM_POSITION_BASIC_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtx/norm.hpp>

#include <opview/type_definition.h>
#include <opview/GraphicalModelBuilder.hpp>

namespace opview {


    #define coordinatecycles(X0, Xn, Y0, Yn, Z0, Zn) for(int x = X0; x < Xn; x++) for(int y = Y0; y < Yn; y++) for(int z = Z0; z < Zn; z++)

    #define LABELS 100
    #define VARS 3
    
    class BasicGraphicalModel : public GraphicalModelBuilder {
    public:
        BasicGraphicalModel(SolverGeneratorPtr solver, GLMVec3List &cams, double goalAngle=45, double dispersion=1);
        ~BasicGraphicalModel();

    protected:
        virtual void fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void fillConstraintFunction(GMSparseFunction &constraints, GMSparseFunction &distances, GLMVec3 &centroid);
        virtual void addValueToConstraintFunction(GMSparseFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3 &centroid);

        virtual LabelType logVonMises(GLMVec3 &point, GLMVec3 &centroid, GLMVec3 &normalVector, VonMisesConfigurationPtr config);
        virtual LabelType logVonMises(GLMVec3 &v, GLMVec3 &normalVector, VonMisesConfigurationPtr config);
        virtual LabelType logVonMises(double angle, VonMisesConfigurationPtr config);
        virtual LabelType logBessel0(double K);

        virtual GLMVec3 scalePoint(GLMVec3 point);
        
        void initShapes();
        virtual size_t numVariables();
        virtual size_t numLabels();
        
        std::function<float()> scale = [this](){ return 2.0 / (float)numLabels(); };
        std::function<float()> offsetX = [](){ return -1.0; };
        std::function<float()> offsetY = [](){ return -1.0; };
        std::function<float()> offsetZ = [](){ return -1.0; };

        std::vector<size_t> variableIndices;
        std::vector<size_t> shape;

    private:
        GLMVec3List cams;
        VonMisesConfiguration vonMisesConfig;
    };


} // namespace opview

#endif // CAM_POSITION_BASIC_GRAPHICAL_MODEL_H
