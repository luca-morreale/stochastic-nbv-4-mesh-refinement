#ifndef CAM_POSITION_BASIC_GRAPHICAL_MODEL_H
#define CAM_POSITION_BASIC_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtx/norm.hpp>

#include <opview/GraphicalModelBuilder.hpp>
#include <opview/type_definition.h>
#include <opview/utilities.hpp>

namespace opview {

    #define coordinatecycles(X0, Xn, Y0, Yn, Z0, Zn) for(int x = X0; x < Xn; x++) for(int y = Y0; y < Yn; y++) for(int z = Z0; z < Zn; z++)

    #define LABELS 100
    #define VARS 3
    #define BD_AERIAL 0.2f
    #define BD_TERRESTRIAL_PROSPECTIVE 0.25f
    #define BD_TERRESTRIAL_ARCHITECTURAL 0.33f 
    
    class BasicGraphicalModel : public GraphicalModelBuilder {
    public:
        BasicGraphicalModel(SolverGeneratorPtr solver, GLMVec3List &cams, double goalAngle=45, double dispersion=1);
        ~BasicGraphicalModel();

        GLMVec3List getCams();
        void setCams(GLMVec3List &cams);

        VonMisesConfigurationPtr vonMisesConfiguration();
        void setVonMisesConfiguration(VonMisesConfiguration vonMisesConfig);

        virtual size_t numVariables();
        virtual size_t numLabels();

    protected:
        virtual void fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector) override;
        virtual void fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void fillConstraintFunction(GMExplicitFunction &constraints, GLMVec3 &centroid);
        virtual void addValueToConstraintFunction(GMExplicitFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3 &centroid, size_t coords[]);

        virtual GLMVec3 scalePoint(GLMVec3 point);
        virtual GLMVec3 unscalePoint(GLMVec3 point);

        virtual double logVonMisesWrapper(GLMVec3 &pos, GLMVec3 &centroid, GLMVec3 &normVector);
        
        virtual void initShapes();
        
        
        LambdaFloat scale = [this](){ return 1.5 / (float)numLabels(); };
        LambdaFloat offsetX = [](){ return -1.0; };
        LambdaFloat offsetY = [](){ return -1.0; };
        LambdaFloat offsetZ = [](){ return -1.0; };

        SizeTList variableIndices;
        SizeTList shape;

    private:
        GLMVec3List cams;
        VonMisesConfiguration vonMisesConfig;
    };


} // namespace opview

#endif // CAM_POSITION_BASIC_GRAPHICAL_MODEL_H
