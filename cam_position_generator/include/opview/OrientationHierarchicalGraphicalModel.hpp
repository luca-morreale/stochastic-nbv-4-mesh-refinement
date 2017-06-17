#ifndef CAM_POSITION_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include <opview/HierarchicalDiscreteGraphicalModel.hpp>
#include <opview/type_definition.h>

namespace opview {

    #define orientationcycles(P0, Pn, YAW0, YAWn) for(int ptc = P0; ptc < Pn; ptc++) for(int yaw = YAW0; yaw < YAWn; yaw++)
    #define ORIENTATION_VARS 5
    
    class OrientationHierarchicalGraphicalModel : public HierarchicalDiscreteGraphicalModel {
    public:
        OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                    CameraGeneralConfiguration &camConfig, std::string meshFile, 
                                                    GLMVec3List &cams, double goalAngle=55, double dispersion=5);
        ~OrientationHierarchicalGraphicalModel();

        virtual size_t numVariables() override;
        virtual size_t orientationLabels();
        float getDeltaAngle();

        std::string getMeshFilename();
        void setMeshFilename(std::string filename);


    protected:
        virtual LabelList extractResults(AdderInferencePtr algorithm) override;

        virtual void fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector) override;

        virtual void fillExplicitOrientationFunctions(GMExplicitFunctionList &modelFunctions, BoostObjFunctionList &evals, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void computeDistributionForFunctions(GMExplicitFunctionList &modelFunctions, BoostObjFunctionList &evals, size_t coord[], GLMVec3 &centroid, GLMVec3 &normVector);

        virtual LabelType visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        virtual LabelType estimateObjDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        virtual LabelType imagePlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);

        virtual bool isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isOppositeView(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isIntersecting(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid);

        virtual RotationMatrix getRotationMatrix(float roll, float pitch, float yaw);
        virtual CameraMatrix getCameraMatrix(EigVector5 &pose);
        virtual GLMVec2 getProjectedPoint(EigVector5 &pose, GLMVec3 &centroid);

        virtual EigVector5 getPose(GLMVec3 &scaledPos, GLMVec2 &scaledOri);

        virtual GLMVec2 scaleOrientation(GLMVec2 orientation);

        virtual void initShapes() override;

        SizeTList coordinateIndices;
        SizeTList coordinateShape;


    private:
        float deltaAngle;
        const GLMVec3 zdir = GLMVec3(0.0, 0.0, 1.0);
        std::string meshFilename;
        TreePtr tree;

        CameraGeneralConfiguration camConfig;

        void fillTree();
        Polyhedron extractPolyhedron();
        TriangleList getTriangleList(Polyhedron &poly);

        typedef HierarchicalDiscreteGraphicalModel super;

    };


} // namespace opview

#endif // CAM_POSITION_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
