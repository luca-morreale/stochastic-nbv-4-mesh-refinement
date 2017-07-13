#ifndef CAM_POSITION_GENERATOR_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_GENERATOR_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include <CGAL/exceptions.h>

#include <opview/HierarchicalDiscreteGraphicalModel.hpp>
#include <opview/type_definition.h>
#include <opview/ReportWriter.hpp>

namespace opview {

    #define orientationcycles(P0, Pn, YAW0, YAWn) for(int ptc = P0; ptc < Pn; ptc++) for(int yaw = YAW0; yaw < YAWn; yaw++)
    #define ORIENTATION_VARS 5
    
    class OrientationHierarchicalGraphicalModel : public HierarchicalDiscreteGraphicalModel {
    public:
        OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                    CameraGeneralConfiguration &camConfig, std::string meshFile, 
                                                    GLMVec3List &cams, double goalAngle=55, double dispersion=5);
        ~OrientationHierarchicalGraphicalModel();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

        virtual size_t numVariables() override;
        virtual size_t orientationLabels();
        float getDeltaAngle();

        std::string getMeshFilename();
        void setMeshFilename(std::string filename);


    protected:
        virtual LabelList extractResults(AdderInferencePtr algorithm) override;

        virtual void fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector) override;

        virtual void fillExplicitOrientationFunction(GMExplicitFunction &modelFunction, BoostObjFunction evals, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void computeDistributionForFunction(GMExplicitFunction &modelFunction, BoostObjFunction &evals, size_t coord[], GLMVec3 &centroid, GLMVec3 &normVector);

        virtual LabelType visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        virtual LabelType estimateObjDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        virtual LabelType imagePlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);

        virtual bool isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isOppositeView(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isIntersecting(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isMathemathicalError(Segment_intersection &intersection, Point &point);
        virtual bool isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid);

        virtual RotationMatrix getRotationMatrix(float roll, float pitch, float yaw);
        virtual CameraMatrix getCameraMatrix(EigVector5 &pose);
        virtual GLMVec2 getProjectedPoint(EigVector5 &pose, GLMVec3 &centroid);

        virtual void reduceScale(LabelList &currentOptimal, int depth);
        virtual void resetPosition() override;

        virtual EigVector5 getPose(GLMVec3 &scaledPos, GLMVec2 &scaledOri);

        virtual GLMVec2 scaleOrientation(GLMVec2 orientation);
        virtual GLMVec2 unscaleOrientation(GLMVec2 orientation);

        virtual void initShapes() override;

        virtual VarIndexList getOptimaForDiscreteSpace(LabelList &currentOptima) override;
        virtual void fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector) override;
        virtual void addValueToConstraintFunction(GMExplicitFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3 &centroid, size_t coords[]) override;

        ReportWriterPtr getLogger();
        void setLogger(ReportWriterPtr log);

        SizeTList coordinateIndices;
        SizeTList coordinateShape;

    private:
        float deltaAngle;
        const GLMVec3 zdir = GLMVec3(0.0, 0.0, 1.0);
        std::string meshFilename;
        TreePtr tree;

        CameraGeneralConfiguration camConfig;
        OrientationHierarchicalConfiguration orientConfig;

        ReportWriterPtr log;

        void fillTree();
        Polyhedron extractPolyhedron();
        TriangleList getTriangleList(Polyhedron &poly);

        typedef HierarchicalDiscreteGraphicalModel super;

    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
