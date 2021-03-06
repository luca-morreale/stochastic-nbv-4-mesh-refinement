#ifndef CAM_POSITION_GENERATOR_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_GENERATOR_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtx/string_cast.hpp>

#include <CGAL/exceptions.h>

#include <opview/HierarchicalDiscreteGraphicalModel.hpp>
#include <opview/type_definition.h>
#include <opview/ReportWriter.hpp>
#include <opview/orientation_utilities.hpp>

namespace opview {

    #define orientationcycles(P0, Pn, YAW0, YAWn) for(int ptc = P0; ptc < Pn; ptc++) for(int yaw = YAW0; yaw < YAWn; yaw++)
    #define ORIENTATION_VARS 5
    
    class OrientationHierarchicalGraphicalModel : public HierarchicalDiscreteGraphicalModel {
    public:
        OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                    CameraGeneralConfiguration &camConfig, std::string meshFile, 
                                                    GLMVec3List &cams, double goalAngle=55, double dispersion=5);
        ~OrientationHierarchicalGraphicalModel();

        virtual LabelList estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

        virtual size_t numVariables() override;
        virtual size_t orientationLabels();
        float getDeltaAngle();

        std::string getMeshFilename();
        void setMeshFilename(std::string filename);

        CameraGeneralConfigPtr getCamConfig();
        TreePtr getTree();


    protected:
    
        virtual LabelList extractResults(AdderInferencePtr algorithm) override;

        virtual void fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector) override;

        virtual void fillExplicitOrientationFunction(GMExplicitFunction &modelFunction, BoostObjFunction evals, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void computeDistributionForFunction(GMExplicitFunction &modelFunction, BoostObjFunction &evals, size_t coord[], GLMVec3 &centroid, GLMVec3 &normVector);

        virtual void reduceScale(LabelList &currentOptimal, int depth);
        virtual void resetPosition() override;

        virtual EigVector5 getPose(GLMVec3 &scaledPos, GLMVec2 &scaledOri);

        virtual GLMVec2 scaleOrientation(GLMVec2 orientation);
        virtual GLMVec2 unscaleOrientation(GLMVec2 orientation);

        virtual void initShapes() override;

        virtual VarIndexList getOptimaForDiscreteSpace(LabelList &currentOptima) override;
        virtual void fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector) override;
        virtual void fillConstraintFunction(GMExplicitFunction &constraints, GLMVec3 &centroid) override;
    
        ReportWriterPtr getLogger();
        void setLogger(ReportWriterPtr log);

        SizeTList coordinateIndices;
        SizeTList coordinateShape;

    private:

        float deltaAngle;
        
        std::string meshFilename;
        TreePtr tree;

        CameraGeneralConfiguration camConfig;
        OrientationHierarchicalConfiguration orientConfig;

        TriangleList triangles;

        ReportWriterPtr log;

        void fillTree();
        Polyhedron extractPolyhedron();
        TriangleList getTriangleList(Polyhedron &poly);

        typedef HierarchicalDiscreteGraphicalModel super;

    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
