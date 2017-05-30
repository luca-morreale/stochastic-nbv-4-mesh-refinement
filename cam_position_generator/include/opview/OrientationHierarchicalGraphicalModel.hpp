#ifndef CAM_POSITION_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <cmath>
#include <cstdlib>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>

#include <opview/HierarchicalDiscreteGraphicalModel.hpp>
#include <opview/type_definition.h>

namespace opview {

    #define orientationcycles(P0, Pn, YAW0, YAWn) for(int ptc = P0; ptc < Pn; ptc++) for(int yaw = YAW0; yaw < YAWn; yaw++)
    #define ORIENTATION_VARS 5
    
    class OrientationHierarchicalGraphicalModel : public HierarchicalDiscreteGraphicalModel {
    public:
        OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                            std::string meshFile, GLMVec3List &cams, double goalAngle=55, double dispersion=5);
        ~OrientationHierarchicalGraphicalModel();

        virtual size_t numVariables() override;
        virtual size_t orientationLabels();
        float getDeltaAngle();

        std::string getMeshFilename();
        void setMeshFilename(std::string filename);


    protected:
        virtual void fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector) override;
        virtual void fillObjectiveFunction(GMSparseFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual LabelType computeObjectiveFunction(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);

        virtual bool isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isOppositeView(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isIntersecting(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid);

        virtual RotationMatrix getRotationMatrix(float roll, float pitch, float yaw);
        virtual CameraMatrix getCameraMatrix(EigVector5 &pose);

        virtual EigVector5 getPose(GLMVec3 &scaledPos, GLMVec2 &scaledOri);

        virtual GLMVec2 scaleOrientation(GLMVec2 orientation);


    private:
        float deltaAngle;
        const GLMVec3 zdir = GLMVec3(0.0, 0.0, 1.0);
        std::string meshFilename;
        Tree *tree;

        void fillTree();
        Polyhedron extractPolyhedron();
        TriangleList getTriangleList(Polyhedron &poly);

    };


} // namespace opview

#endif // CAM_POSITION_ORIENTATION_HIERARCHICAL_GRAPHICAL_MODEL_H
