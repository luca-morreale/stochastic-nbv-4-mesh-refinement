#ifndef CAM_POSITION_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <opview/MultipointHierarchicalGraphicalModel.hpp>
#include <opview/type_definition.h>

namespace opview {
    
    class AutonomousMultipointHierarchicalGraphicalModel : public MultipointHierarchicalGraphicalModel {
    public:
        AutonomousMultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                        CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                        size_t maxPoints, long double maxUncertainty, double goalAngle=55, double dispersion=5);
        ~AutonomousMultipointHierarchicalGraphicalModel();

        virtual void updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double accuracy) override;
        virtual void updateMeshInfo(int pointIndex, GLMVec3 normal, double accuracy) override;
        virtual void updateMeshInfo(int pointIndex, double accuracy) override;
        virtual void addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty) override;

        virtual void estimateBestCameraPosition();

    protected:
        virtual int checkWorstPoint();
        virtual void setWorstPoint();

        // bring up overloaded function from parent
        using MultipointHierarchicalGraphicalModel::estimateBestCameraPosition;
        
        virtual void updateWorstReference(int pointIndex, double accuracy);

    private:
        long double worstUncertainty;
        int worstPointIndex;

        typedef MultipointHierarchicalGraphicalModel super;

    };


} // namespace opview

#endif // CAM_POSITION_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
