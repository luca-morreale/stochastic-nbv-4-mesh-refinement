#ifndef CAM_POSITION_GENERATOR_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_GENERATOR_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <opview/MultipointHierarchicalGraphicalModel.hpp>
#include <opview/type_definition.h>

namespace opview {
    
    class AutonomousMultipointHierarchicalGraphicalModel : public MultipointHierarchicalGraphicalModel {
    public:
        AutonomousMultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                        CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                        size_t maxPoints, long double maxUncertainty, double goalAngle=55, double dispersion=5);
        ~AutonomousMultipointHierarchicalGraphicalModel();

        virtual void estimateBestCameraPosition();

    protected:
        // bring up overloaded function from parent
        using MultipointHierarchicalGraphicalModel::estimateBestCameraPosition;

        virtual LabelType logVonMisesWrapper(GLMVec3 &point, GLMVec3 &centroid, GLMVec3 &normal) override;
        virtual LabelType visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector) override;
        virtual LabelType imagePlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector) override;

    private:
        LabelType origianlLogVonMisesWrapper(EigVector5 &point, GLMVec3 &centroid, GLMVec3 &normal);
        LabelType origianlVisibilityDistribution(EigVector5 &point, GLMVec3 &centroid, GLMVec3 &normal);
        LabelType origianlImagePlaneWeight(EigVector5 &point, GLMVec3 &centroid, GLMVec3 &normal);

        typedef MultipointHierarchicalGraphicalModel super;
    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
