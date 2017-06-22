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

        virtual void estimateBestCameraPosition();

    protected:
        // bring up overloaded function from parent
        using MultipointHierarchicalGraphicalModel::estimateBestCameraPosition;

    private:
        typedef MultipointHierarchicalGraphicalModel super;

    };


} // namespace opview

#endif // CAM_POSITION_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
