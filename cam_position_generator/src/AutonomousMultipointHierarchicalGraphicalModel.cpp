#include <opview/AutonomousMultipointHierarchicalGraphicalModel.hpp>

namespace opview {

    AutonomousMultipointHierarchicalGraphicalModel::AutonomousMultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                                CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                                size_t maxPoints, long double maxUncertainty, double goalAngle, double dispersion)
                    : MultipointHierarchicalGraphicalModel(solver, config, camConfig, meshConfig, maxPoints, maxUncertainty, goalAngle, dispersion)
    {
        getLogger()->resetFile("auto_multi.json");
    }
    
    AutonomousMultipointHierarchicalGraphicalModel::~AutonomousMultipointHierarchicalGraphicalModel()
    { /*    */ }

    void AutonomousMultipointHierarchicalGraphicalModel::estimateBestCameraPosition()
    {
        int pointIndex = getWorstPointsList()[0].second;
        this->estimateBestCameraPosition(getPoints()[pointIndex], getNormals()[pointIndex]);
    }

} // namespace opview
