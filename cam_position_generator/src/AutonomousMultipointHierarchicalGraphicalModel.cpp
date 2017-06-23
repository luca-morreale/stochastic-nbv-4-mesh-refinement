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

    LabelType AutonomousMultipointHierarchicalGraphicalModel::logVonMisesWrapper(GLMVec3 &point, GLMVec3 &centroid, GLMVec3 &normal)
    {
        EigVector5 pose;
        pose << point.x, point.y, point.z, 0.0, 0.0;
        return estimateForWorstPointSeen(pose, boost::bind(&AutonomousMultipointHierarchicalGraphicalModel::origianlLogVonMisesWrapper, this, _1, _2, _3));
    }

    LabelType AutonomousMultipointHierarchicalGraphicalModel::visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&AutonomousMultipointHierarchicalGraphicalModel::origianlVisibilityDistribution, this, _1, _2, _3));
    }

    LabelType AutonomousMultipointHierarchicalGraphicalModel::imagePlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&AutonomousMultipointHierarchicalGraphicalModel::origianlImagePlaneWeight, this, _1, _2, _3));
    }

    LabelType AutonomousMultipointHierarchicalGraphicalModel::origianlLogVonMisesWrapper(EigVector5 &point, GLMVec3 &centroid, GLMVec3 &normal)
    {
        GLMVec3 pose(point[0], point[1], point[2]);
        return -OrientationHierarchicalGraphicalModel::logVonMisesWrapper(pose, centroid, normal);
    }

    LabelType AutonomousMultipointHierarchicalGraphicalModel::origianlVisibilityDistribution(EigVector5 &point, GLMVec3 &centroid, GLMVec3 &normal)
    {
        return -OrientationHierarchicalGraphicalModel::visibilityDistribution(point, centroid, normal);
    }

    LabelType AutonomousMultipointHierarchicalGraphicalModel::origianlImagePlaneWeight(EigVector5 &point, GLMVec3 &centroid, GLMVec3 &normal)
    {
        return -OrientationHierarchicalGraphicalModel::imagePlaneWeight(point, centroid, normal);
    }

} // namespace opview
