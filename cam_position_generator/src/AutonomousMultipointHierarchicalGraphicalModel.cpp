#include <opview/AutonomousMultipointHierarchicalGraphicalModel.hpp>

namespace opview {

    AutonomousMultipointHierarchicalGraphicalModel::AutonomousMultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                                CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                                size_t maxPoints, long double maxUncertainty, double goalAngle, double dispersion)
                    : MultipointHierarchicalGraphicalModel(solver, config, camConfig, meshConfig, maxPoints, maxUncertainty, goalAngle, dispersion)
    {
        setWorstPoint();
    }
    
    AutonomousMultipointHierarchicalGraphicalModel::~AutonomousMultipointHierarchicalGraphicalModel()
    { /*    */ }

    void AutonomousMultipointHierarchicalGraphicalModel::updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double accuracy)
    {
        super::updateMeshInfo(pointIndex, point, normal, accuracy);
        updateWorstReference(pointIndex, accuracy);
    }

    void AutonomousMultipointHierarchicalGraphicalModel::updateMeshInfo(int pointIndex, GLMVec3 normal, double accuracy)
    {
        super::updateMeshInfo(pointIndex, normal, accuracy);
        updateWorstReference(pointIndex, accuracy);
    }

    void AutonomousMultipointHierarchicalGraphicalModel::updateMeshInfo(int pointIndex, double accuracy)
    {
        super::updateMeshInfo(pointIndex, accuracy);
        updateWorstReference(pointIndex, accuracy);
    }

    void AutonomousMultipointHierarchicalGraphicalModel::addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        super::addPoint(point, normal, uncertainty);
        updateWorstReference(this->getPoints().size()-1, uncertainty);
    }

    void AutonomousMultipointHierarchicalGraphicalModel::updateWorstReference(int pointIndex, double accuracy)
    {
        if (accuracy > worstUncertainty) {
            worstUncertainty = accuracy;
            worstPointIndex = pointIndex;
        }
    }

    int AutonomousMultipointHierarchicalGraphicalModel::checkWorstPoint()
    {
        DoubleList uncertainties = getUncertainties();
        int worstPointIndex = 0;
        for (int p = 0; p < getPoints().size(); p++) {
            if (uncertainties[worstPointIndex] < uncertainties[p]) {
                worstPointIndex = p;
            }
        }
        return worstPointIndex;
    }

    void AutonomousMultipointHierarchicalGraphicalModel::setWorstPoint()
    {
        this->worstPointIndex = this->checkWorstPoint();
        this->worstUncertainty = this->getUncertainties()[worstPointIndex];
    }

    void AutonomousMultipointHierarchicalGraphicalModel::estimateBestCameraPosition()
    {
        GLMVec3 worstCentroid = this->getPoints()[this->worstPointIndex];
        GLMVec3 normal = this->getNormals()[this->worstPointIndex];
        this->estimateBestCameraPosition(worstCentroid, normal);
    }

} // namespace opview
