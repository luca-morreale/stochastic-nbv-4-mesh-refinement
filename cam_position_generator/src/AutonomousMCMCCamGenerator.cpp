#include <opview/AutonomousMCMCCamGenerator.hpp>

namespace opview {

    AutonomousMCMCCamGenerator::AutonomousMCMCCamGenerator(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                            MCConfiguration &mcConfig, double goalAngle, double dispersion)
                                                            : MCMCCamGenerator(camConfig, meshConfig.filename, meshConfig.cams, mcConfig, goalAngle, dispersion)
    {
        this->points = meshConfig.points;
        this->normals = meshConfig.normals;
        this->uncertainty = meshConfig.uncertainty;

        SUM_UNCERTAINTY = 0.0;
        for (int p = 0; p < uncertainty.size(); p++) {
            SUM_UNCERTAINTY += uncertainty[p];
        }

        setWorstPoint();

        this->getLogger()->resetFile("auto_mcmc.json");
    }
    
    AutonomousMCMCCamGenerator::~AutonomousMCMCCamGenerator()
    { /*    */ }

    void AutonomousMCMCCamGenerator::precomputeSumUncertainty()
    {
        SUM_UNCERTAINTY = 0.0;
        for (int p = 0; p < uncertainty.size(); p++) {
            SUM_UNCERTAINTY += uncertainty[p];
        }
    }

    void AutonomousMCMCCamGenerator::updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points[pointIndex] = point;
        updateMeshInfo(pointIndex, normal, uncertainty);
        updateWorstReference(pointIndex, uncertainty);
    }

    void AutonomousMCMCCamGenerator::updateMeshInfo(int pointIndex, GLMVec3 normal, double uncertainty)
    {
        this->normals[pointIndex] = normal;
        updateMeshInfo(pointIndex, uncertainty);
        updateWorstReference(pointIndex, uncertainty);
    }

    void AutonomousMCMCCamGenerator::updateMeshInfo(int pointIndex, double uncertainty)
    {
        SUM_UNCERTAINTY += this->uncertainty[pointIndex] - uncertainty;
        this->uncertainty[pointIndex] = uncertainty;
        updateWorstReference(pointIndex, uncertainty);
    }

    void AutonomousMCMCCamGenerator::addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points.push_back(point);
        this->normals.push_back(normal);
        this->uncertainty.push_back(uncertainty);
        SUM_UNCERTAINTY += uncertainty;
        updateWorstReference(this->points.size()-1, uncertainty);
    }

    void AutonomousMCMCCamGenerator::updateWorstReference(int pointIndex, double uncertainty)
    {
        if (uncertainty > worstUncertainty) {
            worstUncertainty = uncertainty;
            worstPointIndex = pointIndex;
        }
    }

    int AutonomousMCMCCamGenerator::checkWorstPoint()
    {
        DoubleList uncertainties = uncertainty;
        int worstPointIndex = 0;
        for (int p = 0; p < points.size(); p++) {
            if (uncertainties[worstPointIndex] < uncertainties[p]) {
                worstPointIndex = p;
            }
        }
        return worstPointIndex;
    }

    void AutonomousMCMCCamGenerator::setWorstPoint()
    {
        this->worstPointIndex = this->checkWorstPoint();
        this->worstUncertainty = this->uncertainty[worstPointIndex];
    }

    void AutonomousMCMCCamGenerator::estimateBestCameraPosition()
    {
        GLMVec3 worstCentroid = this->points[this->worstPointIndex];
        GLMVec3 normal = this->normals[this->worstPointIndex];
        this->estimateBestCameraPosition(worstCentroid, normal);
    }

    GLMVec3List AutonomousMCMCCamGenerator::getPoints()
    {
        return points;
    }
    GLMVec3List AutonomousMCMCCamGenerator::getNormals()
    {
        return normals;
    }
    DoubleList AutonomousMCMCCamGenerator::getUncertainties()
    {
        return uncertainty;
    }

} // namespace opview
