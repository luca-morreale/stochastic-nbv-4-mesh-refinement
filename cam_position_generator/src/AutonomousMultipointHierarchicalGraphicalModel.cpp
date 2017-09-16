#include <opview/AutonomousMultipointHierarchicalGraphicalModel.hpp>

namespace opview {

    AutonomousMultipointHierarchicalGraphicalModel::AutonomousMultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                                CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                                size_t maxPoints, double goalAngle, double dispersion)
                    : MultipointHierarchicalGraphicalModel(solver, config, camConfig, meshConfig.filename, meshConfig.cams, goalAngle, dispersion)
    {
        this->points = meshConfig.points;
        this->normals = meshConfig.normals;
        this->uncertainty = meshConfig.uncertainty;

        this->maxPoints = maxPoints;

        // precomputeSumUncertainty();
        setupWorstPoints();
        
        getLogger()->resetFile("auto_multi.json");
    }
    
    AutonomousMultipointHierarchicalGraphicalModel::~AutonomousMultipointHierarchicalGraphicalModel()
    { /*    */ }

    LabelList AutonomousMultipointHierarchicalGraphicalModel::estimateBestCameraPosition()
    {
        GLMVec3ListPair worstCentroids = getWorstPointsList();
        return this->estimateBestCameraPosition(worstCentroids.first, worstCentroids.second);
    }

    // double AutonomousMultipointHierarchicalGraphicalModel::computeWeightForPoint(int pointIndex)
    // {
    //     return (double)uncertainty[pointIndex] / (double)SUM_UNCERTAINTY;
    // }

    void AutonomousMultipointHierarchicalGraphicalModel::updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points[pointIndex] = point;
        updateMeshInfo(pointIndex, normal, uncertainty);
    }

    void AutonomousMultipointHierarchicalGraphicalModel::updateMeshInfo(int pointIndex, GLMVec3 normal, double uncertainty)
    {
        this->normals[pointIndex] = normal;
        updateMeshInfo(pointIndex, uncertainty);
    }

    void AutonomousMultipointHierarchicalGraphicalModel::updateMeshInfo(int pointIndex, double uncertainty)
    {
        // SUM_UNCERTAINTY += this->uncertainty[pointIndex] - uncertainty;
        this->uncertainty[pointIndex] = uncertainty;
        updateWorstPoints(pointIndex, uncertainty);
    }

    void AutonomousMultipointHierarchicalGraphicalModel::addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points.push_back(point);
        this->normals.push_back(normal);
        this->uncertainty.push_back(uncertainty);
        // SUM_UNCERTAINTY += uncertainty;
    }

    // void AutonomousMultipointHierarchicalGraphicalModel::precomputeSumUncertainty()
    // {
    //     SUM_UNCERTAINTY = 0.0;
    //     for (int p = 0; p < this->uncertainty.size(); p++) {
    //         SUM_UNCERTAINTY += uncertainty[p];
    //     }
    // }

    void AutonomousMultipointHierarchicalGraphicalModel::setupWorstPoints()
    {
        worstPointsList.clear();
        for (int p = 0; p < points.size(); p++) {
            worstPointsList.push_back(std::make_pair(uncertainty[p], p));
        }
        retainWorst();
    }

    void AutonomousMultipointHierarchicalGraphicalModel::updateWorstPoints(int index, long double uncertainty)
    {
        worstPointsList.push_back(std::make_pair(uncertainty, index));
        retainWorst();
    }

    void AutonomousMultipointHierarchicalGraphicalModel::retainWorst()
    {
        std::sort(worstPointsList.rbegin(), worstPointsList.rend());
        int eraseFrom = std::min(this->maxPoints, worstPointsList.size());
        worstPointsList.erase(worstPointsList.begin() + eraseFrom, worstPointsList.end());
    }

    GLMVec3ListPair AutonomousMultipointHierarchicalGraphicalModel::getWorstPointsList()
    {
        GLMVec3List worstPoints;
        GLMVec3List worstNormals;

        for (int i = 0; i < worstPointsList.size(); i++) {
            int index = worstPointsList[i].second;
            worstPoints.push_back(this->points[index]);
            worstNormals.push_back(this->normals[index]);
        }

        return std::make_pair(worstPoints, worstNormals);
    }

    GLMVec3List AutonomousMultipointHierarchicalGraphicalModel::getPoints()
    {
        return points;
    }
    GLMVec3List AutonomousMultipointHierarchicalGraphicalModel::getNormals()
    {
        return normals;
    }
    DoubleList AutonomousMultipointHierarchicalGraphicalModel::getUncertainties()
    {
        return uncertainty;
    }
    

} // namespace opview
