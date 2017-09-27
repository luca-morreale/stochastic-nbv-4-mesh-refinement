#include <opview/AutonomousStochasticMethod.hpp>

namespace opview {

    AutonomousStochasticMethod::AutonomousStochasticMethod(CameraGeneralConfiguration &camConfig, 
                MeshConfiguration &meshConfig, StochasticConfiguration &stoConfig, size_t maxPoints, double offspring, double goalAngle, double dispersion)
                : StochasticMethod(camConfig, meshConfig.filename, meshConfig.cams, stoConfig, offspring, goalAngle, dispersion)
    {
        this->points = meshConfig.points;
        this->normals = meshConfig.normals;
        this->uncertainty = meshConfig.uncertainty;
        this->maxPoints = maxPoints;

        setupWorstPoints();

        this->getLogger()->resetFile("auto_stochastic.json");
    }

    AutonomousStochasticMethod::~AutonomousStochasticMethod()
    { /*    */ }

    DoubleList AutonomousStochasticMethod::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        throw NotImplementedMethodException("Method not implemented in Autonomous version");
    }


    OrderedPose AutonomousStochasticMethod::uniformSamplingStep(GLMVec3List &centroids, GLMVec3List &normals, int round)
    {
        EigVector5List orientedPoints = uniformPointsGetter();
        
        OrderedPose poses = computeEnergyForPoses(centroids, normals, orientedPoints);
        
        return this->extractBestResults(poses, round);
    }

    OrderedPose AutonomousStochasticMethod::computeEnergyForPoses(GLMVec3List &centroids, GLMVec3List &normals, EigVector5List &orientedPoints)
    {
        DoubleList values(orientedPoints.size());

        #pragma omp parallel for
        for (int i = 0; i < orientedPoints.size(); i++) {
            values[i] = getFormulation()->computeEnergy(orientedPoints[i], centroids, normals);
        }
        
        return orderPoses(orientedPoints, values);
    }

    void AutonomousStochasticMethod::updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points[pointIndex] = point;
        updateMeshInfo(pointIndex, normal, uncertainty);
    }

    void AutonomousStochasticMethod::updateMeshInfo(int pointIndex, GLMVec3 normal, double uncertainty)
    {
        this->normals[pointIndex] = normal;
        updateMeshInfo(pointIndex, uncertainty);
    }

    void AutonomousStochasticMethod::updateMeshInfo(int pointIndex, double uncertainty)
    {
        this->uncertainty[pointIndex] = uncertainty;
        updateWorstPoints(pointIndex, uncertainty);
    }

    void AutonomousStochasticMethod::addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points.push_back(point);
        this->normals.push_back(normal);
        this->uncertainty.push_back(uncertainty);
    }

    void AutonomousStochasticMethod::setupWorstPoints()
    {
        worstPointsList.clear();
        for (int p = 0; p < points.size(); p++) {
            worstPointsList.push_back(std::make_pair(uncertainty[p], p));
        }
        retainWorst();
    }

    void AutonomousStochasticMethod::updateWorstPoints(int index, long double uncertainty)
    {
        worstPointsList.push_back(std::make_pair(uncertainty, index));
        retainWorst();
    }

    void AutonomousStochasticMethod::retainWorst()
    {
        std::sort(worstPointsList.rbegin(), worstPointsList.rend());
        int eraseFrom = std::min(this->maxPoints, worstPointsList.size());
        worstPointsList.erase(worstPointsList.begin() + eraseFrom, worstPointsList.end());
    }

    GLMVec3ListPair AutonomousStochasticMethod::getWorstPointsList()
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

    GLMVec3List AutonomousStochasticMethod::getPoints()
    {
        return points;
    }

    GLMVec3List AutonomousStochasticMethod::getNormals()
    {
        return normals;
    }
    
    DoubleList AutonomousStochasticMethod::getUncertainties()
    {
        return uncertainty;
    }

} // namespace opview
