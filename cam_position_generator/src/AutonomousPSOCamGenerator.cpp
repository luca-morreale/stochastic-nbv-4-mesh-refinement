#include <opview/AutonomousPSOCamGenerator.hpp>


namespace opview {

    AutonomousPSOCamGenerator::AutonomousPSOCamGenerator(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                            MCConfiguration &config, size_t maxPoints, long double maxUncertainty, double goalAngle, double dispersion)
                                                            : PSOCamGenerator(camConfig, meshConfig.filename, meshConfig.cams, config, goalAngle, dispersion)
    {
        this->points = meshConfig.points;
        this->normals = meshConfig.normals;
        this->uncertainty = meshConfig.uncertainty;
        this->maxPoints = maxPoints;
        this->maxUncertainty = maxUncertainty;

        precomputeSumUncertainty();
        setupWorstPoints();

        this->getLogger()->resetFile("auto_pso.json");
    }
    
    AutonomousPSOCamGenerator::~AutonomousPSOCamGenerator()
    { /*    */ }

    void AutonomousPSOCamGenerator::precomputeSumUncertainty()
    {
        SUM_UNCERTAINTY = 0.0;
        for (int p = 0; p < uncertainty.size(); p++) {
            SUM_UNCERTAINTY += uncertainty[p];
        }
    }

    double AutonomousPSOCamGenerator::computeWeightForPoint(int pointIndex)
    {
        return (double)uncertainty[pointIndex] / (double)SUM_UNCERTAINTY;
    }

    DoubleList AutonomousPSOCamGenerator::estimateBestCameraPosition()
    {
        size_t worstPointIndex = worstPointsList[0].second;

        GLMVec3 worstCentroid = this->points[worstPointIndex];
        GLMVec3 normal = this->normals[worstPointIndex];
        // std::cout << worstCentroid.x << " " << worstCentroid.y << " " << worstCentroid.z << std::endl;
        return this->estimateBestCameraPosition(worstCentroid, normal);
    }

    LabelType AutonomousPSOCamGenerator::logVonMisesWrapper(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normal)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&AutonomousPSOCamGenerator::parentCatllToLogVonMises, this, _1, _2, _3));
    }

    LabelType AutonomousPSOCamGenerator::visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&AutonomousPSOCamGenerator::parentCallToVisibilityEstimation, this, _1, _2, _3));
    }

    LabelType AutonomousPSOCamGenerator::imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&AutonomousPSOCamGenerator::parentCallToPlaneDistribution, this, _1, _2, _3));
    }

    double AutonomousPSOCamGenerator::estimateForWorstPointSeen(EigVector5 &pose, BoostObjFunction function)
    {
        LabelType val = 0.0;
        #pragma omp parallel for
        for (int i = 0; i < worstPointsList.size(); i++) {
            DoubleIntPair el = worstPointsList[i];
            double normalizedWeight = computeWeightForPoint(el.second);     // more the uncertainty is high more important it is seen
            LabelType local_val = function(pose, points[el.second], normals[el.second]);
            
            #pragma omp critical
            val += local_val * normalizedWeight;
        }
        
        return val;
    }

    LabelType AutonomousPSOCamGenerator::parentCatllToLogVonMises(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normal)
    {
        return super::logVonMisesWrapper(pose, centroid, normal);
    }

    LabelType AutonomousPSOCamGenerator::parentCallToVisibilityEstimation(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return super::visibilityDistribution(pose, centroid, normalVector);
    }

    LabelType AutonomousPSOCamGenerator::parentCallToPlaneDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return super::imageProjectionDistribution(pose, centroid, normalVector);
    }

    void AutonomousPSOCamGenerator::updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points[pointIndex] = point;
        updateMeshInfo(pointIndex, normal, uncertainty);
        updateWorstPoints(pointIndex, uncertainty);
    }

    void AutonomousPSOCamGenerator::updateMeshInfo(int pointIndex, GLMVec3 normal, double uncertainty)
    {
        this->normals[pointIndex] = normal;
        updateMeshInfo(pointIndex, uncertainty);
        updateWorstPoints(pointIndex, uncertainty);
    }

    void AutonomousPSOCamGenerator::updateMeshInfo(int pointIndex, double uncertainty)
    {
        SUM_UNCERTAINTY += uncertainty - this->uncertainty[pointIndex];
        this->uncertainty[pointIndex] = uncertainty;
        updateWorstPoints(pointIndex, uncertainty);
    }

    void AutonomousPSOCamGenerator::addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points.push_back(point);
        this->normals.push_back(normal);
        this->uncertainty.push_back(uncertainty);
        SUM_UNCERTAINTY += uncertainty;
        updateWorstPoints(this->points.size()-1, uncertainty);
    }

    void AutonomousPSOCamGenerator::setupWorstPoints()
    {
        worstPointsList.clear();
        for (int p = 0; p < points.size(); p++) {
            worstPointsList.push_back(std::make_pair(uncertainty[p], p));
        }
        retainWorst();
    }

    void AutonomousPSOCamGenerator::updateWorstPoints(int index, long double uncertainty)
    {
        worstPointsList.push_back(std::make_pair(uncertainty, index));
        retainWorst();
    }

    void AutonomousPSOCamGenerator::retainWorst()
    {
        std::sort(worstPointsList.rbegin(), worstPointsList.rend());
        int eraseFrom = std::min(this->maxPoints, worstPointsList.size());
        worstPointsList.erase(worstPointsList.begin() + eraseFrom, worstPointsList.end());
    }

    DoubleIntList AutonomousPSOCamGenerator::getWorstPointsList()
    {
        return worstPointsList;
    }  

    GLMVec3List AutonomousPSOCamGenerator::getPoints()
    {
        return points;
    }

    GLMVec3List AutonomousPSOCamGenerator::getNormals()
    {
        return normals;
    }
    
    DoubleList AutonomousPSOCamGenerator::getUncertainties()
    {
        return uncertainty;
    }

} // namespace opview
