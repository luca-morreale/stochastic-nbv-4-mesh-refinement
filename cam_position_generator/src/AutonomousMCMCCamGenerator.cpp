#include <opview/AutonomousMCMCCamGenerator.hpp>

namespace opview {

    AutonomousMCMCCamGenerator::AutonomousMCMCCamGenerator(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                            MCConfiguration &mcConfig, size_t maxPoints, long double maxUncertainty, double goalAngle, double dispersion)
                                                            : MCMCCamGenerator(camConfig, meshConfig.filename, meshConfig.cams, mcConfig, goalAngle, dispersion)
    {
        this->points = meshConfig.points;
        this->normals = meshConfig.normals;
        this->uncertainty = meshConfig.uncertainty;
        this->maxPoints = maxPoints;
        this->maxUncertainty = maxUncertainty;

        precomputeSumUncertainty();
        setupWorstPoints();

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

    double AutonomousMCMCCamGenerator::computeWeightForPoint(int pointIndex)
    {
        return (double)uncertainty[pointIndex] / (double)SUM_UNCERTAINTY;
    }

    void AutonomousMCMCCamGenerator::estimateBestCameraPosition()
    {
        size_t worstPointIndex = worstPointsList[0].first;
        GLMVec3 worstCentroid = this->points[worstPointIndex];
        GLMVec3 normal = this->normals[worstPointIndex];
        this->estimateBestCameraPosition(worstCentroid, normal);
    }

    LabelType AutonomousMCMCCamGenerator::logVonMisesWrapper(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normal)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&AutonomousMCMCCamGenerator::parentCatllToLogVonMises, this, _1, _2, _3));
    }

    LabelType AutonomousMCMCCamGenerator::visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&AutonomousMCMCCamGenerator::parentCallToVisibilityEstimation, this, _1, _2, _3));
    }

    LabelType AutonomousMCMCCamGenerator::imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&AutonomousMCMCCamGenerator::parentCallToPlaneDistribution, this, _1, _2, _3));
    }

    double AutonomousMCMCCamGenerator::estimateForWorstPointSeen(EigVector5 &pose, BoostObjFunction function)
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

    LabelType AutonomousMCMCCamGenerator::parentCatllToLogVonMises(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normal)
    {
        return super::logVonMisesWrapper(pose, centroid, normal);
    }

    LabelType AutonomousMCMCCamGenerator::parentCallToVisibilityEstimation(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return super::visibilityDistribution(pose, centroid, normalVector);
    }

    LabelType AutonomousMCMCCamGenerator::parentCallToPlaneDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return super::imageProjectionDistribution(pose, centroid, normalVector);
    }

    void AutonomousMCMCCamGenerator::updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points[pointIndex] = point;
        updateMeshInfo(pointIndex, normal, uncertainty);
        updateWorstPoints(pointIndex, uncertainty);
    }

    void AutonomousMCMCCamGenerator::updateMeshInfo(int pointIndex, GLMVec3 normal, double uncertainty)
    {
        this->normals[pointIndex] = normal;
        updateMeshInfo(pointIndex, uncertainty);
        updateWorstPoints(pointIndex, uncertainty);
    }

    void AutonomousMCMCCamGenerator::updateMeshInfo(int pointIndex, double uncertainty)
    {
        SUM_UNCERTAINTY += uncertainty - this->uncertainty[pointIndex];
        this->uncertainty[pointIndex] = uncertainty;
        updateWorstPoints(pointIndex, uncertainty);
    }

    void AutonomousMCMCCamGenerator::addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points.push_back(point);
        this->normals.push_back(normal);
        this->uncertainty.push_back(uncertainty);
        SUM_UNCERTAINTY += uncertainty;
        updateWorstPoints(this->points.size()-1, uncertainty);
    }

    void AutonomousMCMCCamGenerator::setupWorstPoints()
    {
        worstPointsList.clear();
        #pragma omp parallel for        // not much to gain using parallelism
        for (int p = 0; p < points.size(); p++) {
            if (uncertainty[p] > this->maxUncertainty) {
                #pragma omp critical
                worstPointsList.push_back(std::make_pair(uncertainty[p], p));
            }
        }
        retainWorst();
    }

    void AutonomousMCMCCamGenerator::updateWorstPoints(int index, long double uncertainty)
    {
        worstPointsList.push_back(std::make_pair(uncertainty, index));
        retainWorst();
    }

    void AutonomousMCMCCamGenerator::retainWorst()
    {
        std::sort(worstPointsList.rbegin(), worstPointsList.rend());
        int eraseFrom = std::min(this->maxPoints, worstPointsList.size());
        worstPointsList.erase(worstPointsList.begin() + eraseFrom, worstPointsList.end());
    }

    DoubleIntList AutonomousMCMCCamGenerator::getWorstPointsList()
    {
        return worstPointsList;
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
