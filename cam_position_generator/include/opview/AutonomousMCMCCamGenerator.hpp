#ifndef CAM_POSITION_GENERATOR_AUTONOMOUS_MCMC_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_AUTONOMOUS_MCMC_CAM_GENERATOR_H

#include <opview/type_definition.h>
#include <opview/alias_definition.h>
#include <opview/MCMCCamGenerator.hpp>
#include <opview/utilities.hpp>

namespace opview {

    class AutonomousMCMCCamGenerator : public MCMCCamGenerator {
    public:
        AutonomousMCMCCamGenerator(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                            MCConfiguration &mcConfig, size_t maxPoints, long double maxUncertainty, double goalAngle=55, double dispersion=8);

        ~AutonomousMCMCCamGenerator();

        virtual void updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double accuracy);
        virtual void updateMeshInfo(int pointIndex, GLMVec3 normal, double accuracy);
        virtual void updateMeshInfo(int pointIndex, double accuracy);
        virtual void addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty);

        virtual void estimateBestCameraPosition();
        
    protected:
        virtual GLMVec3List getPoints();
        virtual GLMVec3List getNormals();
        virtual DoubleList getUncertainties();

        virtual void setupWorstPoints();
        virtual void updateWorstPoints(int index, long double uncertainty);
        virtual void retainWorst();
        virtual DoubleIntList getWorstPointsList();

        virtual double estimateForWorstPointSeen(EigVector5 &pose, BoostObjFunction function);
        virtual double computeWeightForPoint(int pointIndex);

        virtual LabelType logVonMisesWrapper(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normal);
        virtual LabelType visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        virtual LabelType imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);

        using MCMCCamGenerator::estimateBestCameraPosition;

    private:
        GLMVec3List points;
        GLMVec3List normals;
        DoubleList uncertainty;
        DoubleIntList worstPointsList;
        long double SUM_UNCERTAINTY;

        size_t maxPoints;
        long double maxUncertainty;

        void precomputeSumUncertainty();

        LabelType parentCatllToLogVonMises(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normal);
        LabelType parentCallToVisibilityEstimation(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        LabelType parentCallToPlaneDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);

        typedef MCMCCamGenerator super;

    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_AUTONOMOUS_MCMC_CAM_GENERATOR_H
