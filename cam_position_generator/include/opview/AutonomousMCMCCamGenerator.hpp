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
                                            MCConfiguration &mcConfig, double goalAngle=55, double dispersion=8);

        ~AutonomousMCMCCamGenerator();

        virtual void updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double accuracy);
        virtual void updateMeshInfo(int pointIndex, GLMVec3 normal, double accuracy);
        virtual void updateMeshInfo(int pointIndex, double accuracy);
        virtual void addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty);

        virtual void estimateBestCameraPosition();
        
    protected:

        virtual void updateWorstReference(int pointIndex, double accuracy);
        virtual int checkWorstPoint();
        virtual void setWorstPoint();

        virtual GLMVec3List getPoints();
        virtual GLMVec3List getNormals();
        virtual DoubleList getUncertainties();

        using MCMCCamGenerator::estimateBestCameraPosition;

    private:
        GLMVec3List points;
        GLMVec3List normals;
        DoubleList uncertainty;

        long double SUM_UNCERTAINTY;
        long double worstUncertainty;
        int worstPointIndex;

        void precomputeSumUncertainty();

        typedef MCMCCamGenerator super;

    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_AUTONOMOUS_MCMC_CAM_GENERATOR_H
