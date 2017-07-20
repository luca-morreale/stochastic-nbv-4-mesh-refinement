#ifndef CAM_POSITION_GENERATOR_PSO_SAMPLE_GENERATOR_H
#define CAM_POSITION_GENERATOR_PSO_SAMPLE_GENERATOR_H

#include <gsl/gsl_sf_gamma.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#include <opview/alias_definition.h>
#include <opview/SwarmReportWriter.hpp>
#include <opview/MCMCCamGenerator.hpp>
#include <opview/Particle.hpp>

namespace opview {

    #define COORDINATE_SIZE 3
    #define ORIENTATION_SIZE 5

    class PSOCamGenerator : public MCMCCamGenerator {
    public:
        PSOCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            MCConfiguration &mcConfig, double goalAngle=45, double dispersion=8);
        ~PSOCamGenerator();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

    protected:

        void convertSamplesToParticles(OrderedPose &currentOptima);
        void updateParticles(GLMVec3 &centroid, GLMVec3 &normVector);
        void updateVelocityParticle(int j);
        void updatePositionParticle(int j);
        void evaluateSwarm(GLMVec3 &centroid, GLMVec3 &normVector);
        void updateSwarmValues(DoubleList &values);
        
    private:
        const gsl_rng *randGen;
        ParticleList particles;

        EigVector5 omega;
        EigVector5 phiP;

        int bestParticleIndex;


        const long SEED = time(NULL);

        void logParticles(int round);
        void deleteParticles();
        EigVector5List extractSwarmPositions();


    };

    typedef PSOCamGenerator* PSOCamGeneratorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_PSO_SAMPLE_GENERATOR_H
