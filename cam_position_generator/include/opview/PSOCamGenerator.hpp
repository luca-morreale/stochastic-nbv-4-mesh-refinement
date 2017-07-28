#ifndef CAM_POSITION_GENERATOR_PSO_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_PSO_CAM_GENERATOR_H

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

    class PSOCamGenerator : public MCMCCamGenerator {
    public:
        PSOCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            PSOConfiguration &psoConfig, double goalAngle=45, double dispersion=8);

        ~PSOCamGenerator();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

    protected:

        virtual void convertSamplesToParticles(OrderedPose &currentOptima);
        virtual void updateParticles(GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void updateVelocityParticle(int p);
        virtual void updatePositionParticle(int p);
        virtual void evaluateSwarm(GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void updateSwarmValues(DoubleList &values);

        virtual void fixSpaceVelocity(int p);
        virtual void fixSpacePosition(int p);

        virtual double uniform();
        virtual EigVector5 randVector();

        ParticleList particles;
        EigVector5 inertiaWeight;
        EigVector5 c1;
        EigVector5 c2;
        
    private:
        const gsl_rng *randGen;
        
        GLMVec3 spaceLowerBounds;
        GLMVec3 spaceUpperBounds;

        int bestParticleIndex;


        const long SEED = time(NULL);

        void logParticles(int round);
        void deleteParticles();
        EigVector5List extractSwarmPositions();


    };

    typedef PSOCamGenerator* PSOCamGeneratorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_PSO_SAMPLE_GENERATOR_H
