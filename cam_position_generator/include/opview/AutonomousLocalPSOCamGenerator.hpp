#ifndef CAM_POSITION_GENERATOR_AUTONOMOUS_LOCAL_PSO_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_AUTONOMOUS_LOCAL_PSO_CAM_GENERATOR_H

#include <gsl/gsl_sf_gamma.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#include <opview/AutonomousPSOCamGenerator.hpp>

namespace opview {

    class AutonomousLocalPSOCamGenerator : public AutonomousPSOCamGenerator {
    public:
        AutonomousLocalPSOCamGenerator(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                            PSOConfiguration &psoConfig, double goalAngle=45, double dispersion=8);

        ~AutonomousLocalPSOCamGenerator();

    protected:

        virtual void updateVelocityParticle(int p) override;
    };

    typedef AutonomousLocalPSOCamGenerator* AutonomousLocalPSOCamGeneratorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_AUTONOMOUS_LOCAL_PSO_CAM_GENERATOR_H
