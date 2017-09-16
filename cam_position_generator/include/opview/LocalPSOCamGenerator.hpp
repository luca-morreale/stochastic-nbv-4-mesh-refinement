#ifndef CAM_POSITION_GENERATOR_LOCAL_PSO_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_LOCAL_PSO_CAM_GENERATOR_H

#include <gsl/gsl_sf_gamma.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>

#include <opview/PSOCamGenerator.hpp>

namespace opview {

    class LocalPSOCamGenerator : public PSOCamGenerator {
    public:
        LocalPSOCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            StochasticConfiguration &config, double goalAngle=45, double dispersion=8);

        ~LocalPSOCamGenerator();

    protected:

        virtual void updateVelocityParticle(int p) override;
    };

    typedef LocalPSOCamGenerator* LocalPSOCamGeneratorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_LOCAL_PSO_CAM_GENERATOR_H
