#include <opview/LocalPSOCamGenerator.hpp>

namespace opview {


    LocalPSOCamGenerator::LocalPSOCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                                    StochasticConfiguration &config, double offspring, double goalAngle, double dispersion)
                                                    : PSOCamGenerator(camConfig, meshFile, cams, config, offspring, goalAngle, dispersion)
    { /*    */ }

    LocalPSOCamGenerator::~LocalPSOCamGenerator()
    { /*    */ }

    void LocalPSOCamGenerator::updateVelocityParticle(int p)
    {
        EigVector5 randBestLeft = this->randVector();
        EigVector5 randBestRight = this->randVector();
        EigVector5 randPrevious = this->randVector();

        int left = (p - 1 < 0) ? particles.size() - 1 : p - 1;
        int right = (p + 1 >= particles.size()) ? 0 : p + 1;

        EigVector5 diffPreviousBest = particles[p]->bestPosition - particles[p]->position;
        EigVector5 diffLeft = particles[left]->position - particles[p]->position;
        EigVector5 diffRight = particles[right]->position - particles[p]->position;

        particles[p]->velocity = inertiaWeight.cwiseProduct(particles[p]->velocity);
        particles[p]->velocity += randPrevious.cwiseProduct(c1.cwiseProduct(diffPreviousBest));
        particles[p]->velocity += randBestLeft.cwiseProduct(c2.cwiseProduct(diffLeft));
        particles[p]->velocity += randBestRight.cwiseProduct(c2.cwiseProduct(diffRight));

        this->fixSpaceVelocity(p);
    }

} // namespace opview
