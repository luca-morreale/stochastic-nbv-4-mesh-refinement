#include <opview/Formulation.hpp>

namespace opview {
    Formulation::Formulation(VonMisesConfiguration vonMisesConfig, CameraGeneralConfiguration camConfig, TreePtr tree, GLMVec3List &cams)
    {
        this->vonMisesConfig = vonMisesConfig;
        this->camConfig = camConfig;
        this->tree = tree;
        this->cams = cams;

    }
    
    Formulation::~Formulation()
    { /*    */ }

    double Formulation::computeEnergy(EigVector5 &pose, GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        double energy = 0.0;

        #pragma omp parallel for reduction(+:energy)
        for (int p = 0; p < centroids.size(); p++) {
            energy += computeEnergy(pose, centroids[p], normVectors[p]);
        }

        return energy;
    }

    double Formulation::computeEnergy(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        return Formulation::computeEnergy(pose, centroid, normVector, this->vonMisesConfig, this->camConfig, this->tree, this->cams);
    }



    double Formulation::computeEnergy(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normVector, 
                                    VonMisesConfiguration &vonMisesConfig, CameraGeneralConfiguration &camConfig, 
                                    TreePtr tree, GLMVec3List &cams)
    {
        double vonMises = 0.0;
        double visibility = 0.0;
        double projection = 0.0;
        double constraints = 0.0;

        #pragma omp parallel sections 
        {
            #pragma omp section
            vonMises = logVonMisesWrapper(pose, centroid, normVector, vonMisesConfig);
            #pragma omp section
            visibility = visibilityDistribution(pose, centroid, normVector, camConfig, tree);
            #pragma omp section
            projection = imageProjectionDistribution(pose, centroid, normVector, camConfig, tree);
            #pragma omp section
            constraints = computeBDConstraint(pose, centroid, cams);
        }

        return vonMises + visibility + projection + constraints;
    }

    double Formulation::logVonMisesWrapper(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normVector, VonMisesConfiguration &vonMisesConfig)
    {
        GLMVec3 point = GLMVec3(pose[0], pose[1], pose[2]);
        return -logVonMises(point, centroid, normVector, vonMisesConfig);
    }

    double Formulation::visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector, CameraGeneralConfiguration &camConfig, TreePtr tree)
    {
        if (isPointInsideImage(pose, centroid, camConfig) && isMeaningfulPose(pose, centroid, tree, camConfig)){
            return 1.0;
        }
        return -10.0;
    }

    double Formulation::imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector, CameraGeneralConfiguration &camConfig, TreePtr tree)
    {
        if (!isPointInsideImage(pose, centroid, camConfig)) {  // fast rejection, fast to compute.
            return -10.0;
        }
        if (!isMeaningfulPose(pose, centroid, tree, camConfig)) {
            return -10.0;
        }
        
        GLMVec2 point = getProjectedPoint(pose, centroid, camConfig);
        double centerx = (double)camConfig.size_x / 2.0;
        double centery = (double)camConfig.size_y / 2.0;
        double sigma_x = (double)camConfig.size_x / 3.0;
        double sigma_y = (double)camConfig.size_y / 3.0;

        return logBivariateGaussian(point.x, point.y, centerx, centery, sigma_x, sigma_y);  // positive because gaussian have highest value in the center
    }

    double Formulation::computeBDConstraint(EigVector5 &pose, GLMVec3 &centroid, GLMVec3List &cams)
    {
        double constraint = 0.0;

        #pragma omp parallel for reduction(+:constraint)
        for (int c = 0; c < cams.size(); c++) {
            constraint += computeBDConstraint(pose, centroid, cams[c]);
        }

        return constraint;
    }

    double Formulation::computeBDConstraint(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &cam)
    {
        GLMVec3 point = GLMVec3(pose[0], pose[1], pose[2]);
        double D, B;
        #pragma omp parallel sections
        {
            #pragma omp section
            B = glm::distance(point, cam);
            #pragma omp section
            D = std::min(glm::distance(cam, centroid), glm::distance(point, centroid));
        }
        
        if (B / D < BD_TERRESTRIAL_ARCHITECTURAL) {
            return -10.0;
        }
        return 1.0;
    }

} // namespace opview
