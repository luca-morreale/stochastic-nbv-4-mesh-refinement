#include <opview/ScorerGM.hpp>

namespace opview {

    ScorerGM::ScorerGM(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig,
                                    std::string meshFile, GLMVec3List &cams, double goalAngle, double dispersion) 
                                    : OrientationHierarchicalGraphicalModel(solver, config, camConfig, meshFile, cams, goalAngle, dispersion)
    { /*    */ }
    ScorerGM::~ScorerGM()
    { /*    */ }


    double ScorerGM::score(EigVector5 &camPos, GLMVec3 &centroid, GLMVec3 &normal)
    {
        camPos[3] = deg2rad(camPos[3]);
        camPos[4] = deg2rad(camPos[4]);
        double visibility = 0.0, projection = 0.0, vonMises = 0.0, constraint = 0.0;
        GLMVec3 point = GLMVec3(camPos[0], camPos[1], camPos[2]);

        #pragma omp parallel sections 
        {
            #pragma omp section
            visibility = visibilityDistribution(camPos, centroid, normal);
            #pragma omp section
            projection = imagePlaneWeight(camPos, centroid, normal);
            #pragma omp section
            vonMises = -logVonMisesWrapper(point, centroid, normal);
            #pragma omp section
            constraint = constraintValue(point, centroid);
        }

        return visibility + projection + vonMises + constraint;
    }

    double ScorerGM::constraintValue(GLMVec3 &point, GLMVec3 &centroid)
    {
        for (GLMVec3 cam : getCams()) {
            double B = glm::distance(point, cam);
            double D = std::min(glm::distance(cam, centroid), glm::distance(point, centroid));

            if (B / D < BD_TERRESTRIAL_ARCHITECTURAL) {
                return -10.0;
            }
        }
        return 0.0;
    }


} // namespace opview
