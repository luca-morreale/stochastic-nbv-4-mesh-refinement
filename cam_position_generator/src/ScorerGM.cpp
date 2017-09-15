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
        VonMisesConfigurationPtr vmConfig = vonMisesConfiguration();
        CameraGeneralConfigPtr camConfig = getCamConfig();
        TreePtr tree = getTree();
        GLMVec3List cams = getCams();

        camPos[3] = deg2rad(camPos[3]);
        camPos[4] = deg2rad(camPos[4]);
        double visibility = 0.0, projection = 0.0, vonMises = 0.0, constraint = 0.0;
        GLMVec3 point = GLMVec3(camPos[0], camPos[1], camPos[2]);

        #pragma omp parallel sections 
        {
            #pragma omp section
            visibility = Formulation::visibilityDistribution(camPos, centroid, normal, *camConfig, tree);
            #pragma omp section
            projection = Formulation::imageProjectionDistribution(camPos, centroid, normal, *camConfig, tree);
            #pragma omp section
            vonMises = Formulation::logVonMisesWrapper(point, centroid, normal, *vmConfig);
            #pragma omp section
            constraint = Formulation::computeBDConstraint(point, centroid, cams);
        }

        return visibility + projection + vonMises + constraint;
    }

} // namespace opview
