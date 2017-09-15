#ifndef CAM_POSITION_GENERATOR_FORMULATION_H_
#define CAM_POSITION_GENERATOR_FORMULATION_H_

#include <opview/alias_definition.h>
#include <opview/orientation_utilities.hpp>
#include <opview/type_definition.h>
#include <opview/utilities.hpp>


namespace opview {

    #define BD_TERRESTRIAL_ARCHITECTURAL 0.33f 
    
    class Formulation {
    public:

        Formulation(VonMisesConfiguration vonMisesConfig, CameraGeneralConfiguration camConfig, TreePtr tree, GLMVec3List &cams);
        ~Formulation();

        double computeEnergy(EigVector5 &pose, GLMVec3List &centroids, GLMVec3List &normVectors);
        double computeEnergy(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normVector);

        static double computeEnergy(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normVector, 
                                    VonMisesConfiguration &vonMisesConfig, CameraGeneralConfiguration &camConfig, 
                                    TreePtr tree, GLMVec3List &cams);

        static double logVonMisesWrapper(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normVector, VonMisesConfiguration &vonMisesConfig);
        static double visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector, CameraGeneralConfiguration &camConfig, TreePtr tree);
        static double imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector, CameraGeneralConfiguration &camConfig, TreePtr tree);
        static double computeBDConstraint(EigVector5 &pose, GLMVec3 &centroid, GLMVec3List &cams);
        static double computeBDConstraint(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &cam);


    private:
        VonMisesConfiguration vonMisesConfig;
        CameraGeneralConfiguration camConfig;
        TreePtr tree;
        GLMVec3List cams;

    
    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_FORMULATION_H_
