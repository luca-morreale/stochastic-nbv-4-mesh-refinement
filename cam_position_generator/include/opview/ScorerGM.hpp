#ifndef CAM_POSITION_GENERATOR_SCORER_GRAPHICAL_MODEL_H
#define CAM_POSITION_GENERATOR_SCORER_GRAPHICAL_MODEL_H

#include <opview/alias_definition.h>
#include <opview/type_definition.h>
#include <opview/OrientationHierarchicalGraphicalModel.hpp>


namespace opview {
    
    class ScorerGM : public OrientationHierarchicalGraphicalModel {

    public:
        ScorerGM(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig,
                                            std::string meshFile, GLMVec3List &cams, double goalAngle=55, double dispersion=5);
        ~ScorerGM();

        virtual double score(EigVector5 &camPos, GLMVec3 &centroid, GLMVec3 &normal);
    
    private:

        typedef OrientationHierarchicalGraphicalModel super;


    };

} // namespace opview


#endif // CAM_POSITION_GENERATOR_SCORER_GRAPHICAL_MODEL_H
