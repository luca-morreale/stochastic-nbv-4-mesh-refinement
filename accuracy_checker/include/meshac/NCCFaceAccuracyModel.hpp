#ifndef MESH_ACCURACY_NCC_FACE_ACCURACY_MODEL_H
#define MESH_ACCURACY_NCC_FACE_ACCURACY_MODEL_H

#include <meshac/GeneralIndexFaceAccuracyModel.hpp>

namespace meshac {

    /**
     * Computes 1 - SSDN of each triangles, so the higher the better.
     */
    
    class NCCFaceAccuracyModel : public GeneralIndexFaceAccuracyModel {
    public:
        NCCFaceAccuracyModel(std::string &meshFile, SfMData &data, std::string &pathPrefix);
        ~NCCFaceAccuracyModel();

    protected:
        virtual double computeIndex(CVMatList triangles);
        virtual double worstIndex();

    };

    typedef NCCFaceAccuracyModel* NCCFaceAccuracyModelPtr;

}

#endif // MESH_ACCURACY_NCC_FACE_ACCURACY_MODEL_H
