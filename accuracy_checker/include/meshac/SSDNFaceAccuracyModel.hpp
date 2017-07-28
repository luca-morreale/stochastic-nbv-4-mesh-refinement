#ifndef MESH_ACCURACY_SSDN_FACE_ACCURACY_MODEL_H
#define MESH_ACCURACY_SSDN_FACE_ACCURACY_MODEL_H

#include <meshac/GeneralIndexFaceAccuracyModel.hpp>

namespace meshac {

    /**
     * Computes 1 - SSDN of each triangles, so the higher the better.
     */
    
    class SSDNFaceAccuracyModel : public GeneralIndexFaceAccuracyModel {
    public:
        SSDNFaceAccuracyModel(std::string &meshFile, SfMData &data, std::string &pathPrefix);
        ~SSDNFaceAccuracyModel();

    protected:
        virtual double computeIndex(CVMatList triangles);

    };

    typedef SSDNFaceAccuracyModel* SSDNFaceAccuracyModelPtr;

}

#endif // MESH_ACCURACY_SSDN_FACE_ACCURACY_MODEL_H
