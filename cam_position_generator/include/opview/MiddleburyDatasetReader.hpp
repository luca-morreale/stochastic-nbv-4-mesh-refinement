#ifndef CAM_POSITION_GENERATOR_MIDDLEBURY_DATASET_READER_H
#define CAM_POSITION_GENERATOR_MIDDLEBURY_DATASET_READER_H


#include <fstream>

#include <opview/alias_definition.h>
#include <opview/type_definition.h>

namespace opview {

    #define LINE_SIZE 22

    class MiddleburyDatasetReader {
    public:
        MiddleburyDatasetReader(std::string &cameraPoses);
        ~MiddleburyDatasetReader();

        CameraMatrix getRotation(StringList blocks);
        CameraMatrix getIntrinsic(StringList blocks);
        GLMVec4 getTranslation(StringList blocks);
        StringList getViews();
        CameraList getCamerasMatrices();

    protected:
        void readPoses();

    private:
        std::string cameraPoses;

        CameraList cameras;
        StringList views;

        
    };

    typedef MiddleburyDatasetReader* MiddleburyDatasetReaderPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_MIDDLEBURY_DATASET_READER_H
