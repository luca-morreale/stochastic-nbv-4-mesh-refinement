#ifndef EVALUATION_CAMERA_POSITION_MIDDLEBURY_DATASET_READER_H
#define EVALUATION_CAMERA_POSITION_MIDDLEBURY_DATASET_READER_H


#include <fstream>

#include <aliases.h>
#include <type_definition.h>
#include <utilities.hpp>

namespace cameval {

    #define LINE_SIZE 22

    class MiddleburyDatasetReader {
    public:
        MiddleburyDatasetReader(std::string &cameraPoses);
        ~MiddleburyDatasetReader();

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

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_MIDDLEBURY_DATASET_READER_H
