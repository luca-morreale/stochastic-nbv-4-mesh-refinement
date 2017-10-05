#include <MiddleburyDatasetReader.hpp>

namespace cameval {

    MiddleburyDatasetReader::MiddleburyDatasetReader(std::string &cameraPoses)
    {
        this->cameraPoses = cameraPoses;
        readPoses();
    }

    MiddleburyDatasetReader::~MiddleburyDatasetReader()
    {
        views.clear();
        cameras.clear();
    }

    void MiddleburyDatasetReader::readPoses()
    {
        ArrayStringList lines;
        StringList blocks(LINE_SIZE);
        int rows;

        std::ifstream cin(cameraPoses);
        
        cin >> rows;
        std::cout << rows << std::endl;
        for (int r = 0; r < rows; r++) {
            for (int i = 0; i < LINE_SIZE; i++) {
                cin >> blocks[i];
            }
            lines.push_back(blocks);
        }
        
        // NOTE can not use parallelism beacuse otherwise in json insertion is done at random
        for(int i = 0; i < lines.size(); i++) {

            blocks = lines[i];

            std::string view = blocks[0];
            Camera cam = getCamera(blocks);

            views.push_back(view);
            cameras.push_back(cam);
        }
    }

    StringList MiddleburyDatasetReader::getViews()
    {
        return views;
        
    }

    CameraList MiddleburyDatasetReader::getCamerasMatrices()
    {
        return cameras;
    }

} // namespace cameval
