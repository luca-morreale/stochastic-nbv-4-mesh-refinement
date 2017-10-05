#include <opview/MiddleburyDatasetReader.hpp>

namespace opview {

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
        std::cout << "poses size: " << rows << std::endl;
        for (int r = 0; r < rows; r++) {
            for (int i = 0; i < LINE_SIZE; i++) {
                cin >> blocks[i];
            }
            lines.push_back(blocks);
        }
        
        #pragma omp parallel for
        for(int i = 0; i < lines.size(); i++) {

            blocks = lines[i];

            std::string view = blocks[0];
            CameraMatrix R = getRotation(blocks);
            CameraMatrix K = getIntrinsic(blocks);
            GLMVec4 t = getTranslation(blocks);

            Camera cam(R, K, t);

            #pragma omp critical
            {
                views.push_back(view);
                cameras.push_back(cam);
            }
        }
    }

    CameraMatrix MiddleburyDatasetReader::getRotation(StringList blocks)
    {
        CameraMatrix R;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R[i][j] = strtod(blocks[10 + i * 3 + j].c_str(), NULL);     // check indices
            }
        }

        return R;
    }

    CameraMatrix MiddleburyDatasetReader::getIntrinsic(StringList blocks)
    {
        CameraMatrix K;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                K[i][j] = strtod(blocks[1 + i * 3 + j].c_str(), NULL);     // check indices
            }
        }

        return K;
    }

    GLMVec4 MiddleburyDatasetReader::getTranslation(StringList blocks)
    {
        GLMVec4 t;
        t.x = strtod(blocks[19].c_str(), NULL);
        t.y = strtod(blocks[20].c_str(), NULL);
        t.z = strtod(blocks[21].c_str(), NULL);
        return t;
    }

    StringList MiddleburyDatasetReader::getViews()
    {
        return views;
        
    }

    CameraList MiddleburyDatasetReader::getCamerasMatrices()
    {
        return cameras;
    }

} // namespace opview
