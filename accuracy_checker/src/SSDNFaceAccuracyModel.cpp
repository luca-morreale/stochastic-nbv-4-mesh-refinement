#include <meshac/SSDNFaceAccuracyModel.hpp>

namespace meshac {

    SSDNFaceAccuracyModel::SSDNFaceAccuracyModel(std::string &meshFile, SfMData &data, std::string &pathPrefix) 
                                                    : GeneralIndexFaceAccuracyModel(meshFile, data, pathPrefix)
    { /*    */ }

    SSDNFaceAccuracyModel::~SSDNFaceAccuracyModel()
    { /*    */ }  

    double SSDNFaceAccuracyModel::computeIndex(CVMatList triangles)
    {
        CVMat result;
        int result_cols =  1;
        int result_rows = 1;

        result.create(result_rows, result_cols, triangles[0].type());
        DoubleList maxs;

        #pragma omp parallel for collapse(2)
        for (int i = 0; i < triangles.size() - 1; i++) {    // try all possible pair of triangles only once!!!
            for (int j = 0; j < triangles.size(); j++) {
                if (j <= i) continue;

                CVMat matArray[] = { triangles[i], triangles[j] };
                CVMat out;

                double maxVal;
                CVPoint maxLoc;
                cv::matchTemplate(triangles[i], triangles[j], result, CV_TM_SQDIFF_NORMED);     // gives more meaningful results
                cv::minMaxLoc(result, NULL, &maxVal, NULL, &maxLoc, CVMat());
                maxVal = (maxVal > 1.0) ? 1.0 : maxVal;

                #pragma omp critical
                maxs.push_back(maxVal);
            }
        }
        double average = std::accumulate(maxs.begin(), maxs.end(), 0.0) / static_cast<double>(maxs.size());

        return average;
    }

    double SSDNFaceAccuracyModel::worstIndex()
    {
        return 1.0;
    }

} // namespace meshac
