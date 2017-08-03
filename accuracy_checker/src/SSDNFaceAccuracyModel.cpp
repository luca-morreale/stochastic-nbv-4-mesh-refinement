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

        // NOTE if parallelized probably dies
        // #pragma omp parallel for
        for (int i = 0; i < triangles.size() - 1; i++) {    // try all possible pair of triangles only once!!!
            for (int j = i+1; j < triangles.size(); j++) {

                CVMat matArray[] = { triangles[i], triangles[j] };
                CVMat out;

                double maxVal;
                CVPoint maxLoc;
                // cv::matchTemplate(triangles[i], triangles[j], result, CV_TM_CCORR_NORMED);
                cv::matchTemplate(triangles[i], triangles[j], result, CV_TM_SQDIFF_NORMED);     // gives more meaningful results
                cv::minMaxLoc(result, NULL, &maxVal, NULL, &maxLoc, CVMat());

                #pragma omp critical
                maxs.push_back(maxVal);
            }
        }

        double average = std::accumulate(maxs.begin(), maxs.end(), 0.0) / (double)maxs.size(); 
        return average;
    }

} // namespace meshac
