#include <PointCloudIntersecter.hpp>

namespace cameval {

    PointCloudIntersecter::PointCloudIntersecter(SfMData &dataPC1, SfMData &dataPC2)
    {
        setUpClouds(dataPC1, dataPC2);
    }


    PointCloudIntersecter::PointCloudIntersecter(std::string &filePC1, std::string &filePC2)
    {
        OpenMvgParser parser1(filePC1);
        parser1.parse();
        OpenMvgParser parser2(filePC2);
        parser2.parse();

        SfMData dataPC1 = parser1.getSfmData();
        SfMData dataPC2 = parser2.getSfmData();
        setUpClouds(dataPC1, dataPC2);
    }

    PointCloudIntersecter::~PointCloudIntersecter()
    {
        this->pointsPC1.clear();
        this->pointsPC2.clear();

        this->point3DTo2DThroughCamPC1.clear();
        this->point3DTo2DThroughCamPC2.clear();
    }

    void PointCloudIntersecter::setUpClouds(SfMData &dataPC1, SfMData &dataPC2)
    {
        this->pointsPC1 = dataPC1.points_;
        this->pointsPC2 = dataPC2.points_;

        this->point3DTo2DThroughCamPC1 = dataPC1.point3DTo2DThroughCam_;
        this->point3DTo2DThroughCamPC2 = dataPC2.point3DTo2DThroughCam_;
    }

    void PointCloudIntersecter::write(std::string &file, GLMVec3List &points)
    {
        std::ofstream outputPly;
        outputPly.open(file);
        
        outputPly << "ply" << std::endl;
        outputPly << "format ascii 1.0" << std::endl;
        outputPly << "comment This contains a Splatted Point Cloud" << std::endl;
        outputPly << "element vertex " << points.size() << std::endl;
        outputPly << "property float x" << std::endl;
        outputPly << "property float y" << std::endl;
        outputPly << "property float z" << std::endl;        
        outputPly << "element face 0" << std::endl;
        outputPly << "property list uchar int vertex_indices" << std::endl;
        outputPly << "end_header" << std::endl;

        for (int i = 0; i < points.size(); i++) {
            outputPly << points[i].x << " " << points[i].y << " " << points[i].z << std::endl;
        }
        outputPly.close();
    }

    void PointCloudIntersecter::write(std::string outPC1, std::string outPC2, double threshold)
    {
        intersect(threshold);
        PointCloudIntersecter::write(outPC1, pointsPC1Intersected);
        PointCloudIntersecter::write(outPC2, pointsPC2Intersected);
    }

    void PointCloudIntersecter::intersect(double threshold)
    {
        std::cout << pointsPC1.size() << " " << pointsPC2.size() << std::endl;
        #pragma omp parallel for collapse(2)
        for (int i = 0; i < pointsPC1.size(); i++) {
            for (int j = 0; j < pointsPC2.size(); j++) {
                IntGLMVe2Map map1 = point3DTo2DThroughCamPC1[i];
                IntGLMVe2Map map2 = point3DTo2DThroughCamPC2[j];
                if(commonFeatures(map1, map2, threshold)) {
                    #pragma omp critical
                    {
                        pointsPC1Intersected.push_back(pointsPC1[i]);
                        pointsPC2Intersected.push_back(pointsPC2[j]);
                    }
                }
            }
        }
    }

    bool PointCloudIntersecter::commonFeatures(IntGLMVe2Map map1, IntGLMVe2Map map2, double threshold)
    {
        int equals = 0;

        #pragma omp parallel for reduction(+:equals) 
        for (int i = 0; i < map1.size(); i++) {
            IntGLMVe2Map::iterator it1 = std::begin(map1);
            advance(it1, i);
            
            int camIndex = it1->first;
            auto point = it1->second;

            if (map2.find(camIndex) != map2.end()) {

                if (glm::all(glm::equal(point, map2[camIndex]))) {
                    equals += 1;
                }
            }
        }

        return (double)equals > map1.size() * threshold;
    }

    


} // namespace cameval
