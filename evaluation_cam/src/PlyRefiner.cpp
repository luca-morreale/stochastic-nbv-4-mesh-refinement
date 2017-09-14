#include <PlyRefiner.hpp>

namespace cameval {

    PlyRefiner::PlyRefiner(std::string &file, GLMVec3 &color, float outlierThreshold)
    {
        pointColor = color;
        this->outlierThreshold = outlierThreshold;
        load(file);
    }

    PlyRefiner::~PlyRefiner()
    {
        cameras.clear();
        points.clear();
        camViewingPoint.clear();
        point2DoncamViewingPoint.clear();
        inliers.clear();
    }

    void PlyRefiner::load(std::string &file)
    {
        OpenMvgParser json(file);
        json.parse();
        auto sfm = json.getSfmData();

        for (int i = 0; i < sfm.camerasList_.size(); i++) {
            cameras.push_back(sfm.camerasList_[i].cameraMatrix);
        }

        points = sfm.points_;
        camViewingPoint = sfm.camViewingPointN_;
        point2DoncamViewingPoint = sfm.point2DoncamViewingPoint_;
    }

    void PlyRefiner::write(std::string &file, bool colors)
    {
        std::ofstream outputPly;
        outputPly.open(file);
        
        outputPly << "ply" << std::endl;
        outputPly << "format ascii 1.0" << std::endl;
        outputPly << "comment This contains a Splatted Point Cloud" << std::endl;
        outputPly << "element vertex " << getPointsCount() << std::endl;
        outputPly << "property float x" << std::endl;
        outputPly << "property float y" << std::endl;
        outputPly << "property float z" << std::endl;
        
        if (colors) {
            outputPly << "property uchar red" << std::endl;
            outputPly << "property uchar green" << std::endl;
            outputPly << "property uchar blue" << std::endl;
        }
        
        outputPly << "element face 0" << std::endl;
        outputPly << "property list uchar int vertex_indices" << std::endl;
        outputPly << "end_header" << std::endl;


        for (int i = 0; i < this->points.size(); i++) {
            if (isInlier(i)) {
                outputPly << this->points[i].x << " " << this->points[i].y << " " << this->points[i].z << " ";
                if (colors) {
                    outputPly << this->pointColor.x << " " << this->pointColor.y << " " << this->pointColor.z << " ";
                }
                outputPly << std::endl;
            }
        }
        outputPly.close();
    }

    int PlyRefiner::getPointsCount()
    {
        if (inliers.size() != points.size()) {
            return points.size();
        }
        return std::count(inliers.begin(), inliers.end(), true);
    }

    bool PlyRefiner::isInlier(int pointIndex) 
    {
        if (inliers.size() != points.size()) {
            return true;
        }
        return inliers[pointIndex];
    } 

    void PlyRefiner::filterOutliers() 
    {
        this->inliers.assign(this->points.size(), false);
        CVMatList cameras;
        CVPoint2fList measures;
        CVPoint3f init3Dpoint;
        CVPoint3f optimizedPoint;

        for (int curPt3D = 0; curPt3D < this->points.size(); curPt3D++) {
            cameras.clear();
            cameras.assign(this->camViewingPoint[curPt3D].size(), CVMat());
            for (int curC = 0; curC < camViewingPoint[curPt3D].size(); curC++) {
                cameras[curC] = CVMat(4, 4, CV_32F);
                for (int row = 0; row < 4; row++) {
                    for (int col = 0; col < 4; col++) {
                        int camIndex = camViewingPoint[curPt3D][curC];
                        cameras[curC].at<float>(row, col) = this->cameras[camIndex][row][col];
                    }
                }
            }

            measures.clear();
            measures.assign(this->point2DoncamViewingPoint[curPt3D].size(), CVPoint2f());
            for (int curMeas = 0; curMeas < this->point2DoncamViewingPoint[curPt3D].size(); curMeas++) {
                measures[curMeas].x = this->point2DoncamViewingPoint[curPt3D][curMeas].x;
                measures[curMeas].y = this->point2DoncamViewingPoint[curPt3D][curMeas].y;
            }

            init3Dpoint.x = this->points[curPt3D].x;
            init3Dpoint.y = this->points[curPt3D].y;
            init3Dpoint.z = this->points[curPt3D].z;

            if (GaussNewton(cameras, measures, init3Dpoint, optimizedPoint, outlierThreshold) != -1) {

                this->points[curPt3D].x = optimizedPoint.x;
                this->points[curPt3D].y = optimizedPoint.y;
                this->points[curPt3D].z = optimizedPoint.z;
                this->inliers[curPt3D] = true;
            }
        }

    }

    int PlyRefiner::GaussNewton(const CVMatList &cameras, const CVPoint2fList &points, CVPoint3f init3Dpoint, CVPoint3f &optimizedPoint, const float outlierThreshold) 
    {
        int numMeasures = points.size();
        cv::Mat r = cv::Mat(numMeasures * 2, 1, CV_32F);

        cv::Mat curEstimate3DPoint = cv::Mat(3, 1, CV_32F);
        cv::Mat curEstimate3DPointH = cv::Mat(4, 1, CV_32F);
        curEstimate3DPoint.at<float>(0, 0) = init3Dpoint.x;
        curEstimate3DPoint.at<float>(1, 0) = init3Dpoint.y;
        curEstimate3DPoint.at<float>(2, 0) = init3Dpoint.z;

        cv::Mat J, H;
        float last_mse = 0;
        int i;
        for (i = 0; i < 30; i++) {

            float mse = 0;
            //compute residuals
            for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
                curEstimate3DPointH.at<float>(0, 0) = curEstimate3DPoint.at<float>(0, 0);
                curEstimate3DPointH.at<float>(1, 0) = curEstimate3DPoint.at<float>(1, 0);
                curEstimate3DPointH.at<float>(2, 0) = curEstimate3DPoint.at<float>(2, 0);
                curEstimate3DPointH.at<float>(3, 0) = 1.0;
                cv::Mat cur2DpositionH = cameras[curMeas] * curEstimate3DPointH;

                r.at<float>(2 * curMeas, 0) = ((points[curMeas].x - cur2DpositionH.at<float>(0, 0) / cur2DpositionH.at<float>(2, 0)));
                mse += r.at<float>(2 * curMeas, 0) * r.at<float>(2 * curMeas, 0);

                r.at<float>(2 * curMeas + 1, 0) = ((points[curMeas].y - cur2DpositionH.at<float>(1, 0) / cur2DpositionH.at<float>(2, 0)));
                mse += r.at<float>(2 * curMeas + 1, 0) * r.at<float>(2 * curMeas + 1, 0);
            }

            if (abs(mse / (numMeasures * 2) - last_mse) < 0.0000000005) {
                break;
            }
            last_mse = mse / (numMeasures * 2);

            if (point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1) {
                return -1;
            }

            curEstimate3DPoint += H.inv() * J.t() * r;

        }

        if (last_mse < outlierThreshold/*3 pixels*/) {
            optimizedPoint.x = curEstimate3DPoint.at<float>(0, 0);
            optimizedPoint.y = curEstimate3DPoint.at<float>(1, 0);
            optimizedPoint.z = curEstimate3DPoint.at<float>(2, 0);
            return 1;
        } else {
            return -1;
        }
    }

    int PlyRefiner::point2D3DJacobian(const CVMatList &cameras, const CVMat &cur3Dpoint, CVMat &J, CVMat &hessian) 
    {
        int numMeasures = cameras.size();
        cv::Mat cur3DPointHomog = cv::Mat(4, 1, CV_32F);

        cur3DPointHomog.at<float>(0, 0) = cur3Dpoint.at<float>(0, 0);
        cur3DPointHomog.at<float>(1, 0) = cur3Dpoint.at<float>(1, 0);
        cur3DPointHomog.at<float>(2, 0) = cur3Dpoint.at<float>(2, 0);
        cur3DPointHomog.at<float>(3, 0) = 1.0;

        J = cv::Mat(2 * numMeasures, 3, CV_32FC1);  //2 rows for each point: one for x, the other for y
        hessian = cv::Mat(3, 3, CV_32FC1);

        for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
            cv::Mat curReproj = cameras[curMeas] * cur3DPointHomog;
            float xH = curReproj.at<float>(0, 0);
            float yH = curReproj.at<float>(1, 0);
            float zH = curReproj.at<float>(2, 0);
            float p00 = cameras[curMeas].at<float>(0, 0);
            float p01 = cameras[curMeas].at<float>(0, 1);
            float p02 = cameras[curMeas].at<float>(0, 2);
            float p10 = cameras[curMeas].at<float>(1, 0);
            float p11 = cameras[curMeas].at<float>(1, 1);
            float p12 = cameras[curMeas].at<float>(1, 2);
            float p20 = cameras[curMeas].at<float>(2, 0);
            float p21 = cameras[curMeas].at<float>(2, 1);
            float p22 = cameras[curMeas].at<float>(2, 2);

            //d(P*X3D)/dX
            J.at<float>(2 * curMeas, 0) = (p00 * zH - p20 * xH) / (zH * zH);
            J.at<float>(2 * curMeas + 1, 0) = (p10 * zH - p20 * yH) / (zH * zH);

            //d(P*X3D)/dY
            J.at<float>(2 * curMeas, 1) = (p01 * zH - p21 * xH) / (zH * zH);
            J.at<float>(2 * curMeas + 1, 1) = (p11 * zH - p21 * yH) / (zH * zH);

            //d(P*X3D)/dZ
            J.at<float>(2 * curMeas, 2) = (p02 * zH - p22 * xH) / (zH * zH);
            J.at<float>(2 * curMeas + 1, 2) = (p12 * zH - p22 * yH) / (zH * zH);
        }

        hessian = J.t() * J;
        float d;
        d = cv::determinant(hessian);
        if (d < 0.0000000001) {
            return -1;
        } else {
            return 1;
        }
    }


}
