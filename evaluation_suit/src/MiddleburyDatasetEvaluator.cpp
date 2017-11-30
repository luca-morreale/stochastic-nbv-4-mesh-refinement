#include <MiddleburyDatasetEvaluator.hpp>

namespace cameval {
    std::string tmp = "";
    std::string fake_pose = "fake_poses.txt";

    MiddleburyDatasetEvaluator::MiddleburyDatasetEvaluator(std::string &groundTruthFilename, std::string &poseFilename, 
            std::string &baseImageFolder, std::string &basicPoses, std::string &inputImagesFolder, std::string &reportFile)
            : SystemEvaluator(groundTruthFilename, fake_pose, baseImageFolder, tmp, reportFile, tmp)
    {
        this->poseFilename = poseFilename;
        this->inputImagesFolder = inputImagesFolder;

        readDatasetCameras();

        log("\nPopulating Middlebury camera poses");
        setBasicPoseCamera(basicPoses);
    }

    MiddleburyDatasetEvaluator::~MiddleburyDatasetEvaluator()
    { /*    */ }

    void MiddleburyDatasetEvaluator::setBasicPoseCamera(std::string &basicPoses)
    {
        readInitialPoses(basicPoses);
        
        for (std::string posesString : views) {
            Pose newPose = getPose(posesString);
            appendToCameraPoses(newPose);
        }
        
        OpenMvgSysCall::intrinsicParams =  "";
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cameval::OpenMvgSysCall::intrinsicParams += std::to_string(cameras[0].K[i][j]);
                if (i != 2 || j != 2) OpenMvgSysCall::intrinsicParams += ";";
            }
        }
    }

    void MiddleburyDatasetEvaluator::readInitialPoses(std::string &basicPoses)
    {
        std::ifstream cin(basicPoses);

        while(!cin.eof()) {
            std::string view;
            cin >> view;
            std::cout << view << std::endl;
            if (view.empty()) continue;

            views.push_back(view);
            cameras.push_back(getCameraOfView(view));
        }
    }

    Camera MiddleburyDatasetEvaluator::getCameraOfView(std::string &view)
    {
        for (int i = 0; i < datasetViews.size(); i++) {
            if (datasetViews[i].compare(view) == 0) {
                return datasetCameras[i];
            }
        }
        std::cerr << "view not found" << std::endl;
    }

    void MiddleburyDatasetEvaluator::readDatasetCameras()
    {   
        MiddleburyDatasetReader reader(poseFilename);
        datasetCameras = reader.getCamerasMatrices();
        datasetViews = reader.getViews();
    }

    void MiddleburyDatasetEvaluator::datasetEvaluation(int steps, std::string reconstructionExe, std::string accuracyExe)
    {
        systemEvaluation(steps, reconstructionExe, accuracyExe, "./middlebury_scorer ");
    }

    std::string MiddleburyDatasetEvaluator::computeStructure(std::string &jsonFile, int imgId)
    {
        return OpenMvgSysCall::computeStructureFromPoses(jsonFile, imgId);
    }


    std::string MiddleburyDatasetEvaluator::computeDistance(std::string &alignedCloud, std::string &groundTruthFilename)
    {
        std::string logfilename = alignedCloud.substr(0, alignedCloud.find_last_of("/")) + "/log_distance.txt";

        std::string command = "CloudCompare -SILENT -LOG_FILE " + logfilename + " -O " + alignedCloud + " -O " + groundTruthFilename + " -c2m_dist";
        execute(command);

        return logfilename;
    }

    std::string MiddleburyDatasetEvaluator::transformToLookat(std::string &fixedPosefile)
    {
        return fixedPosefile;
    }

    std::string MiddleburyDatasetEvaluator::readPose(std::string &poseFile)
    {
        std::string tmp, posesString;
        std::ifstream cin(poseFile);
        while (!cin.eof()) {
            cin >> tmp;
            if (tmp.empty()) continue;
            posesString = tmp;
        }
        cin.close();

        return posesString;
    }

    Pose MiddleburyDatasetEvaluator::getPose(std::string &data)
    {
        for (int i = 0; i < datasetViews.size(); i++) {
            if (datasetViews[i].compare(data) == 0) {
                // GLMMat3 mat(cameras[i].R);
                return Pose(GLMVec3(datasetCameras[i].t.x, datasetCameras[i].t.y, datasetCameras[i].t.z), GLMMat3(datasetCameras[i].R));
            }
        }
        std::cerr << "View not found: " << data << std::endl;
    }

    std::string MiddleburyDatasetEvaluator::getImage(std::string &poseString)
    {
        std::string source = inputImagesFolder + ((inputImagesFolder[inputImagesFolder.size()-1] == '/') ? "" : "/") + poseString;
        std::string dest = "zz_" + getTimeStamp() + ".png";
        FileHandler::copyFile(source, dest);
        return dest;
    }

} // namespace cameval
