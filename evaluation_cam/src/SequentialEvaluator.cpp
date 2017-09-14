#include <SequentialEvaluator.hpp>

namespace cameval {

    SequentialEvaluator::SequentialEvaluator(std::string &pointsFile, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                            std::string &baseImageFolder, std::string &intrinsicParams, std::string &reportFile, std::string &basicPovFile) 
                            : PoseEvaluator(pointsFile, groundTruthFilename, basicPoseFilename, baseImageFolder, intrinsicParams, reportFile, basicPovFile)
    { /*    */ 
        steps = 5;
    }

    SequentialEvaluator::~SequentialEvaluator()
    {    }

    void SequentialEvaluator::cleanFiles(StringList files)
    { /*    */ }

    void SequentialEvaluator::multistepEvaluation(std::string reconstructionExe, std::string accuracyExe, std::string optimalCamExe)
    {
        DoubleList distances;
        distances.assign(steps, 0.0);
        std::string matches = "";

        std::ofstream globalOut(getOuputFile());

        for (int stepNumber = 0; stepNumber < steps; stepNumber++) {

            std::cout << "step #" << stepNumber << std::endl << std::endl;

            log("initOpenMvg");
            std::string basicFolder = initOpenMvg();
            std::string mvgJson = basicFolder + "/sfm_data.json";

            OpenMvgJsonHandler mvgJsonHandler(mvgJson);
            PoseList cameraPoses = getCameraPoses();
            for (int index = 0; index < cameraPoses.size(); index++) {
                mvgJsonHandler.setCamPosition(index, cameraPoses[index].first, cameraPoses[index].second);
            }
            mvgJsonHandler.saveChanges();

            log("perform reconstruction");
            OpenMvgSysCall::extractMvgFeatures(mvgJson, matches);

            log("\nCompute Structure");
            std::string mvgFolder = OpenMvgSysCall::computeStructureFromPoses(mvgJson, stepNumber, false);
            
            mvgJson = mvgFolder + "/sfm_data.json";
            std::string offFile = "output/from_gen_config/manifold_final.off";

            log(mvgJson);

            log("reconstruction");
            execute(reconstructionExe + " " + mvgJson);
            log("accuracy");
            execute(accuracyExe + " " + mvgJson + " " + offFile + " points.txt");
            log("optimal cam");
            execute(optimalCamExe + " " + mvgJson + " " + offFile + " points.txt optimalPose.txt");
            std::string optimalPosefile = "optimalPose.txt";
            
            // transform angles to look at
            log("transformation to lookat");
            std::string poseFile = transformToLookat(optimalPosefile);
            
            log("readpose");
            StringList posesString = InputReader::readDatabase(poseFile);
            IntStringPair entry(stepNumber, posesString[0]);

            log("evaluation");            
            double distance = evaluateIntersectedPose(entry, basicFolder);

            log("add camera to poses");
            Pose newPose = getPose(entry.second);

            appendToCameraPoses(newPose);

            distances[stepNumber] = distance;
            globalOut << stepNumber << " " << distance << std::endl;
        }
        globalOut.close();
    }

    double SequentialEvaluator::evaluateIntersectedPose(IntStringPair &entry, std::string &basicFolder)
    {
        log("\nParsing filename to pose");
        Pose pose = getPose(entry.second);

        log("\nDownloading image");
        std::string filename = getImage(entry);
        if (filename.size() == 0) return DBL_MAX;

        std::string jsonFile = basicFolder + "/sfm_data.json";

        log("\nMove file into folder");
        moveImageIntoImagesFolder(filename);
        
        log("\nAppend image");
        size_t imgId = appendImageToJson(jsonFile, filename);
        
        log("\nSet position camera");
        setPositionOfCameras(jsonFile, pose, imgId);

        std::string pairFile = "";

        OpenMvgSysCall::extractMvgFeatures(jsonFile, pairFile);

        log("\nCompute Structure");
        std::string mvgFolder = OpenMvgSysCall::computeStructureFromPoses(jsonFile, imgId, false);

        std::string inputCloud = mvgFolder + "/sfm_data.ply";

        // if (entry.first > 0) {
        //     std::string inputJson = mvgFolder + "/sfm_data.json";
        //     std::string originalJson = "poses_sfm_data_0/sfm_data.json";
        //     std::string intersectedFirst = mvgFolder + "/first_sfm_data.ply";
            
        //     PointCloudIntersecter intersect(originalJson, inputJson);
        //     intersect.write(intersectedFirst, inputCloud, entry.first);
        // }

        std::string gt = getGroundTruthFilename();
        
        log("\nCompute Distance");
        std::string logFile = computeDistance(gt, inputCloud); 

        log("\nExtract distance from log");
        double distance = parseDistance(logFile);     
        
        cleanFiles({filename, pairFile, mvgFolder, "images/"+filename, "matches/matches.f.bin"});

        return distance;
    }

    int SequentialEvaluator::execute(std::string command)
    {
        return system(command.c_str());
    }

    std::string SequentialEvaluator::transformToLookat(std::string &fixedPosefile)  // READ just one pose!
    {
        std::ifstream cin(fixedPosefile);
        
        std::vector<std::string> blocks;
        std::string val;

        for (int i = 0; i < 5; i++) {
            cin >> val;
            blocks.push_back(val);
        }
        cin.close();

        std::string newFile = "converted_poses.txt";
        std::ofstream cout(newFile);
        cout << blocks[0] << "_" << blocks[1] << "_" << blocks[2] << "_0.0_" << blocks[3] << "_" << blocks[4] <<  std::endl;
        cout.close();

        return newFile;
    }

} // namespace cameval
