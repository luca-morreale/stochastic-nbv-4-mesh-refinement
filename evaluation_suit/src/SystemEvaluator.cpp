#include <SystemEvaluator.hpp>

namespace cameval {

    SystemEvaluator::SystemEvaluator(std::string &groundTruthFilename, std::string &basicPoseFilename, std::string &baseImageFolder, 
            std::string &intrinsicParams, std::string &reportFile, std::string &basicPovFile)
            : BasicEvaluator(groundTruthFilename, basicPoseFilename, baseImageFolder, intrinsicParams, reportFile)
    {
        this->basicPovFile = basicPovFile;
    }

    SystemEvaluator::~SystemEvaluator()
    { /*    */ }


    void SystemEvaluator::cleanFiles(StringList files)
    { /*    */ }

    void SystemEvaluator::systemEvaluation(int steps, std::string reconstructionExe, std::string accuracyExe, std::string optimalCamExe)
    {
        DoubleList distances;
        distances.assign(steps, 0.0);
        std::string matches = "";

        std::ofstream globalOut(getOuputFile());

        log("\n\nStart system evaluation, to perform" + std::to_string(steps) + " steps");
        for (int stepNumber = 0; stepNumber < steps; stepNumber++) {

            log("Start step #" + std::to_string(stepNumber));

            log("Init OpenMvg");
            std::string basicFolder = initOpenMvg();
            std::string mvgJson = basicFolder + "/sfm_data.json";

            OpenMvgSysCall::extractMvgFeatures(mvgJson, matches);

            log("\nCompute Structure");
            std::string mvgFolder = OpenMvgSysCall::computeStructureFromPoses(mvgJson, stepNumber, false);
            
            mvgJson = mvgFolder + "/sfm_data.json";
            std::string offFile = "output/from_gen_config/manifold_final.off";

            log("Reconstruction step");
            execute(reconstructionExe + " " + mvgJson);
            
            log("Accuracy step");
            execute(accuracyExe + " " + mvgJson + " " + offFile + " points.txt");
            
            log("Pptimal Cam step");
            std::string optimalPosefile = "optimalPose.txt";
            execute(optimalCamExe + " " + mvgJson + " " + offFile + " points.txt " + optimalPosefile);
            
            log("Transformation to output pose to look at");
            std::string poseFile = transformToLookat(optimalPosefile);
            
            log("Read new pose");
            StringList posesString = InputReader::readDatabase(poseFile);
            

            log("Evaluation new pose");            
            double distance = evaluatePose(posesString[0], basicFolder);
            distances[stepNumber] = distance;
            globalOut << stepNumber << " " << distance << std::endl;
            globalOut.flush();

            log("Add camera to poses");
            Pose newPose = getPose(posesString[0]);
            appendToCameraPoses(newPose);
        }
        globalOut.close();
    }

    std::string SystemEvaluator::initOpenMvg()
    {
        OpenMvgSysCall::initOpenMvg();

        setMvgJsonHandler(new OpenMvgJsonHandler("matches/sfm_data.json"));

        log("Set default camera poses");
        setDefaultCameraPoses();
        getMvgJsonHandler()->saveChanges();
        
        return "matches";
    }

    std::string SystemEvaluator::transformToLookat(std::string &fixedPosefile)  // READ just one pose!
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


    Pose SystemEvaluator::getPose(std::string &data)
    {
        AnglePose p = parseEntry(data);

        glm::mat3 rot = rotationMatrix(p.second);      // optimal cam put rad as output!!!
        return Pose(p.first, rot);
    }

    std::string SystemEvaluator::getImage(std::string &poseString)
    {
        std::string filePrefix = "zz_" + getTimeStamp();
        std::string file = filePrefix + ".pov";  // so that it results after all the "model_*" images!
        std::string imageFile = filePrefix + ".png";

        generatePovrayFile(file, poseString);

        std::string command = "povray " + file + " -D default_config.ini +WT8";
        system(command.c_str());

        cleanFiles({file});

        return imageFile;
    }

    void SystemEvaluator::generatePovrayFile(std::string &file, std::string &poseString)
    {
        std::ofstream out(file);
        out << generatePovrayDeclaration(poseString) << std::endl;
        out.close();

        std::string command = "cat " + this->basicPovFile + " >> " + file;
        system(command.c_str());
    }

    std::string SystemEvaluator::generatePovrayDeclaration(std::string &data)
    {
        Pose pose = getPose(data);
        GLMVec3 z(0, 0, 10);
        GLMVec3 lookat = pose.second * z + pose.first;

        std::stringstream out;
        out << "#declare PdV=<" << pose.first.x << "," << pose.first.y << "," << pose.first.z << ">;\n";
        out << "#declare V=<" << lookat.x << "," << lookat.y << "," << lookat.z << ">;\n";
        return out.str();
    }

    std::string SystemEvaluator::getTimeStamp()
    {
        std::time_t seconds;
        std::time(&seconds);
        return boost::lexical_cast<std::string>(seconds);
    }

} // namespace cameval
