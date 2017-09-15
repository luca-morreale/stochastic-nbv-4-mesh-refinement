#include <EvaluatorDatabase.hpp>

namespace cameval {

    EvaluatorDatabase::EvaluatorDatabase(std::string &databaseFilename, std::string &groundTruthFilename, 
            std::string &basicPoseFilename, std::string &baseImageFolder, std::string &intrinsicParams, 
            std::string &outputFile, std::string &sshconfig) : thresholdWaitSsh(THRESHOLD_SSH),
            BasicEvaluator(groundTruthFilename, basicPoseFilename, baseImageFolder, intrinsicParams, outputFile)
    {
        this->sshHandler = new SshHandler(sshconfig);

        log("\nSet poses to analyze");
        setPosesToEvaluate(databaseFilename);

        log("\nSet images prefix and suffix");
        setImagesPattern();
    }

    EvaluatorDatabase::~EvaluatorDatabase()
    {
        delete sshHandler;
    }

    void EvaluatorDatabase::setPosesToEvaluate(std::string posesFilename)
    {
        this->posesFilename = posesFilename;
        log("\nPopulating Poses To be Analyzed");
        posesList = InputReader::readDatabase(posesFilename);
        remapListToQueue(posesList);
        // log("\nPoses size: " + std::to_string(posesList.size()));
    }

    void EvaluatorDatabase::remapListToQueue(StringList &dataList)
    {
        for (int i = 0; i < dataList.size(); i++) {
            this->poses.push(IntStringPair(i, dataList[i]));
        }
    }

    void EvaluatorDatabase::setImagesPattern()
    {
        std::string fileExample = poses.front().second;
        this->filePrefix = fileExample.substr(0, fileExample.find_first_of("_"));
        this->fileExtention = fileExample.substr(fileExample.find_last_of(".") + 1, fileExample.size());
    }

    void EvaluatorDatabase::appendToPoses(IntStringPair &file)
    {
        poses.push(file);
    }

    void EvaluatorDatabase::evaluateDatabase()
    {
        log("\n\nStart evaluation");
        std::ofstream out(this->getOuputFile());

        log("Init openMvg");
        std::string basicFolder = initOpenMvg();
        
        DoubleList distances;
        distances.assign(poses.size(), 0.0);

        log("Start loop over poses");
        while (!poses.empty()) {
            IntStringPair entry = poses.front();
            poses.pop();

            log("Start analysis of: " + std::to_string(entry.first));
            double tmp = evaluatePose(entry, basicFolder);
            distances[entry.first] = tmp;

            if (tmp != DBL_MAX) {
                out << entry.first << " " << distances[entry.first] << std::endl;
                out.flush();
                log("Done analysis of: " + std::to_string(entry.first));
            }            
        }

        log("\n\nDone computation of all poses");
        
        int index = getIndexOfSmallestDistance(distances);

        log("min distance: " + std::to_string(distances[index]));
        log("pose: " + posesList[index]);

        out << "min distance: " << distances[index] << std::endl;
        out << "pose: " << posesList[index] << std::endl;

        out.close();

        cleanFiles({basicFolder, "matches/", "poses_sfm_data/"});
    }

    double EvaluatorDatabase::evaluatePose(IntStringPair &entry, std::string &basicFolder)
    {
        double distance = super::evaluatePose(entry.second, basicFolder);

        if (distance == DBL_MAX) {
            appendToPoses(entry);
        }
        return distance;
    }

    int EvaluatorDatabase::getIndexOfSmallestDistance(DoubleList &distances)
    {
        int index = 0;
        for (int i = 0; i < distances.size(); i++) {
            if (distances[i] < distances[index]) {
                index = i;
            }
        }
        return index;
    }

    std::string EvaluatorDatabase::initOpenMvg()
    {
        OpenMvgSysCall::initOpenMvg();

        setMvgJsonHandler(new OpenMvgJsonHandler("matches/sfm_data.json"));

        log("Set default camera poses");
        setDefaultCameraPoses();
        getMvgJsonHandler()->saveChanges();

        generateBasicPairFile();

        return "matches";
    }

    void EvaluatorDatabase::generateBasicPairFile()
    {
        std::string pairFile = "matches_pairs.txt";
        if (!FileHandler::checkSourceExistence(pairFile)) {
            std::ofstream out("matches_pairs.txt");
            for (int i = 0; i < getCameraPoses().size() - 1; i++) {
                for (int j = i + 1; j < getCameraPoses().size(); j++) {
                    out << i << " " << j << std::endl;
                }
            }
            out.close();
        }
    }

    std::string EvaluatorDatabase::getImage(std::string &imageName)
    {
        std::chrono::milliseconds start = now();
        while (now() - start < thresholdWaitSsh) {
            try {
                return sshHandler->download(imageName);
            } catch (const SSHException &ex) {
                std::cerr << ex.what() << std::endl;
                usleep(SLEEP_SSH_ERROR);
            }
            
        }
        return "";      // put back name in a map to redo
    }

    Pose EvaluatorDatabase::getPose(std::string &imageName) // FIXME assume no need of mapping
    {
        // // return Mapper::slowMappingToPose(data);    // FIXME this might be empty, so check, if not here just parse.
        // return this->mapper->mapFileToPose(data);
        return Pose();
    }

} // namespace cameval
