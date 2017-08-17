#include <EvaluatorDatabase.hpp>

namespace cameval {

    EvaluatorDatabase::EvaluatorDatabase(std::string &databaseFilename, std::string &groundTruthFilename, 
            std::string &basicPoseFilename, std::string &baseImageFolder, std::string &intrinsicParams, 
            std::string &outputFile, std::string &sshconfig) 
            : BasicEvaluator(databaseFilename, groundTruthFilename, basicPoseFilename, baseImageFolder, intrinsicParams, outputFile), 
            thresholdWaitSsh(THRESHOLD_SSH)
    {
        this->sshHandler = new SshHandler(sshconfig);
        this->mapper = new Mapper("output_mapping.txt", "database.txt");
    }

    EvaluatorDatabase::~EvaluatorDatabase()
    {
        delete sshHandler;
        delete mapper;
    }

    std::string EvaluatorDatabase::getImage(IntStringPair &entry)
    {
        std::chrono::milliseconds start = now();
        while (now() - start < thresholdWaitSsh) {
            try {
                return sshHandler->download(entry.second);
            } catch (const SSHException &ex) {
                std::cerr << ex.what() << std::endl;
                usleep(SLEEP_SSH_ERROR);
            }
            
        }
        appendToPoses(entry);
        return "";      // put back name in a map to redo
    }

    Pose EvaluatorDatabase::getPose(std::string &data)
    {
        // return Mapper::slowMappingToPose(data);    // FIXME this might be empty, so check, if not here just parse.
        return this->mapper->mapFileToPose(data);
    }

    MapperPtr EvaluatorDatabase::getMapper()
    {
        return mapper;
    }

} // namespace cameval
