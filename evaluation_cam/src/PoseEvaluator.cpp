#include <PoseEvaluator.hpp>

namespace cameval {

    PoseEvaluator::PoseEvaluator(std::string &pointFile, std::string &groundTruthFilename, 
            std::string &basicPoseFilename, std::string &baseImageFolder, std::string &intrinsicParams, 
            std::string &reportFile, std::string &basicPovFile)
            : BasicEvaluator(pointFile, groundTruthFilename, basicPoseFilename, baseImageFolder, intrinsicParams, reportFile)
    {
        this->basicPovFile = basicPovFile;
    }

    PoseEvaluator::~PoseEvaluator()
    { }

    Pose PoseEvaluator::getPose(std::string &data)
    {
        AnglePose p = parseEntry(data);

        glm::mat4 view = glm::lookAt(p.first, p.second, GLMVec3(0, -1, 0));
        glm::mat3 rot(view);

        return Pose(p.first, rot);
    }

    std::string PoseEvaluator::getImage(IntStringPair &entry)
    {
        // std::string file = extractFilename(entry.second);
        std::cout << entry.second << std::endl;
        std::string file = "tmp_povray_render";
        std::ofstream out(file + ".pov");
        out << generatePovrayDeclaration(entry.second) << std::endl;
        out.close();

        std::string command = "cat " + basicPovFile + " >> " + file + ".pov";
        system(command.c_str());

        command = "povray " + file + ".pov -D default_config.ini +WT8";
        system(command.c_str());

        FileHandler::cleanAll({file + ".pov"});

        return file + ".png";
    }

    std::string PoseEvaluator::generatePovrayDeclaration(std::string &data)
    {
        StringList blocks = divideStringPose(data);

        std::string out;
        out += "#declare PdV=<";
        
        for (int i = 0; i < 3; i++) {
            out += blocks[i];
            if (i < 2) out += ",";
        }
        out += ">;\n";
        out += "#declare V=<";
        for (int i = 0; i < 3; i++) {
            out += blocks[i + 3];
            if (i < 2) out += ",";
        }
        out += ">;\n";
        return out;
    }

} // namespace cameval
