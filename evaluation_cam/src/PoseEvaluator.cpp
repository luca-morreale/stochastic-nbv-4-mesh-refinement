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

        glm::mat3 rot = rotationMatrix(p.second);      // optimal cam put rad as output!!!
        return Pose(p.first, rot);
    }

    std::string PoseEvaluator::getImage(IntStringPair &entry)
    {
        // std::string file = extractFilename(entry.second);

        std::string file = "zz_" + getTimeStamp();  // so that it results after all the "model_*" images!
        std::ofstream out(file + ".pov");
        out << generatePovrayDeclaration(entry.second) << std::endl;
        out.close();

        std::string command = "cat " + basicPovFile + " >> " + file + ".pov";
        system(command.c_str());

        command = "povray " + file + ".pov -D default_config.ini +WT8";
        system(command.c_str());

        cleanFiles({file + ".pov"});

        return file + ".png";
    }

    std::string PoseEvaluator::generatePovrayDeclaration(std::string &data)
    {
        Pose pose = getPose(data);
        GLMVec3 z(0, 0, 10);
        GLMVec3 lookat = pose.second * z + pose.first;

        std::stringstream out;
        out << "#declare PdV=<" << pose.first.x << "," << pose.first.y << "," << pose.first.z << ">;\n";
        out << "#declare V=<" << lookat.x << "," << lookat.y << "," << lookat.z << ">;\n";
        return out.str();
    }

    std::string PoseEvaluator::getTimeStamp()
    {
        std::time_t seconds;
        std::time(&seconds);
        return boost::lexical_cast<std::string>(seconds);
    }

} // namespace cameval
