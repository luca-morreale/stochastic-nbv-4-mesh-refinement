#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include <utilities.hpp>
#include <aliases.h>
#include <MiddleburyDatasetEvaluator.hpp>

#define ARGS 5

// FIXME remember to put initial images into views to avoid!!

int main(int argc, char **argv) {

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << std::string(argv[0]) << " steps accuracyExe poseFilename.txt inputImageFolder output_report.txt" << std::endl;
        return 1;
    }

    int steps = atoi(argv[1]);
    std::string accuracyExe = "./" + std::string(argv[2]);
    std::string poseFilename = argv[3];
    std::string inputImageFolder = argv[4];
    std::string outputEvaluation = argv[5]; // "output/output_evaluation_emulate.txt";

    std::string groundTruthFilename = "pcl_gt.asc";
    std::string basicPosesFilename = "poses_cam.txt";
    std::string baseImageFolder = "images";

    std::ofstream out("fake_poses.txt");
    out << "0 0 0 0 0" << std::endl;
    out.close();

    std::string reconstructionExe = "./manifoldReconstructor ";
    
    cameval::MiddleburyDatasetEvaluator eval(groundTruthFilename, poseFilename, baseImageFolder, basicPosesFilename, inputImageFolder, outputEvaluation);
    eval.datasetEvaluation(steps, reconstructionExe, accuracyExe);

    return 0;
}
