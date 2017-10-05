
#include <cstdlib>
#include <iostream>
#include <omp.h>

#include <OpenMvgParser.h>

#include <opview/type_definition.h>
#include <opview/MiddleburyDatasetScorer.hpp>
#include <opview/utilities.hpp>

#include <aliases.hpp>
#include <utilities.h>

#define TIMING

#define OMP_THREADS 8
#define ARGS 4

void removeViews(std::string &poses, opview::MiddleburyDatasetScorer &model);

int main(int argc, char **argv) {
    
    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " mgvjson.json meshfile.off accScore.txt output.txt" << std::endl;
        return 1;
    }

    std::string cameraPoses = "poses.txt";
    std::string jsonFile = argv[1];
    std::string meshFile = argv[2];
    std::string score = argv[3];
    std::string output = argv[4];
    
    OpenMvgParser op_openmvg(jsonFile);
    op_openmvg.parse();
    // std::cout << "sfm: " << op_openmvg.getSfmData().numCameras_ << " cams; " << op_openmvg.getSfmData().numPoints_ << " points" << std::endl << std::endl;

    SfMData sfm_data_ = op_openmvg.getSfmData();

    std::vector<glm::vec3> cams;
    for (auto cam : sfm_data_.camerasList_) {
        cams.push_back(cam.center);
    }

    auto scores = utilities::readScores(score);

    opview::MeshConfiguration meshConfig(meshFile, cams, scores.points, scores.normals, scores.uncertainty);
    std::cout << "points: " << scores.points.size() << std::endl;

    size_t maxPoints = 10;
    
    opview::MiddleburyDatasetScorer model(cameraPoses, meshConfig, maxPoints);
    removeViews(output, model);
    std::cout << "done removal" << std::endl;

#ifdef TIMING
    millis start = now();
#endif

    std::string result = model.estimateView();

#ifdef TIMING
        std::cout << std::endl << std::endl << "Total time to compute optimal pose: " << (now()-start).count() << "ms" << std::endl;
#endif

    std::ofstream out(output, std::ofstream::out | std::ofstream::app);
    out << result << std::endl;
    out.close();

    return 0;
}

void removeViews(std::string &poses, opview::MiddleburyDatasetScorer &model)
{
    std::ifstream cin(poses);
    std::string view;
    
    while(!cin.eof()) {
        cin >> view;
        std::cout << view << std::endl;
        if (view.empty()) continue;
        model.removeView(view);

        // for (int i = 0; i < 21; i++) {
        //     cin >> view;
        // }
    }
    cin.close();
}
