
#include <PointKDTree.hpp>
#include <OpenMvgParser.hpp>

#include <omp.h>

#include <glm/glm.hpp>

#include <vector>
#include <string>
#include <array>


#define OMP_THREADS 8

#define ARGS 5

using namespace std;

#pragma omp declare reduction(vec_plus : std::vector<int> : \
                std::transform(omp_out.begin(), omp_out.end(), omp_in.begin(), omp_out.begin(), std::plus<int>())) \
                initializer(omp_priv = omp_orig)


vector<pair<string, string>> mapIntersectionOutput;
vector<glm::vec3> load(std::string &file);


int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " gt.ply test_file.ply coverage_end coverage_n prefix" << std::endl;
        return 1;
    }

    std::string gt = argv[1];
    std::string testFile = argv[2];

    float coverage_end = strtod(argv[3], NULL);
    int n = atoi(argv[4]);
    float scale = coverage_end / (float)n;

    std::string prefix = argv[5];

    vector<glm::vec3> gtPoints = load(gt);
    std::cout << "size gt points: " << gtPoints.size() << std::endl;

    vector<glm::vec3> testPoints = load(testFile);
    std::cout << "size test points: " << testPoints.size() << std::endl;

    cameval::PointKDTree tree(testPoints);

    vector<int> counts(n);
    for (int j = 0; j < gtPoints.size(); j++) {
        glm::vec3 closest = tree.searchClosestPoint(gtPoints[j]);
        float distance = glm::distance(closest, gtPoints[j]);
            
        for (int d = 0; d < n; d++) {
            if (distance < (float)(d+1) * scale) {
                counts[d] += 1;
            }
        }
    }
        
    std::ofstream out(testFile + "_" + prefix + "_coverage.txt");
    for (int d = 0; d < n; d++) {
        double c = (double)counts[d];
        double res = c / (double)gtPoints.size();
        out << (float)(d+1) * scale << " " << res << std::endl;
    }
    out.close();

    return 0;
}

vector<glm::vec3> load(std::string &file)
{
    vector<glm::vec3> list;
    std::ifstream ply(file);

    while (!ply.eof()) {
        std::string buf;
        ply >> buf;
        //ply.getline(buf, 100);
        //std::cout << buf << std::endl;
        //if (strcmp(buf, "end_header") == 0) break;
        if (buf.compare("end_header") == 0) break;
    }
   
    //std::cout << buf << std::endl;

    while (!ply.eof()) {
        double a, b, c, d;
        glm::vec3 point;
        ply >> point.x >> point.y >> point.z;
        ply >> a >> b >> c; // colors
        ply >> a >> b >> c; // normal vectors

        if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) continue;

        // if (point.x > 0.0f) continue;
        list.push_back(point);
        //std::cout << list.size() << std::endl;
    }
    ply.close();
    return list;
}

