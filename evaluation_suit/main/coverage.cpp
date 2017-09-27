
#include <PointKDTree.hpp>
#include <OpenMvgParser.hpp>

#include <omp.h>

#include <glm/glm.hpp>

#include <vector>
#include <array>


#define OMP_THREADS 8

#define ARGS 4

using namespace std;

#pragma omp declare reduction(vec_plus : std::vector<int> : \
                std::transform(omp_out.begin(), omp_out.end(), omp_in.begin(), omp_out.begin(), std::plus<int>())) \
                initializer(omp_priv = omp_orig)


vector<pair<string, string>> mapIntersectionOutput;
vector<glm::vec3> load(std::string &file);


int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " files_list.txt gt.asc end n" << std::endl;
        return 1;
    }

    std::string files_list = argv[1];
    std::string gt = argv[2];
    int end = atoi(argv[3]);
    int n = atoi(argv[4]);
    float scale = (float)end / (float)n;

    vector<glm::vec3> gtPoints = load(gt);

    std::ifstream cin(files_list);

    string original;
    string cloud, output;
    int counts;

    while (!cin.eof()) {
        cin >> counts >> original;

        if (counts == 0 || original.empty()) continue;

        for (int i = 0; i < counts; i++) {
            cin >> cloud >> output;
            mapIntersectionOutput.push_back(make_pair(cloud, output));
        }
    }
    cin.close();


    std::cout << mapIntersectionOutput.size() << std::endl;

    int i = 0;
    for (auto el : mapIntersectionOutput) {
        
        std::cout << i << "/" << mapIntersectionOutput.size() << " " << el.first << std::endl;
        OpenMvgParser parser(el.first);
        parser.parse();
        vector<glm::vec3> points = parser.getSfmData().points_;
        cameval::PointKDTree tree(points);

        vector<int> counts(n);
        for (int k = 0; k < n; k++) counts[k] = 0;

        
        for (int j = 0; j < gtPoints.size(); j++) {


            glm::vec3 closest = tree.searchClosestPoint(gtPoints[j]);
            float distance = glm::distance(closest, gtPoints[j]);
            
            for (int d = 0; d < n; d++) {
                if (distance < (float)(d+1) * scale) {
                    counts[d] += 1;
                }
            }
        }
        
        std::ofstream out(el.second + "_coverage.txt");
        for (int d = 0; d < 10; d++) {
            double c = (double)counts[d];
            double res = c / (double)gtPoints.size();
            out << (float)(d+1) * scale << " " << res << std::endl;
        }
        out.close();
        i++;
    }


    return 0;
}

vector<glm::vec3> load(std::string &file)
{
    vector<glm::vec3> list;
    std::ifstream ply(file);
    
    while(!ply.eof()) {
        glm::vec3 point;
        ply >> point.x >> point.y >> point.z;

        if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) continue;

        // if (point.x > 0.0f) continue;
        list.push_back(point);
    }
    ply.close();
    return list;
}
