
#include <omp.h>

#include <glm/glm.hpp>

#include <vector>
#include <regex>

#include <PointCloudIntersecter.hpp>

#define OMP_THREADS 8

#define ARGS 2

using namespace std;

std::regex distanceRegex = std::regex("\\[(.*)\\] \\[ComputeDistances\\] Mean distance = (.*) / std deviation = (.*)");


vector<pair<pair<string, string>, string>> mapIntersectionOutput;
double parseDistance(std::string &logFile);
// trim from start
std::string &ltrim(std::string &s);
// trim from end
std::string &rtrim(std::string &s);

// trim from both ends
std::string &trim(std::string &s);
std::string readStringFromFile(std::string &filename);


int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " files_list.txt gt.asc" << std::endl;
        return 1;
    }

    std::string files_list = argv[1];
    std::string gt = argv[2];


    std::ifstream cin(files_list);

    string original;
    string cloud, output;
    int counts;

    while (!cin.eof()) {
        cin >> counts >> original;

        if (counts == 0 || original.empty()) continue;

        for (int i = 0; i < counts; i++) {
            cin >> cloud >> output;
            mapIntersectionOutput.push_back(make_pair(make_pair(original, cloud), output));
        }
    }
    cin.close();


    std::cout << mapIntersectionOutput.size() << std::endl;

    int i = 0;
    for (auto el : mapIntersectionOutput) {
        std::cout << i++ << "/" << mapIntersectionOutput.size() << " ";
        std::cout << el.first.first << " " << el.first.second << std::endl;
        if (el.first.second.empty()) break;
        cameval::PointCloudIntersecter intersecter(el.first.first, el.first.second);

        // write(std::string outPC1, std::string outPC2)
        intersecter.write(el.second + "_original.ply", el.second + "_cloud.ply");

        std::string logfilename = el.second + "_vs_gt.txt";
        std::string command = "CloudCompare -SILENT -LOG_FILE " + logfilename + " -O " + el.second + "_cloud.ply" + " -O " + gt + " -c2c_dist -MAX_DIST 10";
        system(command.c_str());

        double dst1 = parseDistance(logfilename);

        command = "CloudCompare -SILENT -LOG_FILE " + logfilename + " -O " + el.second + "_original.ply" + " -O " + gt + " -c2c_dist -MAX_DIST 10";
        system(command.c_str());

        double dst2 = parseDistance(logfilename);


        command = "echo \"original new \n\"" + std::to_string(dst2) + " " + std::to_string(dst1) + " > " + logfilename;  // first original then new cloud
        system(command.c_str());
    }


    return 0;
}

double parseDistance(std::string &logFile)
{
    std::string content = readStringFromFile(logFile);
    
    std::string line;
    std::istringstream strstream(content);
       
    while (std::getline(strstream, line)) {
        line = trim(line);
        if (!std::regex_match(line.begin(), line.end(), distanceRegex)) {
            continue;
        }

        std::smatch sm;
        std::regex_match(line, sm, distanceRegex);

        // Index 0 is whole line
        // Index 1- ... matches
        double distance = std::stod(sm[2]);
        double std = std::stod(sm[3]);

        return distance;
    }
    std::cerr << "Pattern not found" << std::endl;
    return DBL_MAX;
}

// trim from start
std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// trim from both ends
std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

std::string readStringFromFile(std::string &filename)
{
    std::ifstream cin(filename);
    std::string content((std::istreambuf_iterator<char>(cin)), std::istreambuf_iterator<char>());
    cin.close();
    return content;
}