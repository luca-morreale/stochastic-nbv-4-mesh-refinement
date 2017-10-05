#ifndef EVALUATION_CAMERA_POSITION_UTILITIES_H_
#define EVALUATION_CAMERA_POSITION_UTILITIES_H_

#include <algorithm> 
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <locale>
#include <unistd.h>

#include <boost/algorithm/string.hpp>

#include <aliases.h>
#include <type_definition.h>

namespace cameval {

    std::chrono::milliseconds now();

    void log(std::string msg);

    std::string &ltrim(std::string &s);
    std::string &rtrim(std::string &s);
    std::string &trim(std::string &s);
    std::string readStringFromFile(std::string &filename);
    std::string concatBlocks(StringList &blocks);
    std::string extractFilename(std::string &entry);
    std::string removePartsFromEntry(std::string &entry);
    StringList divideStringPose(std::string &stringPose);
    AnglePose parseEntry(std::string &entry);
    std::string getPoseString(std::string &entry);
    int countBlocks(std::string &str);

    void eraseFromVector(IntList &removeIndex, GLMVec3List &list);

    GLMVec3 convert(EigVector3 &arr);
    GLMMat3 convert(EigMatrix3 &mat);
    Float6Array convert(StringList &list);

    float rad(float deg);
    float deg(float rad);
    GLMMat3 rotationMatrix(float yaw, float pitch, float roll);
    GLMMat3 rotationMatrix(GLMVec3 &angles);
    GLMMat3 rotationRoll(float roll);
    GLMMat3 rotationPitch(float pitch);
    GLMMat3 rotationYaw(float yaw);

    Camera getCamera(StringList blocks);
    GLMMat4 getRotation(StringList blocks);
    GLMMat4 getIntrinsic(StringList blocks);
    GLMVec4 getTranslation(StringList blocks);

    template<typename K, typename V>
    std::vector<K> keys(std::map<K, V> &map)
    {
        std::vector<K> keys;
        for (auto pair : map) {
            keys.push_back(pair.first);
        }
        return keys;
    }

    template<typename K, typename V>
    std::vector<K> keys(std::unordered_map<K, V> &map)
    {
        std::vector<K> keys;
        for (auto pair : map) {
            keys.push_back(pair.first);
        }
        return keys;
    }

    template<typename K, typename V>
    std::vector<V> values(std::map<K, V> &map)
    {
        std::vector<V> values;
        for (auto pair : map) {
            values.push_back(pair.second);
        }
        return values;
    }

    template<typename K, typename V>
    std::vector<V> values(std::unordered_map<K, V> &map)
    {
        std::vector<V> values;
        for (auto pair : map) {
            values.push_back(pair.second);
        }
        return values;
    }

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_UTILITIES_H_
