/*
 * OpenMvgParser.h
 *
 *  Created on: 16 mar 2016
 *      Author: Andrea Romanoni
 */

#ifndef CAM_PARSERS_OPENMVGPARSER_H_
#define CAM_PARSERS_OPENMVGPARSER_H_

#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

#include <rapidjson/document.h>
#include <rapidjson/reader.h>

#include <glm/glm.hpp>

#include <meshac/SfMData.h>
#include <meshac/type_definition.hpp>

#include <JsonException.hpp>


class OpenMvgParser {
public:
    OpenMvgParser(std::string path);
    virtual ~OpenMvgParser();
    virtual void parse();

    const meshac::SfMData& getSfmData() const {
      return sfm_data_;
    }

private:
    void parseViews(const std::map<int, glm::mat3>& intrinsics, const std::map<int, meshac::CameraType>& extrinsics);
    void parseIntrinsics(std::map<int, glm::mat3>& intrinsics);
    void parseExtrinsics(std::map<int, meshac::CameraType>& extrinsics);
    void parsePoints();


    rapidjson::Document document_;
    std::string fileName_;
    std::ifstream fileStream_;
    meshac::SfMData sfm_data_;
};

#endif /* CAM_PARSERS_OPENMVGPARSER_H_ */
