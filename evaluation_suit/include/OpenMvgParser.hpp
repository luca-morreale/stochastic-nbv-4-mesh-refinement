/*
 * OpenMvgParser.h
 *
 *  Created on: 16 mar 2016
 *      Author: andrea
 */

#ifndef EVALUATION_CAMERA_POSITION_OPENMVG_PARSER_H_
#define EVALUATION_CAMERA_POSITION_OPENMVG_PARSER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include <rapidjson/document.h>
#include <rapidjson/reader.h>

#include <SfMData.h>
#include <JsonException.hpp>

namespace cameval {

    class OpenMvgParser {
    public:
      OpenMvgParser(std::string path);
      virtual ~OpenMvgParser();
      virtual void parse();

      const cameval::SfMData& getSfmData() const {
        return sfm_data_;
      }

    private:
      void parseViews(const std::map<int, glm::mat3> & intrinsics, const std::map<int, cameval::CameraType> & extrinsics);
      void parseIntrinsics(std::map<int, glm::mat3> & intrinsics);
      void parseExtrinsics(std::map<int, cameval::CameraType> & extrinsics);
      void parsePoints();


      rapidjson::Document document_;
      std::string fileName_;
      std::ifstream fileStream_;
      SfMData sfm_data_;

    };
} // namespace cameval

#endif /* EVALUATION_CAMERA_POSITION_OPENMVG_PARSER_H_ */
