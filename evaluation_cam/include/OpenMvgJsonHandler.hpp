#ifndef EVALUATION_CAMERA_POSITION_OPENMVG_JSON_HANDLER_H_
#define EVALUATION_CAMERA_POSITION_OPENMVG_JSON_HANDLER_H_

#include <fstream>
#include <iostream>

#include <aliases.h>
#include <Exceptions.hpp>

namespace cameval {

    class OpenMvgJsonHandler {
    public:
        OpenMvgJsonHandler(std::string path);
        virtual ~OpenMvgJsonHandler();
        virtual void parse(std::string path);
        virtual void parse();

        virtual int countImages();
        virtual void appendImage(std::string &imagePath, std::string &imageFile);


        virtual void setCamPosition(int camIndex, GLMVec3 &camCenter, GLMMat3 rotation=GLMMat3());  // cam index = image index
        virtual GLMVec3 getCamPosition(int camIndex);
        virtual bool testCamPosition(int camIndex, GLMVec3 &camCenter);

        virtual void saveChanges(std::string newFilename);

        void setFile(std::string path);

    protected:
        virtual void checkViews();
        virtual void checkIntrinsics();
        virtual void checkExtrinsics();
        virtual void checkPoints();
        virtual void checkCameraExistence(int camIndex);

        int getCameraRef(int camIndex);

        void addFieldToDocuemnt(std::string name, JsonValue &content);
        void addFieldTo(JsonValue &parent, std::string name, JsonValue &content);
        void addFieldTo(JsonValue &parent, std::string name, std::string content);
        void addFieldTo(JsonValue &parent, std::string name, int content);
        void addFieldTo(JsonValue &parent, std::string name, size_t content);
        void addFieldTo(JsonValue &parent, std::string name, double content);
        void addExtrinsicMember();
        void pushBackTo(JsonValue &array, JsonValue &content);
        void pushBackTo(JsonValue &array, double value, int n);
        void addCameraToExtrinsic(int camIndex);

    private:
        JsonDoc document;
        std::string fileName;
        std::ifstream cin;
        
    };

} // cameval

#endif /* EVALUATION_CAMERA_POSITION_OPENMVG_JSON_HANDLER_H_ */
