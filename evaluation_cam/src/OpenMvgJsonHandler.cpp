#include <OpenMvgJsonHandler.hpp>

namespace cameval {

    OpenMvgJsonHandler::OpenMvgJsonHandler(std::string path) {
        setFile(path);
    }

    OpenMvgJsonHandler::~OpenMvgJsonHandler() {
        cin.close();
    }

    void OpenMvgJsonHandler::setFile(std::string path)
    {
        fileName = path;
        cin.open(path.c_str());
    }

    void OpenMvgJsonHandler::parse(std::string path)
    {
        setFile(path);
        parse();
    }
    
    void OpenMvgJsonHandler::parse() {

        JsonFINWrapper isw(cin);
        document.ParseStream(isw);
        // std::string str((std::istreambuf_iterator<char>(cin)), std::istreambuf_iterator<char>());
        // document.Parse(str.c_str());

        if (!document.IsObject()) {
            throw JsonParseException("JsonParseException--> the json file " + fileName + " is not valid");
        }
        checkIntrinsics();
        checkExtrinsics();
        checkViews();
        checkPoints();
    }

    void OpenMvgJsonHandler::checkViews()
    {
        if (!document.HasMember("views")) {
            throw JsonAccessException("JsonAccessException--> error while querying HasMember(views)");
        }
    }

    void OpenMvgJsonHandler::checkPoints()
    {
        if (!document.HasMember("structure")) {
            throw JsonAccessException("JsonAccessException--> error while querying HasMember(structure)");
        }
        const JsonValue& structure = document["structure"];

        if (!structure.IsArray()) {
            throw JsonAccessException("JsonAccessException--> error while querying structure.IsArray()");
        }

        for (rapidjson::SizeType curPoint = 0; curPoint < structure.Size(); curPoint++) { // Uses SizeType instead of size_t

            if (!structure[curPoint].IsObject()) {
                throw JsonAccessException("JsonAccessException--> error while querying structure[i].IsObject()");
            }
            
            if (!structure[curPoint].HasMember("value")) {
                throw JsonAccessException("JsonAccessException--> error while querying structure[i].HasMember(value)");
            }
            if (!structure[curPoint]["value"].HasMember("X"))  {
                throw JsonAccessException("JsonAccessException--> error while querying structure[i][value].HasMember(X)");
            }

            const JsonValue& X = structure[curPoint]["value"]["X"];
            if (!X.IsArray()) {
                throw JsonAccessException("JsonAccessException--> error while querying X.IsArray()");
            }

            if (!X[0].IsDouble()) {
                throw JsonAccessException("JsonAccessException--> error while querying X0.IsDouble()");
            }

            if (!X[1].IsDouble()) {
                throw JsonAccessException("JsonAccessException--> error while querying X1.IsDouble()");
            }

            if (!X[2].IsDouble()) {
                throw JsonAccessException("JsonAccessException--> error while querying X2.IsDouble()");
            }

            if (!structure[curPoint]["value"].HasMember("observations")) {
                throw JsonAccessException("JsonAccessException--> error while querying structure[i][value].HasMember(observations)");
            }

            const JsonValue& observations = structure[curPoint]["value"]["observations"];

            if (!observations.IsArray()) {
                throw JsonAccessException("JsonAccessException--> error while querying observations.IsArray()");
            }

            for (rapidjson::SizeType curId = 0; curId < observations.Size(); curId++) {
                if (!observations[curId].HasMember("key")) {
                    throw JsonAccessException("JsonAccessException--> error while querying observations[i].HasMember(key)");
                }
            }
        }
    }

    void OpenMvgJsonHandler::checkIntrinsics()
    {
        if (!document.HasMember("intrinsics")) {
            throw JsonAccessException("JsonAccessException--> error while querying HasMember(views)");
        }
    }

    void OpenMvgJsonHandler::checkExtrinsics()
    {
        if (!document.HasMember("extrinsics")) {
            throw JsonAccessException("JsonAccessException--> error while querying HasMember(extrinsics)");
        }
    }

    void OpenMvgJsonHandler::checkCameraExistence(int camIndex)
    {
        if (!document.HasMember("extrinsics")) {
            throw JsonAccessException("JsonAccessException--> error while querying HasMember(extrinsics)");
        }
        getCameraRef(camIndex);
    }

    void OpenMvgJsonHandler::addFieldToDocuemnt(std::string name, JsonValue &content)
    {
        JsonValue key(name.c_str(), document.GetAllocator());
        document.AddMember(key, content, document.GetAllocator());
    }

    void OpenMvgJsonHandler::addFieldTo(JsonValue &parent, std::string name, JsonValue &content)
    {
        parent.AddMember(JsonValue(name.c_str(), document.GetAllocator()).Move(), content, document.GetAllocator());
    }

    void OpenMvgJsonHandler::addFieldTo(JsonValue &parent, std::string name, std::string content)
    {
        parent.AddMember(JsonValue(name.c_str(), document.GetAllocator()).Move(), 
            JsonValue(content.c_str(), document.GetAllocator()).Move(), document.GetAllocator());
    }

    void OpenMvgJsonHandler::addFieldTo(JsonValue &parent, std::string name, int content)
    {
        parent.AddMember(JsonValue(name.c_str(), document.GetAllocator()).Move(), content, document.GetAllocator());
    }

    void OpenMvgJsonHandler::addFieldTo(JsonValue &parent, std::string name, size_t content)
    {
        parent.AddMember(JsonValue(name.c_str(), document.GetAllocator()).Move(), content, document.GetAllocator());
    }

    void OpenMvgJsonHandler::addFieldTo(JsonValue &parent, std::string name, double content)
    {
        parent.AddMember(JsonValue(name.c_str(), document.GetAllocator()).Move(), content, document.GetAllocator());
    }

    void OpenMvgJsonHandler::pushBackTo(JsonValue &array, JsonValue &content)
    {
        array.PushBack(content, document.GetAllocator());
    }

    void OpenMvgJsonHandler::pushBackTo(JsonValue &array, double value, int n)
    {
        for (size_t i = 0; i < n; i++) {
            array.PushBack(value, document.GetAllocator());
        }
    }

    void OpenMvgJsonHandler::addExtrinsicMember()
    {
        JsonValue extrinsics(JsonArray);
        addFieldToDocuemnt("extrinsics", extrinsics);
    }    

    void OpenMvgJsonHandler::addCameraToExtrinsic(int camIndex)
    {
        JsonValue camera(JsonObject);
        JsonValue key(JsonNumber);
        JsonValue values(JsonObject);
        JsonValue center(JsonArray);
        JsonValue rotation(JsonArray);
        JsonValue row1(JsonArray);
        JsonValue row2(JsonArray);
        JsonValue row3(JsonArray);

        key.SetInt(camIndex);

        pushBackTo(center, 0.0, 3);    
        pushBackTo(row1, 0.0, 3);
        pushBackTo(row2, 0.0, 3);
        pushBackTo(row3, 0.0, 3);
        pushBackTo(rotation, row1);
        pushBackTo(rotation, row2);
        pushBackTo(rotation, row3);
            
        addFieldTo(values, "rotation", rotation);
        addFieldTo(values, "center", center);

        addFieldTo(camera, "key", key);
        addFieldTo(camera, "value", values);

        document["extrinsics"].PushBack(camera, document.GetAllocator());
    }

    int OpenMvgJsonHandler::getCameraRef(int camIndex)
    {
        const JsonValue& extrinsicsValue = document["extrinsics"];
        for (JsonSizeT curInt = 0; curInt < extrinsicsValue.Size(); curInt++) {
            int key = extrinsicsValue[curInt]["key"].GetInt();
            if (key == camIndex) {
                return curInt;
            }
        }
        throw CameraNotPresentException("Camera n° " + std::to_string(camIndex) + " not contained in the json file."); 
    }

    void OpenMvgJsonHandler::setCamPosition(int camIndex, GLMVec3 &camCenter, GLMMat3 rotation)  // cam index = image index
    {
        try {
            checkCameraExistence(camIndex);         // all okay
        } catch (const JsonAccessException &exc) {
            addExtrinsicMember();
            addCameraToExtrinsic(camIndex);
        } catch (const CameraNotPresentException &exc) {
            addCameraToExtrinsic(camIndex);
        }

        JsonValue& camera = document["extrinsics"][getCameraRef(camIndex)];
        camera["value"]["center"][0].SetFloat(camCenter.x);
        camera["value"]["center"][1].SetFloat(camCenter.y);
        camera["value"]["center"][2].SetFloat(camCenter.z);

        for (int curR = 0; curR < 3; curR++) {
            for (int curC = 0; curC < 3; curC++) {
                camera["value"]["rotation"][curR][curC].SetFloat(rotation[curR][curC]);
            }
        }
    }

    GLMVec3 OpenMvgJsonHandler::getCamPosition(int camIndex)
    {
        checkCameraExistence(camIndex);     // if right cam does not exits it dies due to exception

        const JsonValue& extrinsicsValue = document["extrinsics"];

        GLMVec3 C;
        C.x = extrinsicsValue[camIndex]["value"]["center"][0].GetFloat();
        C.y = extrinsicsValue[camIndex]["value"]["center"][1].GetFloat();
        C.z = extrinsicsValue[camIndex]["value"]["center"][2].GetFloat();
        
        return C;
    }

    bool OpenMvgJsonHandler::testCamPosition(int camIndex, GLMVec3 &camCenter)
    {
        GLMVec3 pos = getCamPosition(camIndex);

        return camCenter.x == pos.x && camCenter.y == pos.y && camCenter.z == pos.z;
    }

    int OpenMvgJsonHandler::countImages()
    {
        return document["views"].Size();
    }

    void OpenMvgJsonHandler::appendImage(std::string &imagePath, std::string &imageName)
    {
        const JsonValue &sampleElement = document["views"][countImages() - 1];

        JsonValue image(JsonObject);
        JsonValue value(JsonObject);
        JsonValue wrapper(JsonObject);
        JsonValue data(JsonObject);

        addFieldTo(data, "local_path", "");             // FIXME
        addFieldTo(data, "filename", imageName);
        addFieldTo(data, "width", sampleElement["value"]["ptr_wrapper"]["data"]["width"].GetInt());
        addFieldTo(data, "height", sampleElement["value"]["ptr_wrapper"]["data"]["height"].GetInt());
        addFieldTo(data, "id_view", sampleElement["value"]["ptr_wrapper"]["data"]["id_view"].GetInt() + 1);
        addFieldTo(data, "id_intrinsic", sampleElement["value"]["ptr_wrapper"]["data"]["id_intrinsic"].GetInt());
        addFieldTo(data, "id_pose", sampleElement["value"]["ptr_wrapper"]["data"]["id_pose"].GetInt() + 1);

        addFieldTo(wrapper, "id", (size_t)sampleElement["value"]["ptr_wrapper"]["id"].GetUint() + 1);
        addFieldTo(wrapper, "data", data);

        addFieldTo(value, "polymorphic_id", (size_t)sampleElement["value"]["polymorphic_id"].GetUint());
        addFieldTo(value, "ptr_wrapper", wrapper);

        addFieldTo(image, "key", countImages());
        addFieldTo(image, "value", value);

        document["views"].PushBack(image, document.GetAllocator());
    }

    void OpenMvgJsonHandler::saveChanges(std::string newFilename)
    {
        std::ofstream cout(newFilename);
        JsonFOUTWrapper osw(cout);
        JsonPrettyWriter writer(osw);
        document.Accept(writer);

        cout.close();
    }


} // cameval














