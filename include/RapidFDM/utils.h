//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_UTILS_H_H
#define RAPIDFDM_UTILS_H_H

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

namespace RapidFDM {
    namespace Utils {

        inline double fast_value(const rapidjson::Value &_json, std::string key, float default_value = 0) {
            if (_json.HasMember(key.c_str())) {
                if (_json[key.c_str()].IsInt())
                    return _json[key.c_str()].GetInt();
                else if (_json[key.c_str()].IsDouble())
                    return _json[key.c_str()].GetDouble();
            }
            return default_value;
        }

        inline double fast_value(const rapidjson::Value &_json, int index, double default_value = 0) {
            if (_json.IsArray() && _json.Size() > index) {
                if (_json[index].IsInt())
                    return _json[index].GetInt();
                else if (_json[index].IsDouble())
                    return _json[index].GetDouble();
            }
            return default_value;
        }

        inline Eigen::Vector3d fast_vector3(const rapidjson::Value &_json, std::string key,
                                            Eigen::Vector3d optional_value = Eigen::Vector3d(0, 0, 0)) {
            if (_json.HasMember(key.c_str()) && _json[key.c_str()].IsArray()) {
                const rapidjson::Value &array = _json[key.c_str()];
                return Eigen::Vector3d(fast_value(array, 0), fast_value(array, 1), fast_value(array, 2));
            }
            std::cerr << "Get vector of key <" << key << "> failed" << std::endl;
            return optional_value;
        }

        inline Eigen::Quaterniond fast_quaternion(const rapidjson::Value &_json, std::string key) {
            if (_json.HasMember(key.c_str()) && _json[key.c_str()].IsArray()) {
                const rapidjson::Value &array = _json[key.c_str()];
                return Eigen::Quaterniond(fast_value(array, 0),
                                          fast_value(array, 1),
                                          fast_value(array, 2),
                                          fast_value(array, 3));
            }
            std::cerr << "Get quaternion of key <" << key << "> failed" << std::endl;
            return Eigen::Quaterniond(1, 0, 0, 0);
        }


        inline std::string fast_string(const rapidjson::Value &_json, std::string key) {
            if (_json.HasMember(key.c_str()) && _json[key.c_str()].IsString()) {
                return _json[key.c_str()].GetString();
            }
            std::cerr << "Get string of key <" << key << "> failed" << std::endl;
            return "";
        }

        inline Eigen::Quaterniond fast_attitude(const rapidjson::Value &_json, std::string key) {

            if (_json.HasMember(key.c_str()) && _json[key.c_str()].IsArray()) {
                if (_json[key.c_str()].Size() == 4) {
                    //Is Quaternion
                    return fast_quaternion(_json, key);
                }
                else {
                    //As Euler Angles Roll Pitch Yaw
                    Eigen::Vector3d euler_angles = fast_vector3(_json, key);
                    Eigen::AngleAxisd rollAngle(euler_angles.x() * M_PI / 180, Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd pitchAngle(euler_angles.y() * M_PI / 180, Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd yawAngle(euler_angles.z() * M_PI / 180, Eigen::Vector3d::UnitZ());
                    Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;
                    return quat;
                }
            }

            std::cerr << "Get attitude of key <" << key << "> failed" << std::endl;
            return Eigen::Quaterniond(1, 0, 0, 0);
        }
        inline Eigen::Affine3d fast_transform(const rapidjson::Value & _json,std::string key)
        {
            if (_json.HasMember(key.c_str()) && _json[key.c_str()].IsObject()) {
                Eigen::Affine3d trans;
                trans.fromPositionOrientationScale(
                        fast_vector3(_json[key.c_str()],"vector"),
                        fast_attitude(_json[key.c_str()],"attitude"),
                        Eigen::Vector3d(1,1,1)
                );
                return trans;
            }
            return Eigen::Affine3d::Identity();
        }

        inline void add_attitude(rapidjson::Value &_json, Eigen::Quaterniond trans, rapidjson::Document &d,
                                 std::string name = "attitude") {
            rapidjson::Value v(rapidjson::kArrayType);
            rapidjson::Value namev(rapidjson::kStringType);
            namev.SetString(name.c_str(),d.GetAllocator());
            v.PushBack(trans.w(), d.GetAllocator());
            v.PushBack(trans.x(), d.GetAllocator());
            v.PushBack(trans.y(), d.GetAllocator());
            v.PushBack(trans.z(), d.GetAllocator());
            _json.AddMember(namev, v, d.GetAllocator());
        }

        inline void add_value(rapidjson::Value & _json,double v,rapidjson::Document & d,std::string name)
        {
            rapidjson::Value namev(rapidjson::kStringType);
            namev.SetString(name.c_str(),d.GetAllocator());
            _json.AddMember(namev, v, d.GetAllocator());
        }
        inline void add_vector(rapidjson::Value &_json, Eigen::Vector3d vec, rapidjson::Document &d,
                               std::string name = "vector") {
            rapidjson::Value v(rapidjson::kArrayType);
            rapidjson::Value namev(rapidjson::kStringType);
            namev.SetString(name.c_str(),d.GetAllocator());
            v.PushBack(vec.x(), d.GetAllocator());
            v.PushBack(vec.y(), d.GetAllocator());
            v.PushBack(vec.z(), d.GetAllocator());
            _json.AddMember(namev, v, d.GetAllocator());
        }

        inline void add_transform(rapidjson::Value &_json, Eigen::Affine3d trans, rapidjson::Document &d,
                                  std::string name = "transform") {
            rapidjson::Value v(rapidjson::kObjectType);
            rapidjson::Value namev(rapidjson::kStringType);
            namev.SetString(name.c_str(),d.GetAllocator());
            add_attitude(v, (Eigen::Quaterniond) trans.rotation(), d);
            add_vector(v, (Eigen::Vector3d) trans.translation(), d);

            _json.AddMember(namev, v, d.GetAllocator());
        }


        inline std::vector<std::string> get_file_list(const std::string &path) {
            std::vector<std::string> m_file_list;
            if (!path.empty()) {
                namespace fs = boost::filesystem;

                fs::path apk_path(path);
                fs::directory_iterator end;

                for (fs::directory_iterator i(apk_path); i != end; ++i) {
                    const fs::path cp = (*i);
                    m_file_list.push_back(cp.string());
                }
            }
            return m_file_list;
        }

        inline std::vector<std::string> get_filename_list(const std::string &path) {
            std::vector<std::string> m_file_list;
            if (!path.empty()) {
                namespace fs = boost::filesystem;

                fs::path apk_path(path);
                fs::directory_iterator end;

                for (fs::directory_iterator i(apk_path); i != end; ++i) {
                    const fs::path cp = (*i);
                    m_file_list.push_back(cp.filename().string());
                }
            }
            return m_file_list;
        }

        inline std::string get_string_from_file(const std::string & path)
        {
            std::ifstream ifs(path);
            std::string content((std::istreambuf_iterator<char>(ifs)),
                                (std::istreambuf_iterator<char>()));
            return content;
        }

        inline const char* json_to_buffer(rapidjson::Document & d, int32_t & Count)
        {
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
            d.Accept(writer);
            Count = buffer.GetSize();
            const char * str = buffer.GetString();
            char * res = new char[Count];
            memcpy(res,str,sizeof(char)*Count);
            return res;
        }

        inline std::string json_to_string(rapidjson::Document & d)
        {
            int32_t size;
            const char * str = json_to_buffer(d,size);
            return std::string(str);
        }

    }
};
#endif //RAPIDFDM_UTILS_H_H
