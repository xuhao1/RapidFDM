//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_UTILS_H_H
#define RAPIDFDM_UTILS_H_H

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <Eigen/Eigen>

namespace RapidFDM {
    namespace Utils {

        double fast_value(rapidjson::Value &_json, std::string key) {
            if (_json.HasMember(key.c_str())) {
                if (_json[key.c_str()].IsInt())
                    return _json[key.c_str()].GetInt();
                else if (_json[key.c_str()].IsDouble())
                    return _json[key.c_str()].GetDouble();
            }
            return 0;
        }

        double fast_value(rapidjson::Value &_json, int index) {
            if (_json.IsArray() && _json.Size() > index) {
                if (_json[index].IsInt())
                    return _json[index].GetInt();
                else if (_json[index].IsDouble())
                    return _json[index].GetDouble();
            }
            return 0;
        }

        Eigen::Vector3d fast_vector3(rapidjson::Value &_json, std::string key,
                                     Eigen::Vector3d optional_value = Eigen::Vector3d(0, 0, 0)) {
            if (_json.HasMember(key.c_str()) && _json[key.c_str()].IsArray()) {
                rapidjson::Value &array = _json[key.c_str()];
                return Eigen::Vector3d(fast_value(array, 0), fast_value(array, 1), fast_value(array, 2));
            }
            return optional_value;
        }

        Eigen::Quaterniond fast_quaternion(rapidjson::Value &_json, std::string key) {
            if (_json.HasMember(key.c_str()) && _json[key.c_str()].IsArray()) {
                rapidjson::Value &array = _json[key.c_str()];
                return Eigen::Quaterniond(fast_value(array, 0),
                                          fast_value(array, 1),
                                          fast_value(array, 2),
                                          fast_value(array, 3));
            }
            return Eigen::Quaterniond(1, 0, 0, 0);
        }

        std::string fast_string(rapidjson::Value &_json, std::string key) {
            if (_json.HasMember(key.c_str()) && _json[key.c_str()].IsString()) {
                return _json[key.c_str()].GetString();
            }

        Eigen::Quaterniond fast_attitude(rapidjson::Value &_json, std::string key) {

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
        }

        void add_attitude(rapidjson::Value &_json, Eigen::Quaterniond trans, rapidjson::Document &d,
                          std::string name = "attitude") {
            rapidjson::Value v(rapidjson::kArrayType);
            rapidjson::Value namev(rapidjson::kStringType);
            namev.SetString(rapidjson::StringRef(name.c_str()));
            v.PushBack(trans.w(), d.GetAllocator());
            v.PushBack(trans.x(), d.GetAllocator());
            v.PushBack(trans.y(), d.GetAllocator());
            v.PushBack(trans.z(), d.GetAllocator());
            _json.AddMember(namev, v, d.GetAllocator());
        }

        void add_vector(rapidjson::Value &_json, Eigen::Vector3d vec, rapidjson::Document &d,
                        std::string name = "vector") {
            rapidjson::Value v(rapidjson::kArrayType);
            rapidjson::Value namev(rapidjson::kStringType);
            namev.SetString(rapidjson::StringRef(name.c_str()));
            v.PushBack(vec.x(), d.GetAllocator());
            v.PushBack(vec.y(), d.GetAllocator());
            v.PushBack(vec.z(), d.GetAllocator());
            _json.AddMember(namev, v, d.GetAllocator());
        }

        void add_transform(rapidjson::Value &_json, Eigen::Affine3d trans, rapidjson::Document &d,
                           std::string name = "transform") {
            rapidjson::Value v(rapidjson::kObjectType);
            rapidjson::Value namev(rapidjson::kStringType);
            namev.SetString(rapidjson::StringRef(name.c_str()));
            add_attitude(v, (Eigen::Quaterniond) trans.rotation(), d);
            add_vector(v, (Eigen::Vector3d) trans.translation(), d);
            _json.AddMember(namev, v, d.GetAllocator());
        }
    }
    };
#endif //RAPIDFDM_UTILS_H_H
