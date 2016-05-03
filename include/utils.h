//
// Created by xuhao on 2016/5/2.
//

#ifndef RAPIDFDM_UTILS_H_H
#define RAPIDFDM_UTILS_H_H

#include <rapidjson/rapidjson.h>
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
    }
}
#endif //RAPIDFDM_UTILS_H_H
