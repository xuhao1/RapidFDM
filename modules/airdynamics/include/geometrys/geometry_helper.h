//
// Created by xuhao on 2016/5/4.
//

#ifndef RAPIDFDM_GEOMETRY_HELPER_H
#define RAPIDFDM_GEOMETRY_HELPER_H

#include <rapidjson/rapidjson.h>
#include "geometrys/base_geometry.h"
#include "geometrys/standard_geometrys.h"
#include <utils.h>

using namespace RapidFDM::Utils;

namespace RapidFDM {
    namespace Aerodynamics {
        class GeometryHelper {
        public:
            static BaseGeometry *create_geometry_from_json(rapidjson::Value &v) {
                std::string type = fast_string(v, "type");
                if (type == "box") {
                    return new BoxGeometry(v);
                }
                return new BaseGeometry;
            }

        };
    }
};
#endif //RAPIDFDM_GEOMETRY_HELPER_H
