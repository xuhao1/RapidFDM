#ifndef __RAPIDFDM_AERODYNAMIC_NODE_H__
#define __RAPIDFDM_AERODYNAMIC_NODE_H__
#include <RapidFDM/aerodynamics/nodes/Node.h>
#include <RapidFDM/aerodynamics/FlyingData.h>
#include <rapidjson/document.h>
#include <RapidFDM/aerodynamics/geometrys/geometry_helper.h>
#include <stdio.h>

namespace RapidFDM {
    namespace Aerodynamics {
        //! This node stand for the base of a aircraft
        class AircraftNode : public Node {
        public:
            AircraftNode(rapidjson::Value &_json, rapidjson::Document &document) :
                    Node(_json, document) {
                assert(_json.IsObject());
                if (_json.HasMember("geometry")) {
                    if (_json["geometry"].IsObject())
                        this->geometry = GeometryHelper::create_geometry_from_json(_json["geometry"]);
                }
                this->type = "aircraft_node";
                printf("Success parse aircraft_node\n");
                printf("Name %s type: %s geometry %s\n", this->name.c_str(), this->type.c_str(),
                       geometry->get_type().c_str());

            }

            AircraftNode(rapidjson::Document &document) :
                    Node(document) {
                if (document.HasMember("geometry")) {
                    if (document["geometry"].IsObject())
                        this->geometry = GeometryHelper::create_geometry_from_json(document["geometry"]);
                }
                this->type = "aircraft_node";
                printf("Success parse aircraft_node\n");
                printf("Name %s type: %s geometry %s\n", this->name.c_str(), this->type.c_str(),
                       geometry->get_type().c_str());

            }
        };
    }
}


#endif