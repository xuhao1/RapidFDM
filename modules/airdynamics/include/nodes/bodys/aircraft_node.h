#ifndef __RAPIDFDM_AERODYNAMIC_NODE_H__
#define __RAPIDFDM_AERODYNAMIC_NODE_H__
#include <nodes/Node.h>
#include <FlyingData.h>
#include <rapidjson/document.h>
#include <geometrys/geometry_helper.h>
#include <stdio.h>

namespace RapidFDM {
    namespace Aerodynamics {
        //! This node stand for the base of a aircraft
        class AircraftNode : public Node {
        public:
            AircraftNode(rapidjson::Value &document) :
                    Node(document) {
                if (document.HasMember("geometry") && document["geometry"].IsObject())
                    this->geometry = GeometryHelper::create_geometry_from_json(document["geometry"]);
                this->type = "aircraft_node";
                printf("Success parse aircraft_node\n");
                printf("Name %s type: %s geometry %s\n", this->name.c_str(), this->type.c_str(),
                       geometry->get_type().c_str());

            }

            virtual void brief() override {
                Node::brief();
                this->geometry->brief();
            }
        };
    }
}


#endif