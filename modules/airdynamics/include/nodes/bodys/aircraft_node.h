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
            AircraftNode(rapidjson::Document &document) :
                    Node(document) {
                if (document.HasMember("geometry") && document["geometry"].IsObject())
                    this->geometry = geometry_helper::create_geometry_from_json(document["geometry"]);
                this->type = "aircraft_name";
            }

            virtual void brief() override {
                ((Node * )
                this)->brief();
                this->geometry->brief();
            }
        };
    }
}


#endif