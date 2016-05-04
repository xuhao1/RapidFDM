#ifndef __RAPIDFDM_AERODYNAMIC_NODE_H__
#define __RAPIDFDM_AERODYNAMIC_NODE_H__
#include <nodes/Node.h>
#include <FlyingData.h>
#include <rapidjson/document.h>
#include <geometrys/geometry_helper.h>

namespace RapidFDM {
    namespace Aerodynamics {
        //! This node stand for the base of a aircraft
        class AircraftNode : public Node {

            AircraftNode(rapidjson::Document &document) :
                    Node(document) {
                if (document.HasMember("geometry") && document["geometry"].IsObject())
                    this->geometry = geometry_helper::create_geometry_from_json(document["geometry"]);
            }
        };
    }
}


#endif