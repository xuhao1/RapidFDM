#include <nodes/Node.h>
#include <FlyingData.h>
#include <rapidjson/document.h>

namespace RapidFDM {
    namespace Aerodynamics {
        //! This node stand for the base of a aircraft
        class AircraftNode : public Node {

            AircraftNode(rapidjson::Document &document) :
                    Node(document) {

            }
        };
    }
}