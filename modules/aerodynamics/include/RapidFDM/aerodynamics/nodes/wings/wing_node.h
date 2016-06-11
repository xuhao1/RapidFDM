#ifndef RAPIDFDM_AERODYNAMICS_NODES_WING_H
#define RAPIDFDM_AERODYNAMICS_NODES_WING_H

#include <RapidFDM/aerodynamics/nodes/Node.h>
#include <RapidFDM/aerodynamics/geometrys/wing_geometrys.h>
#include <RapidFDM/aerodynamics/joints/Joint.h>

namespace RapidFDM {
    namespace Aerodynamics {
        class WingNode : public Node {
        protected:
            bool enableControl = false;

            WingGeometry *getWing() {
                return static_cast<WingGeometry *>(this->geometry);
            }

        public:
            WingNode(rapidjson::Value &v, rapidjson::Document &d, Joint *_parent = nullptr) :
                    Node(v, d, _parent) {
                this->geometry = new WingGeometry(v);
                this->type = "wing";
                enableControl = getWing()->params.enableControl;
            }

            WingNode(rapidjson::Document &document, Joint *_parent = nullptr) {
                *this = WingNode(document, document, _parent);
            }
        };
    }
}

#endif