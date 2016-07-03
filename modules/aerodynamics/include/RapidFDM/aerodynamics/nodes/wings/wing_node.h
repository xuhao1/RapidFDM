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
            void init(const rapidjson::Value & v)
            {
                this->geometry = new WingGeometry(v);
                this->type_str = "wing";
                enableControl = getWing()->params.enableControl;
            }
            WingNode(const rapidjson::Value &v, Joint *_parent = nullptr) :
                    Node(v, _parent) {
                init(v);
            }

        };
    }
}

#endif