#ifndef RAPIDFDM_AERODYNAMICS_NODES_WING_H
#define RAPIDFDM_AERODYNAMICS_NODES_WING_H

#include <RapidFDM/aerodynamics/nodes/base_node.h>
#include <RapidFDM/aerodynamics/geometrys/wing_geometrys.h>
#include <RapidFDM/aerodynamics/joints/base_joint.h>

namespace RapidFDM {
    namespace Aerodynamics {
        class WingNode : public BaseNode {
        protected:
            bool enableControl = false;

            WingGeometry *getWing() {
                return dynamic_cast<WingGeometry *>(this->geometry);
            }

        public:
            void init(const rapidjson::Value & v)
            {
                this->geometry = new WingGeometry(v);
                this->node_type = AerodynamicsNodeType ::AerodynamicsWingNode;
                enableControl = getWing()->params.enableControl;
            }
            WingNode(const rapidjson::Value &v, BaseJoint *_parent = nullptr) :
                    BaseNode(v, _parent) {
                init(v);
            }

        };
    }
}

#endif