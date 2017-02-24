#ifndef RAPIDFDM_AERODYNAMICS_NODES_WING_H
#define RAPIDFDM_AERODYNAMICS_NODES_WING_H

#include <RapidFDM/aerodynamics/nodes/base_node.h>
#include <RapidFDM/aerodynamics/geometrys/wing_geometrys.h>
#include <RapidFDM/aerodynamics/joints/base_joint.h>
#include <RapidFDM/aerodynamics/geometrys/Geometrys.h>

namespace RapidFDM {
    namespace Aerodynamics {
        class WingNode : public BaseNode {
        protected:

            WingGeometry *getWing() {
                return dynamic_cast<WingGeometry *>(this->geometry);
            }

        public:
            bool enableControl = false;
            void init(const rapidjson::Value & v)
            {
                this->geometry = new WingGeometry(v);
                this->geometry->_parent = this;
                this->node_type = AerodynamicsNodeType ::AerodynamicsWingNode;
                enableControl = getWing()->params.enableControl;
                if (enableControl)
                {
                    if (getWing()->params.wingPart == 2)
                    {
                        this->internal_states[getUniqueID()+"/flap_0"] = 0;
                        this->internal_states[getUniqueID() + "/flap_1"] = 0;
                        this->control_axis[getUniqueID() + "/flap_0"] = 0;
                        this->control_axis[getUniqueID() + "/flap_1"] = 0;
                    }
                    else
                    {
                        this->internal_states[getUniqueID() + "/flap"] = 0;
                        this->control_axis[getUniqueID() + "/flap"] = 0;
                    }
                }
                this->freq_cut = fast_value(v,"freq_cut",3);
            }
            WingNode(const rapidjson::Value &v, BaseJoint *_parent = nullptr) :
                    BaseNode(v, _parent) {
                init(v);
            }

        };
    }
}

#endif