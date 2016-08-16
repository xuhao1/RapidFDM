#ifndef RAPIDFDM_AERODYNAMICS_NODES_WING_H
#define RAPIDFDM_AERODYNAMICS_NODES_WING_H

#include <RapidFDM/aerodynamics/nodes/base_node.h>
#include <RapidFDM/aerodynamics/geometrys/wing_geometrys.h>
#include <RapidFDM/aerodynamics/joints/base_joint.h>
#include <RapidFDM/aerodynamics/geometrys/Geometrys.h>
#define MIXER_RATIO 0.0

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
                        this->internal_states["flap_0"] = 0;
                        this->internal_states["flap_1"] = 0;
                        this->control_axis["flap_0"] = 0;
                        this->control_axis["flap_1"] = 0;
                        
                    }
                    else
                    {
                        this->internal_states["flap"] = 0;
                        this->control_axis["flap"] = 0;
                    }
                }
            }
            WingNode(const rapidjson::Value &v, BaseJoint *_parent = nullptr) :
                    BaseNode(v, _parent) {
                init(v);
            }
            virtual void iter_internal_state(double deltatime) override
            {
                if (getWing()->params.wingPart == 2)
                {
                    this->internal_states["flap_0"] = this->control_axis["flap_0"] * (1-MIXER_RATIO) + MIXER_RATIO *this->internal_states["flap_0"];
                    this->internal_states["flap_1"] = this->control_axis["flap_1"] * (1-MIXER_RATIO) + MIXER_RATIO *this->internal_states["flap_0"];
                }
                else
                {
                    this->internal_states["flap"] = this->control_axis["flap"] * (1-MIXER_RATIO) + MIXER_RATIO *this->internal_states["flap_0"];
                }
            }


        };
    }
}

#endif