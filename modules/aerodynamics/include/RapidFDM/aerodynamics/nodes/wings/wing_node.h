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
            WingNode(const rapidjson::Value &v, rapidjson::Document &d, Joint *_parent = nullptr) :
                    Node(v, d, _parent) {
                init(v);
            }

            WingNode(rapidjson::Document &document, Joint *_parent = nullptr):
                    Node(document,_parent)
            {
                init(document);
            }
        };
    }
}

#endif