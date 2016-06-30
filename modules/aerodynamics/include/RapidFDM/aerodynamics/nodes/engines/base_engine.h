//
// Created by Hao Xu on 16/6/30.
//

#ifndef RAPIDFDM_BASE_ENGINE_H
#define RAPIDFDM_BASE_ENGINE_H

#include <RapidFDM/aerodynamics/nodes/Node.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        class  BaseEngineNode : public Node
        {
        public:
            virtual Eigen::Vector3d get_engine_force(){
                abort();
                return Eigen::Vector3d(0,0,0);
            }
            virtual Eigen::Vector3d get_engine_torque(){
                abort();
                return Eigen::Vector3d(0,0,0);
            }
            BaseEngineNode(rapidjson::Document & document, Joint *_parent = nullptr):
                    Node(document,_parent)
            {
                this->type_str = "engine_node";
            }

            BaseEngineNode(rapidjson::Value &_json, rapidjson::Document &document, Joint *_parent = nullptr)
                    :Node(_json,document,_parent)
            {
                this->type_str = "engine_node";
            }

            virtual Eigen::Vector3d get_realtime_force () override
            {
                return get_engine_force();
            }
            virtual Eigen::Vector3d get_realtime_torque () override
            {
                return get_engine_torque();
            }

            BaseEngineNode()
            {
                this->type_str = "engine_node";
            }

            virtual Node * instance() override
            {
                BaseEngineNode *node = new BaseEngineNode;
                memcpy(node, this, sizeof(BaseEngineNode));
                node->geometry = this->geometry->instance();
            }

        };
    }
}
#endif //RAPIDFDM_BASE_ENGINE_H
