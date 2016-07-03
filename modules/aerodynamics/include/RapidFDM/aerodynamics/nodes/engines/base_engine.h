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
            virtual Eigen::Vector3d get_engine_force(ComponentData data){
                abort();
                return Eigen::Vector3d(0,0,0);
            }
            virtual Eigen::Vector3d get_engine_torque(ComponentData data){
                abort();
                return Eigen::Vector3d(0,0,0);
            }

            BaseEngineNode(const rapidjson::Value &_json, Joint *_parent = nullptr)
                    :Node(_json,_parent)
            {
                this->type_str = "engine_node";
            }

            virtual Eigen::Vector3d get_realtime_force (ComponentData data) override
            {
                return get_engine_force(data);
            }
            virtual Eigen::Vector3d get_realtime_torque (ComponentData data) override
            {
                return get_engine_torque(data);
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
                return node;
            }

        };
    }
}
#endif //RAPIDFDM_BASE_ENGINE_H
