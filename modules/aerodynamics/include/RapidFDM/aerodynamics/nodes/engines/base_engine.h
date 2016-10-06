//
// Created by Hao Xu on 16/6/30.
//

#ifndef RAPIDFDM_BASE_ENGINE_H
#define RAPIDFDM_BASE_ENGINE_H

#include <RapidFDM/aerodynamics/nodes/base_node.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        class  BaseEngineNode : public BaseNode
        {
        public:
            virtual Eigen::Vector3d get_engine_force(ComponentData data,AirState airState) const{
                abort();
                return Eigen::Vector3d(0,0,0);
            }
            virtual Eigen::Vector3d get_engine_torque(ComponentData data,AirState airState) const {
                abort();
                return Eigen::Vector3d(0,0,0);
            }

            BaseEngineNode(const rapidjson::Value &_json, BaseJoint *_parent = nullptr)
                    :BaseNode(_json,_parent)
            {
                this->node_type = AerodynamicsNodeType ::AerodynamicsBaseEngineNode;
            }

            virtual Eigen::Vector3d get_realtime_force (ComponentData data,AirState airState) const override
            {
                return get_engine_force(data,airState);
            }
            virtual Eigen::Vector3d get_realtime_torque (ComponentData data,AirState airState) const override
            {
                return get_engine_torque(data,airState);
            }

            BaseEngineNode()
            {
                this->node_type = AerodynamicsNodeType ::AerodynamicsBaseEngineNode;
            }

            virtual BaseNode * instance() override
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
