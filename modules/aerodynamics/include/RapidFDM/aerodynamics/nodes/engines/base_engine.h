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
            virtual Eigen::Vector3d get_engine_force() = 0;
            virtual Eigen::Vector3d get_engine_torque() = 0;
            BaseEngineNode(rapidjson::Document & document, Joint *_parent = nullptr):
                    Node(document,_parent)
            {

            }

            BaseEngineNode(rapidjson::Value &_json, rapidjson::Document &document, Joint *_parent = nullptr)
                    :Node(_json,document,_parent)
            {

            }

            virtual Eigen::Vector3d get_realtime_force () override
            {
                return get_engine_force();
            }
            virtual Eigen::Vector3d get_realtime_torque () override
            {
                return get_engine_torque();
            }

        };
    }
}
#endif //RAPIDFDM_BASE_ENGINE_H
