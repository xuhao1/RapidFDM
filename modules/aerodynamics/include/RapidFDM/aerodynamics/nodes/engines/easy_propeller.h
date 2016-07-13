//
// Created by Hao Xu on 16/6/27.
//

#ifndef RAPIDFDM_EASY_PROPELLER_H
#define RAPIDFDM_EASY_PROPELLER_H

#include <RapidFDM/aerodynamics/nodes/engines/base_engine.h>
//TODO:
// rho
#define air_rho 1.29

namespace RapidFDM
{
    namespace Aerodynamics
    {
        //! This node stand for a electrical_propeller, no need battery and no acceleration time
        //  this propeller perfermance spline is independce beyond D and H
        class EasyPropellerNode : public BaseEngineNode
        {
        protected:
            //Ct = A_ct * J + B_ct;
            //T = rho * n^2 * D^4 * Ct
            //Cq = 0.05 / 2pi
            //Q = rho * n^2 * D^5 * Cq
            double A_ct = -0.15;
            double B_ct = 0.09;
            double C_q = 0.007961783439;
            double D;
            double max_n;
            //!Direction = 1 means 顺时针 产生负力矩
            int direction = 1;

            virtual float get_cq(ComponentData data)
            {
                return C_q;
            }

            virtual float get_ct(ComponentData data)
            {
                double wind_speed = get_air_velocity(data).x();
                double n = internal_states["n"];
                double J = wind_speed / n / D;
                if (wind_speed < 0.1 || n < 1)
                    J = 0;
                return A_ct * J + B_ct;
            }

            virtual float get_propeller_force(ComponentData data)
            {
                double n = internal_states["n"];
                return get_ct(data) * air_rho * n * n * pow(D, 4);
            }

            virtual float get_propeller_torque(ComponentData data)
            {
                double n = internal_states["n"];
                return -get_cq(data) * air_rho * n * n * pow(D, 5) * direction;
            }

        public:

            virtual void iter_internal_state(double deltatime) override
            {
                internal_states["n"] = max_n * control_axis["thrust"];
            }

            virtual Eigen::Vector3d get_engine_force(ComponentData data) override
            {
                //Force to negative x axis
                return Eigen::Vector3d(-get_propeller_force(data), 0, 0);
            }

            virtual Eigen::Vector3d get_engine_torque(ComponentData data) override
            {
                //Torque at negative x axis
                return Eigen::Vector3d(-get_propeller_torque(data), 0, 0);
            }

            virtual Node *instance() override
            {
                BaseEngineNode *node = new BaseEngineNode;
                memcpy(node, this, sizeof(BaseEngineNode));
                node->geometry = this->geometry->instance();
                return node;
            }


            EasyPropellerNode(const rapidjson::Value &document) :
                    BaseEngineNode(document)
            {
                if (document.HasMember("geometry") && document["geometry"].IsObject()) {
                    this->geometry = GeometryHelper::create_geometry_from_json(document["geometry"]);
                }
                else {
                    this->geometry = new BaseGeometry();
                }
                D = fast_value(document, "D", 0.254);
                direction = fast_value(document, "direction", 1);
                max_n = fast_value(document, "max_rpm", 10000.0) / 60.0;
                this->type_str = "propeller_node";
                printf("Success parse propeller_node\n");
                printf("Name %s type: %s geometry %s\n", this->name.c_str(), this->type_str.c_str(),
                       geometry->get_type().c_str());

                this->internal_states["n"] = 0;
                this->control_axis["thrust"] = 0;

            }
        };
    }
}
#endif //RAPIDFDM_ELECTRICAL_PROPELLER_H
