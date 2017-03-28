//
// Created by Hao Xu on 16/6/11.
//

#ifndef RAPIDFDM_BASE_CONTROLLER_H
#define RAPIDFDM_BASE_CONTROLLER_H

#include <RapidFDM/aerodynamics/aerodynamics.h>

namespace RapidFDM
{
    namespace ControlSystem
    {
        class BaseController{
        public:

            double roll_sp = 0;
            double pitch_sp = 0;
            double throttle_sp = 0;
            double yaw_sp = 0;

            float pwm[8] ={0};

            Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
            Eigen::Vector3d angular_rate = Eigen::Vector3d(0,0,0);

            Aerodynamics::AircraftNode * aircraftNode;
            BaseController(Aerodynamics::AircraftNode * _aircraftNode)
            {
                this->aircraftNode = _aircraftNode;
            }
            virtual void control_step(float deltatime)
            {

            }
        };
    }
}

#endif //RAPIDFDM_BASE_CONTROLLER_H
