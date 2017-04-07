//
// Created by xuhao on 2017/3/23.
//

#include <RapidFDM/simulation/simulation_sitl_adapter.h>

namespace RapidFDM {
    namespace Simulation {

        simulation_sitl_adapter::simulation_sitl_adapter(SimulatorAircraft *simulatorAircraft,
                                                         BaseController *_controller, float dt) :
                simulation_hil_adapter(simulatorAircraft, dt) {
            this->sim_air = simulatorAircraft;
            this->controller = _controller;
        }

        bool simulation_sitl_adapter::enable_simulation() {
            return true;
        }

        void simulation_sitl_adapter::tick_func(float dt, long tick) {
            controller->angular_rate = this->get_angular_velocity_body_NED();
            controller->quat = this->get_quaternion_NED();
            controller->control_step(dt);
            memcpy(this->pwm, controller->pwm, 8 * sizeof(float));
        }

        void simulation_sitl_adapter::handle_chn_from_joystick(float *pwm, int num) {
            this->controller->roll_sp = pwm[0] * M_PI/3;
            this->controller->pitch_sp = - pwm[1] * M_PI  / 5;
            this->controller->throttle_sp = (pwm[2] + 1) / 2;
            this->controller->yaw_sp = pwm[3];
        }

        void simulation_sitl_adapter::push_json_to_app(rapidjson::Document &d) {
            rapidjson::Value sitl_value(rapidjson::kObjectType);
            add_value(sitl_value, true, d, "online");

            rapidjson::Value pwm_array(rapidjson::kArrayType);

            for (int ch = 0; ch < 8; ch++) {
                pwm_array.PushBack(pwm[ch] * 100, d.GetAllocator());
            }

            sitl_value.AddMember("PWM", pwm_array, d.GetAllocator());

            d.AddMember("sim_status", sitl_value, d.GetAllocator());

        }

    }
}
