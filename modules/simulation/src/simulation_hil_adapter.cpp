//
// Created by xuhao on 2016/12/14.
//

#include <RapidFDM/simulation/simulation_hil_adapter.h>
#include <thread>

#define MIXER_VTOL
//#define MIXER_GENERAL

#define C_EARTH 6378137.0f

namespace RapidFDM {
    namespace Simulation {

        simulation_hil_adapter::simulation_hil_adapter(SimulatorAircraft *_sim_air, float deltatime) :
                sim_air(_sim_air), interval(deltatime) {
            aircraftNode = _sim_air->get_aircraft_node();
            timer = new boost::asio::deadline_timer(io_service, interval);

            intial_lati = 22.5416 * M_PI / 180.0;
            intial_lon = 113.8973 * M_PI / 180.0;
        }

        void simulation_hil_adapter::tick() {
            timer->expires_at(timer->expires_at() + interval);

            total_tick_count ++;

            tick_func();

            timer->async_wait([&](const boost::system::error_code &) {
                this->tick();
            });
        }

        void simulation_hil_adapter::main_thread() {
            timer->async_wait([&](const boost::system::error_code &) {
                this->tick();
            });
            new std::thread([&] {
                io_service.run();
            });
        }

        void simulation_hil_adapter::on_pwm_data_receieve(float *pwm, int num) {

#ifdef MIXER_GENERAL
            aircraftNode->set_control_value("main_engine_0/thrust", (pwm[0] + 1) * 0.5);
            aircraftNode->set_control_value("main_engine_1/thrust", (pwm[1] + 1) * 0.5);
            aircraftNode->set_control_value("main_wing_0/flap_0",  pwm[2]);
            aircraftNode->set_control_value("main_wing_0/flap_1", - pwm[5]);
            aircraftNode->set_control_value("horizon_wing_0/flap_0", pwm[3]);
            aircraftNode->set_control_value("horizon_wing_0/flap_1", pwm[3]);
            aircraftNode->set_control_value("vertical_wing_0/flap", pwm[4]);

#endif

#ifdef MIXER_FLYINGWING
            aircraftNode->set_control_value("main_engine_0/thrust", (pwm[0] + 1) * 0.5);
            aircraftNode->set_control_value("main_wing_0/flap_0",  pwm[2]);
            aircraftNode->set_control_value("main_wing_0/flap_1", - pwm[3]);
            //aircraftNode->set_control_value("horizon_wing_0/flap_0", pwm[3]);
            //aircraftNode->set_control_value("horizon_wing_0/flap_1", pwm[3]);
            aircraftNode->set_control_value("vertical_wing_0/flap", pwm[4]);
#endif

#ifdef MIXER_VTOL
            aircraftNode->set_control_value("main_engine_0/thrust", (pwm[0] + 1) * 0.5);
            aircraftNode->set_control_value("main_engine_1/thrust", (pwm[1] + 1) * 0.5);
            aircraftNode->set_control_value("main_engine_2/thrust", (pwm[2] + 1) * 0.5);
            aircraftNode->set_control_value("main_engine_3/thrust", (pwm[3] + 1) * 0.5);
            aircraftNode->set_control_value("main_engine_5/thrust", (pwm[4] + 1) * 0.5);
            aircraftNode->set_control_value("main_wing_0/flap_0", +pwm[5]);
            aircraftNode->set_control_value("main_wing_0/flap_1", -pwm[6]);
#endif

        }

        Eigen::Quaterniond simulation_hil_adapter::get_quaternion_NED() {
            Eigen::Quaterniond convert = (Eigen::Quaterniond) Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
            return convert * (Eigen::Quaterniond) aircraftNode->get_ground_transform().rotation() * convert;
        }

        Eigen::Vector3d simulation_hil_adapter::get_angular_velocity_body_NED() {
            Eigen::Quaterniond convert = (Eigen::Quaterniond) Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
            return convert * aircraftNode->get_angular_velocity();
        }

        Eigen::Vector3d simulation_hil_adapter::get_ground_velocity_NED() {
            Eigen::Quaterniond convert = (Eigen::Quaterniond) Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
            return convert * aircraftNode->get_ground_velocity();
        }

        Eigen::Vector3d simulation_hil_adapter::get_acc_body_NED() {
            Eigen::Quaterniond convert = (Eigen::Quaterniond) Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
            Eigen::Vector3d acc = sim_air->gAcc;
            acc.z() = acc.z() - 9.81f;
            acc = convert * acc;
            return acc;
        }

        double simulation_hil_adapter::get_airspeed() {
            AirState airState = aircraftNode->airState;
            ComponentData data = aircraftNode->get_component_data();
            return data.get_airspeed_mag(airState);
        }

        Eigen::Vector3d simulation_hil_adapter::get_lati_lon_alti() {
            Eigen::Quaterniond convert = (Eigen::Quaterniond) Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
            Eigen::Vector3d pos = (Eigen::Vector3d) aircraftNode->get_ground_transform().translation();
            pos = convert * pos;
            return Eigen::Vector3d(
                    intial_lati + pos.x() / C_EARTH,
                    intial_lon + pos.y() / C_EARTH / cos(intial_lati + pos.x() / C_EARTH),
                    pos.z()
            );
        }
    }
}

