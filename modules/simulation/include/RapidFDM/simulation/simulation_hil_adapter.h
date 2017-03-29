//
// Created by xuhao on 2016/12/14.
//

#ifndef RAPIDFDM_SIMULATION_HIL_ADAPTER_H
#define RAPIDFDM_SIMULATION_HIL_ADAPTER_H

#include "simulator_aircraft.h"
#include <boost/asio.hpp>
namespace RapidFDM
{
    namespace Simulation{
        enum HIL_TYPE
        {
            HIL_NONE,
            HIL_DJI,
            HIL_PIXHAWK
        };
        class simulation_hil_adapter{
        private:
            void tick();
            boost::posix_time::milliseconds interval;
            boost::asio::deadline_timer *timer = nullptr;
        protected:
            boost::asio::io_service io_service;
            SimulatorAircraft * sim_air = nullptr;
            AircraftNode * aircraftNode = nullptr;

            int total_tick_count = 0;

            double intial_lati = 0;
            double intial_lon = 0;

            virtual  void tick_func(float dt,long tick) = 0;
            bool start = false;
        public:
            void set_sim_status(bool start);

            virtual Eigen::Quaterniond get_quaternion_NED();

            virtual Eigen::Vector3d get_angular_velocity_body_NED();

            virtual Eigen::Vector3d get_acc_body_NED();

            virtual Eigen::Vector3d get_lati_lon_alti();

            virtual double get_airspeed();

            virtual Eigen::Vector3d get_ground_velocity_NED();

            virtual void main_thread();
            simulation_hil_adapter(SimulatorAircraft * _sim_air, float deltatime = 5);

            virtual void on_pwm_data_receieve(float * pwm,int num);//PWM from 0-100

            virtual bool enable_simulation() = 0;


            virtual void push_json_to_app(rapidjson::Document & d) = 0;
        };
    }
}

#endif //RAPIDFDM_SIMULATION_HIL_ADAPTER_H
