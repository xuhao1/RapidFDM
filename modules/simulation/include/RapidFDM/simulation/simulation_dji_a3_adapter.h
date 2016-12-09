//
// Created by Hao Xu on 16/8/11.
//

#ifndef RAPIDFDM_SIMULATION_DJI_A3_ADAPTER_H
#define RAPIDFDM_SIMULATION_DJI_A3_ADAPTER_H

#include <printf.h>
#include <string>
#include <RapidFDM/simulation/simulator_world.h>
#include <RapidFDM/simulation/utils.h>
#include <RapidFDM/aerodynamics/airdynamics_parser.h>
#include <RapidFDM/control_system/control_system.h>
#include <RapidFDM/utils.h>
#include <thread>
#include <boost/asio.hpp>
#include <mutex>
#include <cstdlib>
#include <csetjmp>
#include <csignal>

//using namespace RapidFDM::NetworkProtocol;
using namespace RapidFDM::Simulation;
using namespace RapidFDM::Simulation::Utils;
using namespace RapidFDM::Utils;
using namespace RapidFDM;

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <functional>

#include <iostream>

typedef websocketpp::client<websocketpp::config::asio_client> client;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

// pull out the type of messages sent by our config
typedef websocketpp::config::asio_client::message_type::ptr message_ptr;


namespace RapidFDM
{
    namespace Simulation
    {
        class simulation_dji_a3_adapter
        {
        protected:
            client c_root;
            client * c_simulator = nullptr;
            websocketpp::lib::asio::io_service  realtime_calc_io_service;
            boost::asio::deadline_timer *timer = nullptr;
    
            AircraftNode * aircraft = nullptr;
            boost::posix_time::milliseconds interval;
            websocketpp::connection_hdl sim_connection_hdl;
            float pwm[8] = {0};
            float RcA = 0, RcE = 0,RcR= 0,RcT = 0;
            int no_data_count = 0;
            bool data_update = false;
            websocketpp::connection_hdl sim_hdl;
            std::string file_name;
            std::string root_uri;
            std::string sim_uri;
            int dcount = 0;
            
            float intial_lati = 0;
            float intial_lon = 0;
        public:
            std::function<void(void)> * on_receive_pwm = nullptr;
            long simulator_tick = 0;
            bool motor_starter = false;
            bool sim_online = false;
            bool assiant_online = false;
            SimulatorAircraft * sim_air = nullptr;
            simulation_dji_a3_adapter(AircraftNode * aircraft);
            
            void tick();
            
            
            void on_message_root(client *c, websocketpp::connection_hdl hdl, message_ptr msg);
            
            void on_message_simulator(client *c, websocketpp::connection_hdl hdl, message_ptr msg);
            
            void on_simulator_link_open(client *c,websocketpp::connection_hdl hdl);
            void on_simulator_link_failed(client *c,websocketpp::connection_hdl hdl);
            
            void on_assitant_failed(client *c,websocketpp::connection_hdl hdl);
            void on_assitant_open(client *c,websocketpp::connection_hdl hdl);
            
            void send_realtime_data();
            
            void check_assiant_online();
            
            void reconnect_simulator();
            
            void add_values(rapidjson::Document & d);
            
            void main_thread()
            {
                new std::thread([&]{
                    c_root.run();
                });
            }
            
            
        };
    }
}
#endif //RAPIDFDM_SIMULATION_DJI_A3_ADAPTER_H
