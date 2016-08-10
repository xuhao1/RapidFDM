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
        public:
            simulation_dji_a3_adapter(AircraftNode * aircraft);
            
            void tick();
            
            
            void on_message_root(client *c, websocketpp::connection_hdl hdl, message_ptr msg);
            
            void on_message_simulator(client *c, websocketpp::connection_hdl hdl, message_ptr msg);
            
            void on_simulator_link_open(client *c,websocketpp::connection_hdl hdl);
            
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
