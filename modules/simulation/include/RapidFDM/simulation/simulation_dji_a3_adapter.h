//
// Created by Hao Xu on 16/8/11.
//

#ifndef RAPIDFDM_SIMULATION_DJI_A3_ADAPTER_H
#define RAPIDFDM_SIMULATION_DJI_A3_ADAPTER_H

#include <iostream>
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
#include "simulation_hil_adapter.h"

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
        class simulation_dji_a3_adapter : simulation_hil_adapter
        {
        protected:
            client c_root;
            client * c_simulator = nullptr;

            websocketpp::connection_hdl sim_connection_hdl;
            float pwm[8] = {0};
            float RcA = 0, RcE = 0,RcR= 0,RcT = 0;
            std::string file_name;
            std::string root_uri;
            std::string sim_uri;
            int dcount = 0;
            
        public:
            std::function<void(void)> * on_receive_pwm = nullptr;
            long simulator_tick = 0;
            bool motor_starter = false;
            bool sim_online = false;
            bool assiant_online = false;
            simulation_dji_a3_adapter(SimulatorAircraft * _sim_air);

            virtual void tick_func() override ;
            
            void on_message_root(client *c, websocketpp::connection_hdl hdl, message_ptr msg);
            
            void on_message_simulator(client *c, websocketpp::connection_hdl hdl, message_ptr msg);
            
            void on_simulator_link_open(client *c,websocketpp::connection_hdl hdl);
            void on_simulator_link_failed(client *c,websocketpp::connection_hdl hdl);
            void on_simulator_link_closed(client *c,websocketpp::connection_hdl hdl);

            void on_assitant_failed(client *c,websocketpp::connection_hdl hdl);
            void on_assitant_open(client *c,websocketpp::connection_hdl hdl);
            
            void send_realtime_data();
            
            void try_connect_assistant();
            
            void try_connect_simulator();

            virtual void push_json_to_app(rapidjson::Document & d) override ;

            virtual bool enable_simulation() override;

            virtual void update_before_sim(long tick) override ;
            

        };
    }
}
#endif //RAPIDFDM_SIMULATION_DJI_A3_ADAPTER_H
