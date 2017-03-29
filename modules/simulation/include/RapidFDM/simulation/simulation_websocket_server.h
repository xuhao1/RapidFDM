//
// Created by xuhao on 2016/12/23.
//

#ifndef RAPIDFDM_SIMULATION_WEBSOCKET_SERVER_H
#define RAPIDFDM_SIMULATION_WEBSOCKET_SERVER_H

//
// Created by Hao Xu on 16/7/16.
//


#include <RapidFDM/simulation/simulator_world.h>
#include <RapidFDM/simulation/utils.h>
#include <RapidFDM/simulation/simulation_dji_a3_adapter.h>
#include <RapidFDM/network_protocol/websocket_server.h>
#include <RapidFDM/network_protocol/ws_channel_handler.h>
#include <RapidFDM/aerodynamics/airdynamics_parser.h>
#include <RapidFDM/control_system/control_system.h>
#include <RapidFDM/utils.h>
#include <thread>
#include <boost/asio.hpp>
#include <mutex>
#include <cstdlib>
#include <csetjmp>
#include <csignal>
#include <RapidFDM/simulation/simulation_hil_adapter.h>
#include <RapidFDM/simulation/simulation_pixhawk_adapter.h>

#ifdef WIN32
#include <windows.h>
#else

#include <sys/time.h>

#endif

#include "math.h"
#include <RapidFDM/common/resource_manager.h>


long current_timestamp();

long current_timestamp_us();


namespace RapidFDM {
    namespace Simulation {
        class simulation_websocket_server : public websocket_server {
        protected:
            AircraftNode *aircraftNode = nullptr;
            SimulatorWorld simulatorWorld;
            ws_json_channel_handler *handler_realtime_output = nullptr;

            boost::posix_time::milliseconds simulation_interval;
            boost::posix_time::milliseconds output_interval;
            boost::asio::deadline_timer *simulation_timer = nullptr;
            boost::asio::deadline_timer *output_timer = nullptr;
            float tick_time;
            websocketpp::lib::asio::io_service realtime_calc_io_service;

            std::mutex phys_engine_lock;

            long runninged_tick = 0;

            void run_next_simulation_tick();

            void calc_thread();

            void output();

            void run_phys(float ticktime);


            void init_ws_json_handler();
            bool started = false;

        public:
            SimulatorAircraft *simulatorAircraft;
            simulation_hil_adapter *hil_adapter = nullptr;

            simulation_websocket_server(std::string aircraft_path, float phys_rate = 1000, float hil_rate = 200,int port = 9093 );


        };
    }
}


#endif //RAPIDFDM_SIMULATION_WEBSOCKET_SERVER_H
