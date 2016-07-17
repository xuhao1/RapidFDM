//
// Created by Hao Xu on 16/7/16.
//


#include <RapidFDM/simulation/simulator_world.h>
#include <RapidFDM/network_protocol/websocket_server.h>
#include <RapidFDM/network_protocol/ws_channel_handler.h>
#include <RapidFDM/aerodynamics/airdynamics_parser.h>
#include <RapidFDM/control_system/control_system.h>
#include <RapidFDM/utils.h>
#include <thread>
#include <boost/asio.hpp>

using namespace RapidFDM::NetworkProtocol;
using namespace RapidFDM::Simulation;
using namespace RapidFDM::Utils;
using namespace RapidFDM;

class simulation_websocket_server : public websocket_server
{
protected:
    AircraftNode *aircraftNode = nullptr;
    SimulatorAircraft * simulatorAircraft;
    SimulatorWorld simulatorWorld;
    ws_json_channel_handler *handler_realtime_output = nullptr;

    boost::asio::io_service io_service;
    boost::posix_time::seconds interval;
    boost::asio::deadline_timer * timer = nullptr;
    float tick_time;
public:
    simulation_websocket_server(int port, std::string aircraft_path, float deltatime = 0.005,float tick_time = 0.02) :
            websocket_server(port), simulatorWorld(deltatime),interval(tick_time)
    {

        parser parser1(aircraft_path);
        aircraftNode = parser1.get_aircraft_node();
        assert(aircraftNode != nullptr);

        ControlSystem::BaseController * baseController = new ControlSystem::BaseController(aircraftNode);
        simulatorAircraft = simulatorWorld.create_aircraft(aircraftNode,baseController);

        handler_realtime_output = new ws_json_channel_handler((websocket_server *) this, "output");

        timer = new boost::asio::deadline_timer(io_service,interval);
        this->tick_time = tick_time;

    }
    void run_next_tick()
    {
        timer->async_wait([&](const boost::system::error_code& ){
            this->tick(tick_time);
        });
    }

    void timer_thread()
    {
        run_next_tick();
        io_service.run();
    }

    void tick(float ticktime)
    {

        timer->expires_at(timer->expires_at() + interval);

        simulatorWorld.Step(ticktime);
        rapidjson::Document d;
        d.SetObject();

        add_transform(d,aircraftNode->get_ground_transform(),d);
        handler_realtime_output->send_json(d);

        run_next_tick();
    }

};

int main(int argc,char ** argv)
{
    std::string path = "/Users/xuhao/Develop/FixedwingProj/RapidFDM/sample_data/aircrafts/sample_aircraft";
    if (argc > 1)
    {
        path = std::string(argv[1]);
    }

    simulation_websocket_server server(9093,path);

    new std::thread([&] {
        printf("run server thread\n");
        server.main_thread();
    });
    printf("run simulation\n");
    server.timer_thread();

}