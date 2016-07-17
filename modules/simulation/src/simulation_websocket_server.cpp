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
    SimulatorAircraft *simulatorAircraft;
    SimulatorWorld simulatorWorld;
    ws_json_channel_handler *handler_realtime_output = nullptr;

    boost::asio::io_service io_service;
    boost::posix_time::seconds interval;
    boost::asio::deadline_timer *timer = nullptr;
    float tick_time;
public:
    simulation_websocket_server(int port, std::string aircraft_path, float deltatime = 0.005, float tick_time = 0.02) :
            websocket_server(port), simulatorWorld(deltatime), interval(tick_time)
    {

        parser parser1(aircraft_path);
        aircraftNode = parser1.get_aircraft_node();
        assert(aircraftNode != nullptr);

        PxTransform init_trans = PxTransform::createIdentity();
        init_trans.p.z = 100;
        init_trans.q = PxQuat(5 * M_PI / 180,PxVec3(1,0,0));
        ControlSystem::BaseController *baseController = new ControlSystem::BaseController(aircraftNode);
        simulatorAircraft = simulatorWorld.create_aircraft(aircraftNode, baseController,init_trans,30);

        handler_realtime_output = new ws_json_channel_handler((websocket_server *) this, "output");

        timer = new boost::asio::deadline_timer(io_service, interval);
        this->tick_time = tick_time;
        aircraftNode->set_internal_state("main_engine_0/n",220);

    }

    void run_next_tick()
    {
        timer->async_wait([&](const boost::system::error_code &) {
            this->tick(tick_time);
        });
    }

    void timer_thread()
    {
        run_next_tick();
        io_service.run();
    }

    void output()
    {
        rapidjson::Document d;
        d.SetObject();

        add_transform(d, aircraftNode->get_ground_transform(), d);

        rapidjson::Value value(rapidjson::kObjectType);
        auto trans_body_2_world = aircraftNode->get_ground_transform().linear();
        add_vector(value, trans_body_2_world * aircraftNode->get_total_force(), d, "total_force");
        add_vector(value, trans_body_2_world * aircraftNode->get_total_torque(), d, "total_torque");

        add_vector(value, trans_body_2_world * aircraftNode->get_total_aerodynamics_force(), d, "total_airdynamics_force");
        add_vector(value, trans_body_2_world * aircraftNode->get_total_aerodynamics_torque(), d, "total_airdynamics_torque");

        add_vector(value, trans_body_2_world * aircraftNode->get_total_engine_torque(), d, "total_engine_torque");
        add_vector(value, trans_body_2_world * aircraftNode->get_total_engine_force(), d, "total_engine_force");

        d.AddMember("forces_torques", value, d.GetAllocator());

        rapidjson::Value airspeed_value(rapidjson::kObjectType);
        ComponentData data = aircraftNode->get_component_data();
        AirState airState;
        add_value(airspeed_value,data.get_airspeed_mag(airState),d,"airspeed");
        add_value(airspeed_value,data.get_angle_of_attack(airState),d,"angle_of_attack");
        add_value(airspeed_value,data.get_sideslip(airState),d,"sideslip");
        d.AddMember("airstate",airspeed_value,d.GetAllocator());

        handler_realtime_output->send_json(d);
    }

    void tick(float ticktime)
    {

        timer->expires_at(timer->expires_at() + interval);
        simulatorWorld.Step(ticktime);
        output();
        run_next_tick();
    }

};

int main(int argc, char **argv)
{
    std::string path = "/Users/xuhao/Develop/FixedwingProj/RapidFDM/sample_data/aircrafts/sample_aircraft";
    if (argc > 1) {
        path = std::string(argv[1]);
    }

    simulation_websocket_server server(9093, path);

    new std::thread([&] {
        printf("run server thread\n");
        server.main_thread();
    });
    printf("run simulation\n");
    server.timer_thread();

}