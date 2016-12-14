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
#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif
#include "math.h"
#include <RapidFDM/common/resource_manager.h>


long long current_timestamp() {
#ifdef WIN32
	SYSTEMTIME time;
	GetSystemTime(&time);
	LONG time_ms = (time.wSecond * 1000) + time.wMilliseconds;
	return time_ms;
#else
	struct timeval te;
	gettimeofday(&te, NULL); // get current time
	long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
	 printf("milliseconds: %lld\n", milliseconds);
	return milliseconds;

#endif // WIN32
}

using namespace RapidFDM::NetworkProtocol;
using namespace RapidFDM::Simulation;
using namespace RapidFDM::Simulation::Utils;
using namespace RapidFDM::Utils;
using namespace RapidFDM;

class simulation_websocket_server : public websocket_server
{
protected:
    AircraftNode *aircraftNode = nullptr;
    SimulatorAircraft *simulatorAircraft;
    SimulatorWorld simulatorWorld;
    ws_json_channel_handler *handler_realtime_output = nullptr;

    boost::posix_time::milliseconds simulation_interval;
    boost::posix_time::milliseconds output_interval;
    boost::asio::deadline_timer *simulation_timer = nullptr;
    boost::asio::deadline_timer *output_timer = nullptr;
    float tick_time;
    bool simulator_running = false;
    websocketpp::lib::asio::io_service  realtime_calc_io_service;

    std::mutex phys_engine_lock;

    long runninged_tick = 0;
public:
    simulation_dji_a3_adapter * a3_adapter = nullptr;
    simulation_websocket_server(int port, std::string aircraft_path, float deltatime = 1, int tick_time = 2,bool use_a3 = false) :
            websocket_server(port), simulatorWorld(deltatime), simulation_interval(tick_time),output_interval(15)
    {

        parser parser1(aircraft_path);
        aircraftNode = parser1.get_aircraft_node();
        assert(aircraftNode != nullptr);

        ControlSystem::BaseController *baseController = new ControlSystem::BaseController(aircraftNode);
        simulatorAircraft = simulatorWorld.create_aircraft(aircraftNode, baseController);

        handler_realtime_output = new ws_json_channel_handler((websocket_server *) this, "output");

        simulation_timer = new boost::asio::deadline_timer(realtime_calc_io_service, simulation_interval);
        output_timer = new boost::asio::deadline_timer(*this->io_service_ptr,output_interval);

        this->tick_time = tick_time;

        handler_realtime_output->add_json_handler(
                "start", [&](const rapidjson::Value &value) {
                    this->simulator_running = true;
                    Eigen::Affine3d init_transform = fast_transform(value, "init_transform");
                    auto init_trans = transform_e2p(init_transform);
                    printf("init pos %f %f %f\n", init_trans.p.x, init_trans.p.y, init_trans.p.z);
                    double init_speed = fast_value(value, "init_speed");
                    phys_engine_lock.lock();
                    this->simulatorAircraft->reset_aircraft(transform_e2p(init_transform), init_speed);
                    phys_engine_lock.unlock();

                });

        handler_realtime_output->add_json_handler("pause", [&](const rapidjson::Value &value) {
            printf("Trying to pause simulator \n");
            simulator_running = false;
        });
        handler_realtime_output->add_json_handler("resume", [&](const rapidjson::Value &value) {
            printf("Trying to resume simulator \n");
            simulator_running = true;
        });

        handler_realtime_output->add_json_handler("set_internal_state", [&](const rapidjson::Value &value) {
            for (rapidjson::Value::ConstMemberIterator itr = value.MemberBegin();
                 itr != value.MemberEnd(); ++itr)
            {
                aircraftNode->set_internal_state(
                        itr->name.GetString(),itr->value.GetDouble()
                );
            }
        });

        handler_realtime_output->add_json_handler("set_control_value", [&](const rapidjson::Value &value) {
            for (rapidjson::Value::ConstMemberIterator itr = value.MemberBegin();
                 itr != value.MemberEnd(); ++itr)
            {
                aircraftNode->set_control_value(
                        itr->name.GetString(),itr->value.GetDouble()
                );
            }
        });

        output_timer->async_wait([&](const boost::system::error_code &) {
            this->output();
        });

        if(use_a3)
        {
            a3_adapter = new simulation_dji_a3_adapter(this->aircraftNode);
            a3_adapter->sim_air = this->simulatorAircraft;
            a3_adapter->main_thread();
        }
    }

    void run_next_simulation_tick()
    {
        simulation_timer->expires_at(simulation_timer->expires_at() + simulation_interval);

        this->run_calc_for_a3(tick_time);

        simulation_timer->async_wait([&](const boost::system::error_code &) {
            this->run_next_simulation_tick();
        });
    }

    void calc_thread()
    {
        run_next_simulation_tick();
        new std::thread([&]{
            realtime_calc_io_service.run();
        });
    }

    void output()
    {
        output_timer->expires_at(output_timer->expires_at() + output_interval);
        if (simulator_running) {
            static int count = 0;
            rapidjson::Document d;
            d.SetObject();

            add_transform(d, aircraftNode->get_ground_transform(), d);

            add_vector(d, aircraftNode->get_angular_velocity(), d, "angular_velocity");
            add_vector(d,quat2eulers(aircraftNode->get_ground_attitude()) /M_PI * 180.0,d,"euler");
            add_vector(d,aircraftNode->get_ground_velocity(),d,"vel");

            if (count++ % 10 == 1) {
                rapidjson::Value value(rapidjson::kObjectType);
                Eigen::Matrix3d trans_body_2_world = aircraftNode->get_ground_transform().linear();
                add_vector(value, trans_body_2_world * aircraftNode->get_total_force(), d, "total_force");
                add_vector(value, trans_body_2_world * aircraftNode->get_total_torque(), d, "total_torque");

                add_vector(value, aircraftNode->get_total_aerodynamics_force(), d,
                           "total_airdynamics_force");
                add_vector(value, trans_body_2_world * aircraftNode->get_total_aerodynamics_torque(), d,
                           "total_airdynamics_torque");

                add_vector(value, trans_body_2_world * aircraftNode->get_total_engine_torque(), d,
                           "total_engine_torque");
                add_vector(value, trans_body_2_world * aircraftNode->get_total_engine_force(), d, "total_engine_force");

                rapidjson::Value blade_array(rapidjson::kArrayType);
                blade_array.CopyFrom(
                        *aircraftNode->bladeElementManager.get_blades_information(),
                        d.GetAllocator()
                );
            value.AddMember("blades", blade_array, d.GetAllocator());
                d.AddMember("forces_torques", value, d.GetAllocator());
            }

            rapidjson::Value airspeed_value(rapidjson::kObjectType);
            ComponentData data = aircraftNode->get_component_data();
            AirState airState = aircraftNode->airState;
            add_value(airspeed_value, data.get_airspeed_mag(airState), d, "airspeed");
            add_value(airspeed_value, data.get_angle_of_attack(airState), d, "angle_of_attack");
            add_value(airspeed_value, data.get_sideslip(airState), d, "sideslip");

            if (a3_adapter != nullptr) {
                a3_adapter->add_values(d);
            }
            d.AddMember("airstate", airspeed_value, d.GetAllocator());

            handler_realtime_output->send_json(d);
        }


        output_timer->async_wait([&](const boost::system::error_code &) {
            this->output();
        });
    }

    void run_calc_for_a3(float ticktime)
    {
         if (simulator_running) {
            runninged_tick ++;
            if (a3_adapter != nullptr)
            {
                a3_adapter->simulator_tick = runninged_tick;
                if (a3_adapter->motor_starter || !a3_adapter->assiant_online) {
                    simulatorWorld.Step(ticktime);
                }
            }
            else {
                simulatorWorld.Step(ticktime);
            }
        }
    }

};

int main(int argc, char **argv)
{
    RapidFDM::Common::init_resource_manager(argv);
    if (argc <= 1)
        return -1;
    std::string path = argv[1];
    bool use_a3 = true;
    printf("Loading aircraft %s\n",path.c_str());
    simulation_websocket_server server(9093, path,0.5,5,use_a3);
   std::function<void(void)> cb = ([&]{
       server.a3_adapter->send_realtime_data();
    });
    server.a3_adapter->on_receive_pwm = &cb;

    printf("run server thread\n");
    server.calc_thread();
    server.main_thread();

}
