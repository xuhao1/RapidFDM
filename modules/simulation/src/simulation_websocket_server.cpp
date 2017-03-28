#include <RapidFDM/simulation/simulation_websocket_server.h>
#include <RapidFDM/simulation/simulation_sitl_adapter.h>

using namespace RapidFDM::NetworkProtocol;
using namespace RapidFDM::Simulation;
using namespace RapidFDM::Simulation::Utils;
using namespace RapidFDM::Utils;
using namespace RapidFDM;

long current_timestamp() {
#ifdef WIN32
    SYSTEMTIME time;
    GetSystemTime(&time);
    LONG time_ms = (time.wSecond * 1000) + time.wMilliseconds;
    return time_ms;
#else
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000; // caculate milliseconds
    return milliseconds;

#endif // WIN32
}

long current_timestamp_us() {
#ifdef WIN32
    SYSTEMTIME time;
    GetSystemTime(&time);
    LONG time_ms = (time.wSecond * 1000) + time.wMilliseconds;
    return time_ms;
#else
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec * 1000000LL + te.tv_usec; // caculate milliseconds
    return milliseconds;
#endif // WIN32
}


namespace RapidFDM {
    namespace Simulation {
        simulation_websocket_server::simulation_websocket_server(std::string aircraft_path,
                                                                 float phys_rate,
                                                                 float hil_rate, int port) :
                websocket_server(port), simulatorWorld(1000.0f / phys_rate),
                simulation_interval((int) (1000.0 / hil_rate)),
                output_interval(15) {

            parser parser1(aircraft_path);
            aircraftNode = parser1.get_aircraft_node();
            assert(aircraftNode != nullptr);

            simulatorAircraft = simulatorWorld.create_aircraft(aircraftNode);

            init_ws_json_handler();

            this->tick_time = (int) (1000.0 / hil_rate);

            output_timer = new boost::asio::deadline_timer(*this->io_service_ptr, output_interval);
            output_timer->async_wait([&](const boost::system::error_code &) {
                this->output();
            });

            this->calc_thread();
        }

        void simulation_websocket_server::init_ws_json_handler() {
            handler_realtime_output = new ws_json_channel_handler((websocket_server *) this, "output");

            handler_realtime_output->add_json_handler(
                    "start", [&](const rapidjson::Value &value) {
                        Eigen::Affine3d init_transform = fast_transform(value, "init_transform");
                        auto init_trans = transform_e2p(init_transform);
                        printf("Init pos %f %f %f\n", init_trans.p.x, init_trans.p.y, init_trans.p.z);
                        double init_speed = fast_value(value, "init_speed");
                        phys_engine_lock.lock();
                        this->simulatorAircraft->reset_aircraft(transform_e2p(init_transform), init_speed);
                        phys_engine_lock.unlock();

                        printf("Simulation start!!\n");

                    });

            handler_realtime_output->add_json_handler("pause", [&](const rapidjson::Value &value) {
                printf("Trying to pause simulator \n");
            });
            handler_realtime_output->add_json_handler("resume", [&](const rapidjson::Value &value) {
                printf("Trying to resume simulator \n");
            });

            handler_realtime_output->add_json_handler("set_internal_state", [&](const rapidjson::Value &value) {
                for (rapidjson::Value::ConstMemberIterator itr = value.MemberBegin();
                     itr != value.MemberEnd(); ++itr) {
                    aircraftNode->set_internal_state(
                            itr->name.GetString(), itr->value.GetDouble()
                    );
                }
            });

            handler_realtime_output->add_json_handler("set_control_value", [&](const rapidjson::Value &value) {
                for (rapidjson::Value::ConstMemberIterator itr = value.MemberBegin();
                     itr != value.MemberEnd(); ++itr) {
                    aircraftNode->set_control_value(
                            itr->name.GetString(), itr->value.GetDouble()
                    );
                }
            });

            handler_realtime_output->add_json_handler("set_channel_value", [&](const rapidjson::Value &value) {
                if (!value.IsArray())
                    return;
                const rapidjson::Value &array = value;
                float pwm[8] = {0};
                for (int i = 0; i < 8; i++) {
                    pwm[i] = (float) fast_value(value, i, 0);
                }

                simulation_sitl_adapter *sitl = dynamic_cast <simulation_sitl_adapter *>(hil_adapter);
                if (sitl != nullptr) {
                    sitl->handle_chn_from_joystick(pwm, 8);
                }
            });


        }

        void simulation_websocket_server::run_next_simulation_tick() {
            simulation_timer->expires_at(simulation_timer->expires_at() + simulation_interval);

            this->run_phys(tick_time);

            simulation_timer->async_wait([&](const boost::system::error_code &) {
                this->run_next_simulation_tick();
            });
        }

        void simulation_websocket_server::calc_thread() {
            simulation_timer = new boost::asio::deadline_timer(realtime_calc_io_service, simulation_interval);
            simulation_timer->async_wait([&](const boost::system::error_code &) {
                this->run_next_simulation_tick();
            });
            new std::thread([&] {
                realtime_calc_io_service.run();
            });
        }

        void simulation_websocket_server::output() {
            output_timer->expires_at(output_timer->expires_at() + output_interval);
            static int count = 0;
            rapidjson::Document d;
            d.SetObject();

            add_transform(d, aircraftNode->get_ground_transform(), d);

            add_vector(d, aircraftNode->get_angular_velocity(), d, "angular_velocity");
            add_vector(d, quat2eulers(aircraftNode->get_ground_attitude()) / M_PI * 180.0, d, "euler");
            add_vector(d, aircraftNode->get_ground_velocity(), d, "vel");

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

            if (hil_adapter != nullptr) {
                hil_adapter->push_json_to_app(d);
            }
            d.AddMember("airstate", airspeed_value, d.GetAllocator());

            handler_realtime_output->send_json(d);

            output_timer->async_wait([&](const boost::system::error_code &) {
                this->output();
            });

        }

        void simulation_websocket_server::run_phys(float ticktime) {
            if (hil_adapter != nullptr) {
                if (hil_adapter->enable_simulation()) {
                    runninged_tick++;
                    simulatorWorld.Step(ticktime);
                }
            } else {
                runninged_tick++;
                simulatorWorld.Step(ticktime);
            }
        }
    }
}
