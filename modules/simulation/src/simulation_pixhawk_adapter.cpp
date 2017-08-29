//
// Created by xuhao on 2016/12/14.
//


#include <RapidFDM/simulation/simulation_pixhawk_adapter.h>
#include <mavlink/v2.0/mavlink_helpers.h>
#include <thread>

#define MAV_MODE_FLAG_HIL_ENABLED  32

long current_timestamp_us();

namespace RapidFDM {
    namespace Simulation {
        simulation_pixhawk_adapter::simulation_pixhawk_adapter(SimulatorAircraft *simulatorAircraft) :
                simulation_hil_adapter(simulatorAircraft), UDPClient("127.0.0.1", 14550) {
            intial_lati = 22.3351 * M_PI / 180.0;
            intial_lon = 114.2631 * M_PI / 180.0;

            new std::thread([&] {
                ((SerialPort*)this)->start("/dev/cu.usbmodem1", 230400);
                ((SerialPort*)this)->io_service_.run();
            });
        }

        void simulation_pixhawk_adapter::send_realtime_data() {
            mavlink_hil_state_quaternion_t state;

            Eigen::Quaterniond quat = get_quaternion_NED();
            state.attitude_quaternion[0] = (float) quat.w();
            state.attitude_quaternion[1] = (float) quat.x();
            state.attitude_quaternion[2] = (float) quat.y();
            state.attitude_quaternion[3] = (float) quat.z();

            Eigen::Vector3d global_local = get_lati_lon_alti();
            state.lat = (int32_t) (global_local.x() * 1e7 * 180.0 / M_PI);
            state.lon = (int32_t) (global_local.y() * 1e7 * 180.0 / M_PI);
            state.alt = -(int32_t) (global_local.z() * 1e3);


            Eigen::Vector3d ang_vel = get_angular_velocity_body_NED();
            state.rollspeed = (float) ang_vel.x();// + ((float)(total_tick_count % 2) - 0.5) * 2*0.25;
            state.pitchspeed = (float) ang_vel.y();
            state.yawspeed = (float) ang_vel.z();

            Eigen::Vector3d vel = get_ground_velocity_NED();
            state.vx = (int16_t) (vel.x() * 100.0);
            state.vy = (int16_t) (vel.y() * 100.0);
            state.vz = (int16_t) (vel.z() * 100.0);

            Eigen::Vector3d acc = get_acc_body_NED();
            state.xacc = (int16_t) acc.x();
            state.yacc = (int16_t) acc.y();
            state.zacc = (int16_t) acc.z();

            acc = get_acc_body_NED();

            state.ind_airspeed = (uint16_t) (get_airspeed() * 100);
            state.true_airspeed = (uint16_t) (get_airspeed() * 100);

            mavlink_message_t msg;

            mavlink_msg_hil_state_quaternion_encode(target_system, component_id, &msg, &state);
            send_mavlink_msg_to_fc(&msg);

        }


        void simulation_pixhawk_adapter::send_mavlink_msg_to_fc(mavlink_message_t *msg) {
            uint16_t size = mavlink_msg_to_send_buffer(mavlink_send_fc_buffer, msg);
            write_some((char*)mavlink_send_fc_buffer,size);
        }

        void simulation_pixhawk_adapter::tick_func(float dt, long tick) {
            if (system_online) {
                if (total_tick_count % 200 == 0) {
                    if (!simulator_online) {
                        mavlink_message_t msg;
                        mavlink_set_mode_t set_mode;
                        set_mode.base_mode = 32;
                        set_mode.target_system = target_system;
                        mavlink_msg_set_mode_encode(target_system, component_id, &msg, &set_mode);
                        printf("Try to start simulator %d\n", total_tick_count);
                        send_mavlink_msg_to_fc(&msg);
                    }
                    printf("System online %d\n", total_tick_count);
                }
                if (simulator_online) {
                    if (total_tick_count % 200 == 0)
                        printf("send realtime data %d\n", total_tick_count);
                    send_realtime_data();
                }
            } else {
                if (total_tick_count % 200 == 0) {
                    printf("System offline %d\n", total_tick_count);
                }
            }
        }

        bool simulation_pixhawk_adapter::enable_simulation() {
            return simulator_online && motor_started;
        }

        void simulation_pixhawk_adapter::on_receive_data(uint8_t *data, size_t size) {
            write_some((char*)data,size);
        }


        void simulation_pixhawk_adapter::on_message_hil_controls(mavlink_hil_actuator_controls_t *hil_controls) {
            for (int i = 0; i < 16; i++) {
                pwm[i] = hil_controls->controls[i];
            }
            on_pwm_data_receieve(pwm, 16);
        }

        void
        simulation_pixhawk_adapter::on_receieve_mavlink_message(mavlink_message_t *msg, uint8_t *buffer, int size) {
            target_system = msg->sysid;
            system_online = true;
            write_to_server(buffer, size);
            switch (msg->msgid) {
                case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                    mavlink_hil_actuator_controls_t hil_controls;
                    mavlink_msg_hil_actuator_controls_decode(msg, &hil_controls);
                    on_message_hil_controls(&hil_controls);
                    motor_started = true;
                    break;
                case MAVLINK_MSG_ID_HEARTBEAT:
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(msg, &heartbeat);
//                    if (heartbeat.base_mode & MAV_MODE_FLAG_HIL_ENABLED) {
                        simulator_online = true;
//                    }
                    break;
                case MAVLINK_MSG_ID_RC_CHANNELS:
                    mavlink_rc_channels_t rc;
                    mavlink_msg_rc_channels_decode(msg, &rc);
                    RcA = (rc.chan1_raw - 1520) / 400.0f * 10000;
                    RcE = (rc.chan2_raw - 1520) / 400.0f * 10000;
                    RcT = ((rc.chan3_raw - 1520) / 400.0f + 1) / 2 * 10000;
                    RcR = (rc.chan4_raw - 1520) / 400.0f * 10000;
                    break;
                case MAVLINK_MSG_ID_STATUSTEXT:
                    parse_mavlink_status_text(msg);
                    break;
                default:
                    break;
            }

        }

        void simulation_pixhawk_adapter::parse_mavlink_status_text(mavlink_message_t *msg) {
            mavlink_statustext_t statustext;
            mavlink_msg_statustext_decode(msg, &statustext);
            printf("FC:%d : %s\n",statustext.severity,statustext.text);
            fflush(stdout);

        }

        void simulation_pixhawk_adapter::push_json_to_app(rapidjson::Document &d) {
            rapidjson::Value a3_value(rapidjson::kObjectType);
            add_value(a3_value, 1, d, "motor_started");

            add_value(a3_value, RcA, d, "RcA");
            add_value(a3_value, RcE, d, "RcE");
            add_value(a3_value, RcR, d, "RcR");
            add_value(a3_value, RcT, d, "RcT");

            add_value(a3_value, (int) simulator_online, d, "online");
            add_value(a3_value, 1, d, "sim_mode");

            rapidjson::Value pwm_array(rapidjson::kArrayType);
            for (int ch = 0; ch < 8; ch++) {
                pwm_array.PushBack(pwm[ch] * 100, d.GetAllocator());
            }

            a3_value.AddMember("PWM", pwm_array, d.GetAllocator());
            d.AddMember("sim_status", a3_value, d.GetAllocator());
            //sim mode: 1 hil
            // 0 sitl

        }

//        void simulation_pixhawk_adapter::udp_server_on_receive_data(uint8_t *data, size_t size) {
//            for (int i = 0; i < size; i++) {
//                char c = data[i];
//            }

        void simulation_pixhawk_adapter::on_receive_char(char c) {
            static mavlink_message_t msg;
            static mavlink_status_t status;
            static uint8_t data[16*1024] = {0};
            static int size = 0;
            data[size] = (uint8_t) c;
            size ++;
            if (mavlink_parse_char(0, (uint8_t) c, &msg, &status)) {
                on_receieve_mavlink_message(&msg, data, size);
                size = 0;
            }
        }
    }
}
