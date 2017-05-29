//
// Created by xuhao on 2016/12/14.
//

#ifndef RAPIDFDM_SIMULATIN_PIXHAWK_ADAPTER_H
#define RAPIDFDM_SIMULATIN_PIXHAWK_ADAPTER_H

#include "simulation_hil_adapter.h"
#include <string>
#include <RapidFDM/network_protocol/serial_port.h>
#include <mavlink/v2.0/common/mavlink.h>
#include <RapidFDM/network_protocol/udp_client.h>
#include <RapidFDM/network_protocol/udp_server.h>

using namespace RapidFDM::NetworkProtocol;


namespace RapidFDM {
    namespace Simulation {
        class simulation_pixhawk_adapter : public simulation_hil_adapter, public UDPClient, public SerialPort {
        private:
            std::string serial;
            int rate;
            uint8_t target_system = 1;
            uint8_t component_id = 51;

            float RcA = 0, RcE = 0, RcR = 0, RcT = 0;
            float pwm[16] = {0};

            virtual void on_receieve_mavlink_message(mavlink_message_t *msg, uint8_t *buffer, int size);

            bool system_online = false;
            bool simulator_online = false;
            bool motor_started = false;

            uint8_t mavlink_send_fc_buffer[1024 * 16] = {0};

            uint8_t mavlink_send_qgc_buffer[1024 * 16] = {0};


        protected:
            virtual void send_realtime_data();

            virtual void tick_func(float dt,long tick) override;

            void send_mavlink_msg_to_fc(mavlink_message_t *msg);

            virtual void push_json_to_app(rapidjson::Document &d) override;

            virtual void on_receive_data(uint8_t *data, size_t size) override;

            virtual void on_receive_char(char c) override;

        public:
            simulation_pixhawk_adapter(SimulatorAircraft *simulatorAircraft);

            virtual bool enable_simulation() override;

//            virtual void udp_server_on_receive_data(uint8_t *data, size_t size) override;

            virtual void on_message_hil_controls(mavlink_hil_actuator_controls_t *hil_controls);
        };
    }
}

#endif //RAPIDFDM_SIMULATIN_PIXHAWK_ADAPTER_H
