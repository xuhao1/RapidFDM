//
// Created by Hao Xu on 16/7/2.
//

#include <RapidFDM/network_protocol/ws_channel_handler.h>
#include <RapidFDM/utils.h>
#include <iostream>

using namespace RapidFDM::Utils;


namespace RapidFDM
{
    namespace NetworkProtocol
    {
        ws_channel_handler::ws_channel_handler(websocket_server *server_ptr, std::string channel)
        {
            assert(server_ptr != nullptr);
            server_ptr->add_open_hande_function("/" + channel, [&](ws_server *ws, websocketpp::connection_hdl hdl) {
                this->wsServer = ws;
                connection_hdl = hdl;
                std::cout << "ws channel handle get open " << std::endl;
            });
            server_ptr->add_message_hande_function("/" + channel, [&](ws_server *ws, websocketpp::connection_hdl hdl,
                                                                      message_ptr msg) {
                this->on_message(msg->get_payload());
                std::cout << "ws channel handle get msg " << std::endl;
            });

        }

        void ws_channel_handler::on_message(std::string msg)
        {
            std::cout << "channel msg" << msg << std::endl;
            this->send("fuck off\n");
        }

        void ws_channel_handler::send(std::string msg)
        {
            wsServer->send(connection_hdl, msg, websocketpp::frame::opcode::TEXT);
        }
        void ws_json_channel_handler::add_json_handler(std::string opcode, jsonvalue_callback cb)
        {
            this->json_handler[opcode] = cb;
        }

        void ws_json_channel_handler::send_json(rapidjson::Document &document)
        {
            int32_t size;
            const char * str = json_to_buffer(document,size);
            wsServer->send(connection_hdl,str,size,websocketpp::frame::opcode::text);
        }

        void ws_json_channel_handler::on_message(std::string msg)
        {
            rapidjson::Document d;
            d.Parse(msg.c_str());
            if (!d.IsObject()) {
                std::wcerr << "Parse document failed!" << std::endl;
            }
            else {
                std::string opcode = fast_string(d, "opcode");
                if (json_handler.find(opcode) != json_handler.end()) {
                    if (d.HasMember("data")) {
                        const rapidjson::Value & v = d["data"];
                        json_handler[opcode](v);
                    }
                    else {
                        std::cerr << "Msg has no data field" << std::endl;
                    }
                }
                else {
                    std::cerr << "Channel " << channel_name << " have no function for opcode " << opcode << std::endl;
                }
            }
        }
    }
}
