//
// Created by Hao Xu on 16/7/2.
//

#include <RapidFDM/network_protocol/ws_channel_handler.h>
namespace RapidFDM
{
    namespace NetworkProtocol
    {
        ws_channel_handler::ws_channel_handler(websocket_server *server_ptr,std::string channel)
        {
            assert(server_ptr!= nullptr);
            server_ptr->add_open_hande_function("/"+channel,[&](ws_server * ws,websocketpp::connection_hdl hdl){
                this->wsServer =   ws;
                connection_hdl = hdl;
                std::cout << "ws channel handle get open " << std::endl;
            });
            server_ptr->add_message_hande_function("/"+channel,[&](ws_server*ws,websocketpp::connection_hdl hdl,message_ptr msg){
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
            wsServer->send(connection_hdl,msg,websocketpp::frame::opcode::TEXT);
        }
    }
}
