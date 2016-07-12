//
// Created by Hao Xu on 16/5/9.
//

#include <iostream>
#include "../include/RapidFDM/network_protocol/websocket_server.h"

namespace RapidFDM
{
    namespace NetworkProtocol
    {
        void websocket_server::on_message(ws_server *s, websocketpp::connection_hdl hdl, message_ptr msg)
        {
            websocketpp::server<websocketpp::config::asio>::connection_ptr con = server.get_con_from_hdl(hdl);
            websocketpp::uri_ptr uri = con->get_uri();
//            std::cout << "on_message called with hdl: " << hdl.lock().get() << " URI:" << uri->get_resource() <<
//            " and message: " << msg->get_payload() <<
//            std::endl;
            if (message_handlers.find(uri->get_resource())!= message_handlers.end())
            {
                message_handlers[uri->get_resource()](s,hdl,msg);
            }
            else
            {
                std::cerr << "no callback " << uri->get_resource() <<  " found \n";
            }
        }

        void websocket_server::add_message_hande_function(std::string uri, message_handle_function func)
        {
            message_handlers[uri] = func;
        }

        void websocket_server::add_open_hande_function(std::string uri, open_handle_function func)
        {
            open_handlers[uri] = func;
        }

        void websocket_server::on_connect(ws_server *s, websocketpp::connection_hdl hdl)
        {
            websocketpp::server<websocketpp::config::asio>::connection_ptr con = server.get_con_from_hdl(hdl);
            websocketpp::uri_ptr uri = con->get_uri();
//            std::cout << "on_message called with hdl: " << hdl.lock().get() << " URI:" << uri->get_resource() << std::endl;
            if (open_handlers.find(uri->get_resource())!= open_handlers.end())
            {
                open_handlers[uri->get_resource()](s,hdl);
            }
        }

        int websocket_server::init(int port)
        {
            server.set_access_channels(websocketpp::log::alevel::all);
            server.clear_access_channels(websocketpp::log::alevel::frame_payload);
            server.init_asio();
            // Register our message handler
            server.set_message_handler(bind(&websocket_server::on_message, this, &server, ::_1, ::_2));
            server.set_open_handler(bind(&websocket_server::on_connect,this,&server,::_1));
            // Listen on port
            server.listen(port);
            return 0;
        }
    }
}
