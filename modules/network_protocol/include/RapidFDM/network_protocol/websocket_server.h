//
// Created by Hao Xu on 16/5/9.
//

#ifndef RAPIDFDM_AERODYNAMICS_CONFIGURE_SERVER_H
#define RAPIDFDM_AERODYNAMICS_CONFIGURE_SERVER_H

#include <RapidFDM/aerodynamics/flying_data_defines.h>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <RapidFDM/aerodynamics/airdynamics_parser.h>
#include <functional>
#include <stdio.h>
#include <mutex>
#include <boost/bind.hpp>
#include <vector>

typedef websocketpp::server<websocketpp::config::asio> ws_server;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;
typedef ws_server::message_ptr message_ptr;


using namespace RapidFDM::Aerodynamics;

namespace RapidFDM
{
    namespace NetworkProtocol
    {
        typedef std::function<void(ws_server *, websocketpp::connection_hdl, message_ptr)> message_handle_function;
        typedef std::function<void(ws_server *, websocketpp::connection_hdl)> open_handle_function;
        class websocket_server
        {
        public:
            bool online = false;
            ws_server server;
            websocketpp::lib::asio::io_service * io_service_ptr = nullptr;

            std::map<std::string,message_handle_function> message_handlers;
            std::map<std::string,open_handle_function> open_handlers;
            std::vector <websocketpp::connection_hdl> hdls;

            int init(int port);
            void on_message(ws_server* s, websocketpp::connection_hdl hdl, message_ptr msg);
            void on_connect(ws_server* s,websocketpp::connection_hdl hdl);
            void add_message_hande_function(std::string uri,message_handle_function func);
            void add_open_hande_function(std::string uri,open_handle_function func);
            void on_failed(ws_server *s,websocketpp::connection_hdl hdl);
            websocket_server(int port)
            {
                std::cout << "Create ws on port " << port << std::endl;
                if (init(port) == 1)
                    exit(1);
                io_service_ptr  = &server.get_io_service();
            }
            void stop_server()
            {
                printf("Trying to stop server\n");
                for (websocketpp::connection_hdl hdl : hdls)
                {
                    ws_server::connection_ptr con = server.get_con_from_hdl(hdl);
                    con->close(0,"");
                }
                this->server.stop_listening();
                this->server.stop();
                exit(0);
            }
            void main_thread()
            {
                std::cout << "start accept ws server\n";
                boost::asio::signal_set signals(server.get_io_service(), SIGINT, SIGTERM);
                signals.async_wait(
                        boost::bind(&websocket_server::stop_server, this));
                server.start_accept();
                server.run();
            }
        };


    }
}

#endif //RAPIDFDM_AERODYNAMICS_CONFIGURE_SERVER_H
