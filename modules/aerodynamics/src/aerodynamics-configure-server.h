//
// Created by Hao Xu on 16/5/9.
//

#ifndef RAPIDFDM_AERODYNAMICS_CONFIGURE_SERVER_H
#define RAPIDFDM_AERODYNAMICS_CONFIGURE_SERVER_H

#include <FlyingData.h>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;
typedef server::message_ptr message_ptr;


using namespace RapidFDM::Aerodynamics;

class configure_server
{
public:
    AirState realtime_air_state;
    server server;

    int init();
    configure_server()
    {
        if (init() == 1)
            exit(1);
    }

    void main_thread()
    {
    }
};

#endif //RAPIDFDM_AERODYNAMICS_CONFIGURE_SERVER_H
