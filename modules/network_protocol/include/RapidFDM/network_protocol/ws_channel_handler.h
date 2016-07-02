//
// Created by Hao Xu on 16/7/2.
//

#ifndef RAPIDFDM_WS_CHANNEL_HANDLER_H
#define RAPIDFDM_WS_CHANNEL_HANDLER_H

#include "websocket_server.h"
#include <string>
#include <rapidjson/document.h>

namespace RapidFDM
{
    namespace NetworkProtocol
    {
        //Note: every channel handler only accept one connection
        class ws_channel_handler
        {
        protected:
            websocket_server * server_ptr = nullptr;
            ws_server * wsServer;
            websocketpp::connection_hdl connection_hdl;
            virtual void on_message(std::string msg);
        public:
            ws_channel_handler(websocket_server * server_ptr,std::string channel);
            void send(std::string msg);
        };
    }
}

#endif //RAPIDFDM_WS_CHANNEL_HANDLER_H
