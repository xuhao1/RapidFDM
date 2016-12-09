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
            std::string channel_name = "";
            websocketpp::connection_hdl connection_hdl;
            bool opened = false;
            virtual void on_message(std::string msg);
        public:
            ws_channel_handler(websocket_server * server_ptr,std::string channel);
            void send(std::string msg);
        };


        class ws_json_channel_handler : public ws_channel_handler
        {
        public:
            typedef std::function<void(const rapidjson::Value &)> jsonvalue_callback;
        protected:

            virtual void on_message(std::string msg) override;
//            virtual
            std::map<std::string,jsonvalue_callback> json_handler;
        public:
            ws_json_channel_handler(websocket_server * server_ptr,std::string channel):
                ws_channel_handler(server_ptr,channel)
            {}
            void send_json(rapidjson::Document & document);
            void add_json_handler(std::string opcode,jsonvalue_callback cb);
        };
    }
}

#endif //RAPIDFDM_WS_CHANNEL_HANDLER_H
