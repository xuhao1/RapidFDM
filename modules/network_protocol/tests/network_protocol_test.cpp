//
// Created by Hao Xu on 16/7/2.
//

#include <RapidFDM/network_protocol/websocket_server.h>
#include <RapidFDM/network_protocol/ws_channel_handler.h>

using namespace RapidFDM::NetworkProtocol;
int main()
{
    printf("Hello,world\n");
    websocket_server ws(9091);
    ws_json_channel_handler handler(&ws,"channel");
//    handle
    ws.main_thread();

}

