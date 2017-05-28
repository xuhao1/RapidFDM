//
// Created by Hao Xu on 16/7/2.
//

#include <RapidFDM/network_protocol/websocket_server.h>
#include <RapidFDM/network_protocol/ws_channel_handler.h>
#include <RapidFDM/network_protocol/serial_port.h>
#include <RapidFDM/network_protocol/udp_client.h>
#include <RapidFDM/network_protocol/udp_server.h>
#include <RapidFDM/utils.h>
#include <thread>

using namespace RapidFDM::NetworkProtocol;

int network() {
    printf("Hello,world\n");
    websocket_server ws(9091);
    ws_json_channel_handler handler(&ws, "channel");
    handler.add_json_handler(
            "crystal",
            [&](const rapidjson::Value &v) {
                std::cout << "recieve crystal" << std::endl;
                rapidjson::Document d;
                d.SetObject();
                d.AddMember("test", "fuck", d.GetAllocator());
                handler.send_json(d);
            }
    );
//    handle
    ws.main_thread();
    return 0;
}

int serial() {
    printf("Starting test serial\n");
    SerialPort serialPort;
    std::thread th([&] {
        serialPort.start("/dev/tty.usbmodem1", 230400);
    });
    //usleep(100000);
    uint8_t msg[1024] = {0};
    int size;
    printf("try to send simulator start\n");
    serialPort.write_some((char *) msg, size);
    usleep(10000000);
    serialPort.stop();
    return 0;
}

int udp() {
    UDPClient client("127.0.0.1", 14550);
    std::string str = "hello,world";
    client.write_to_server((uint8_t*)str.c_str(), str.size());
//    client.main_thread();
    while(true);
    return 0;
}

int udp_server()
{
    UDPServer udpServer(9095);
    while(true);
}

int main() {
//    udp();
    serial();
}

