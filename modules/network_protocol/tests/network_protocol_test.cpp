//
// Created by Hao Xu on 16/7/2.
//

#include <RapidFDM/network_protocol/websocket_server.h>
#include <RapidFDM/network_protocol/ws_channel_handler.h>
#include <RapidFDM/network_protocol/serial_port.h>
#include <RapidFDM/utils.h>
#include <unistd.h>
#include <thread>

using namespace RapidFDM::NetworkProtocol;
int network()
{
    printf("Hello,world\n");
    websocket_server ws(9091);
    ws_json_channel_handler handler(&ws,"channel");
    handler.add_json_handler(
            "crystal",
            [&](const rapidjson::Value & v){
                std::cout << "recieve crystal" << std::endl;
                rapidjson::Document d;
                d.SetObject();
                d.AddMember("test","fuck",d.GetAllocator());
                handler.send_json(d);
            }
    );
//    handle
    ws.main_thread();
    return 0;
}
#pragma pack(1)
struct basic_v2_pack_header_t {
    uint8_t head_sign;
    uint16_t length:10;
    uint16_t version:6;
    uint8_t header_checksum;
    uint8_t sender_id:5;
    uint8_t sender_index:3;
    uint8_t receiever_id:5;
    uint8_t receiever_index:3;
    uint16_t seqnum;
    uint8_t encrypt_type:4;
    uint8_t  reserved:1;
    uint8_t ack_type:2;
    uint8_t cmd_type:1;
    uint8_t cmd_set;
    uint8_t cmd_id;
};
struct sim_config
{
    uint8_t rc_type : 1;
    uint8_t reserved0 : 1;
    uint8_t battery_sim : 1;
    uint8_t only_aircraft:1;
    uint8_t use_ns : 1;
    uint8_t reserved:3;
};
struct sim_commmand_msg
{
    uint8_t command;
    sim_config config;
    uint8_t fresh_tick;
    uint8_t gps_svn;
    double lon;
    double lat;
    double alti;
    uint32_t trans_data;
    uint8_t reserved3;
    uint8_t sim_type;
    uint8_t sim_version;
    uint8_t reserved;
};
#pragma pack()
uint16_t make_a3_cmd(uint8_t * buf,uint8_t *msg,int msg_size)
{
    basic_v2_pack_header_t header;
    uint16_t size = sizeof(header) +sizeof(sim_commmand_msg) + 2;
    memset(buf,9,size);
    header.head_sign = 0x55;
    header.length =size;
    header.version = 0;
    header.header_checksum = CRC8((uint8_t*)&header,3,0x31);
    header.sender_id = 0;
    header.sender_index = 10;
    header.receiever_id = 0;
    header.receiever_index = 3;
    header.seqnum = 0;
    header.cmd_type = 0;
    header.ack_type = 0;
    header.encrypt_type = 0;
    header.cmd_set = 11;
    header.cmd_id = 4;
    memcpy(buf,&header,sizeof(header));
    uint16_t crc = gen_crc16(buf,size-2);
    memcpy(buf+size - 2 ,&crc, sizeof(uint16_t));
    return size;
}
void make_start_simulator_cmd(uint8_t* buf,int & size)
{
    sim_commmand_msg cmd;
    cmd.command = 0;
    cmd.config.battery_sim = 0;
    cmd.config.only_aircraft = 0;
    cmd.config.use_ns = 1;
    cmd.config.rc_type = 1;
    cmd.fresh_tick = 8;
    cmd.gps_svn = 5;
    cmd.lat = 0;
    cmd.lon = 0;
    cmd.alti = 0;
    cmd.trans_data = 3758096384;
    cmd.sim_type = 4;
    cmd.sim_version = 0;

    size = make_a3_cmd(buf,(uint8_t*)&cmd,sizeof(cmd));

}


int serial()
{
    printf("Starting test serial\n");
    SerialPort serialPort;
    std::thread th([&]{
        serialPort.start("/dev/cu.usbmodem1411",230400);
    });
    usleep(100000);
    uint8_t  msg[1024]= {0};
    int size;
    make_start_simulator_cmd(msg,size);
    printf("try to send simulator start\n");
    serialPort.write_some((char *)msg,size);
    usleep(10000000);
    serialPort.stop();
    return 0;
}

int main(){
    serial();
}

