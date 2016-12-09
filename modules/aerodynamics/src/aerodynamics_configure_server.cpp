//
// Created by Hao Xu on 16/7/3.
//

#include <RapidFDM/aerodynamics/aerodynamics_configurer.h>
#include <RapidFDM/network_protocol/ws_channel_handler.h>

using namespace RapidFDM::NetworkProtocol;
using namespace RapidFDM::Aerodynamics;

class aerodynamics_network_configurer : public aerodynamics_configurer, public websocket_server
{
protected:
    ws_json_channel_handler *handler_query_model = nullptr;

public:
    aerodynamics_network_configurer(std::string root_path, int port) :
            aerodynamics_configurer(root_path), websocket_server(port)
    {
        handler_query_model = new ws_json_channel_handler((websocket_server *) this, "query");
        handler_query_model->add_json_handler(
                "query_model",
                [&](const rapidjson::Value &v) {
                    rapidjson::Document * d = this->query_model(v);
                    handler_query_model->send_json(*d);
                });

        handler_query_model->add_json_handler(
                "query_configurer",
                [&](const rapidjson::Value &v) {
                    rapidjson::Document * d = this->query_configurer(v);
                    handler_query_model->send_json(*d);
                });
    }
};

int main(int argc,char**argv)
{
	std::string aircraft_path = argv[1];
	printf("Start configure server at %s\n",aircraft_path.c_str());
    aerodynamics_network_configurer configurer(aircraft_path,9091);
    configurer.main_thread();
    printf("Hello,world\n");
}