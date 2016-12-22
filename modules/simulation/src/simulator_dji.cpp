//
// Created by xuhao on 2016/12/23.
//
#include <RapidFDM/simulation/simulation_websocket_server.h>

int main(int argc, char **argv) {
    RapidFDM::Common::init_resource_manager(argv);
    if (argc <= 1)
        return -1;
    std::string path = argv[1];
    printf("Loading aircraft %s\n", path.c_str());
    simulation_websocket_server server(path);
    simulation_hil_adapter * hil_adapter = (simulation_hil_adapter *) new simulation_dji_a3_adapter(server.simulatorAircraft);
    printf("run hil thread\n");
    hil_adapter->main_thread();
    server.hil_adapter = hil_adapter;
    printf("run server thread\n");
    server.main_thread();

}