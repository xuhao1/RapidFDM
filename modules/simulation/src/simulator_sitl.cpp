//
// Created by xuhao on 2017/3/27.
//

#include <RapidFDM/simulation/simulation_sitl_adapter.h>
#include <RapidFDM/simulation/simulation_websocket_server.h>
#include <RapidFDM/control_system/so3_adaptive_controller.h>

int main(int argc,char ** argv)
{
     RapidFDM::Common::init_resource_manager(argv);
    if (argc <= 1)
        return -1;
    std::string path = argv[1];

    printf("Loading aircraft %s\n", path.c_str());
    simulation_websocket_server server(path);
    so3_adaptive_controller * so3_adaptive_controller1 =
            new so3_adaptive_controller(server.simulatorAircraft->get_aircraft_node());
    simulation_hil_adapter * hil_adapter =
            new simulation_sitl_adapter(server.simulatorAircraft,so3_adaptive_controller1);

    printf("run sitl thread\n");
    hil_adapter->main_thread();
    server.hil_adapter = hil_adapter;
    printf("run server thread\n");
    server.main_thread();

    return 0;
}