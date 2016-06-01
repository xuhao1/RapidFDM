//
// Created by Hao Xu on 16/5/9.
//

#include "aerodynamics-configure-server.h"
#include <iostream>
using namespace lcm;

int configure_server::init()
{
    if(!lcm.good()) {
        std::cerr << "Lcm not good" << std::endl;
        return 1;
    }
    printf("LCM Ready Wait for message\n");
    lcm.subscribe("EXAMPLE", &configure_server::handle_message,this);
    return 0;
}
int main()
{
    configure_server server;
    server.main_thread();
    return 0;
}