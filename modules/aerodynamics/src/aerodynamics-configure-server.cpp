//
// Created by Hao Xu on 16/5/9.
//

#include "aerodynamics-configure-server.h"
#include <iostream>

int configure_server::init()
{
    return 0;
}
int main()
{
    configure_server server;
    server.main_thread();
    return 0;
}