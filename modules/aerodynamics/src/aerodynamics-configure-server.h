//
// Created by Hao Xu on 16/5/9.
//

#ifndef RAPIDFDM_AERODYNAMICS_CONFIGURE_SERVER_H
#define RAPIDFDM_AERODYNAMICS_CONFIGURE_SERVER_H

#include <FlyingData.h>
#include <lcm/lcm-cpp.hpp>

using namespace RapidFDM::Aerodynamics;

class configure_server
{
public:
    AirState realtime_air_state;

    void handle_message(const lcm::ReceiveBuffer *rbuf,
                        const std::string &chan,
                        const rapidfdmlcm::airstate *msg)
    {
        printf("Received message on channel \"%s\":\n", chan.c_str());
        printf("Airstate: rho %f vx %f vy %f vz %f", msg->rho, msg->wind_speed[0], msg->wind_speed[1],
               msg->wind_speed[2]);
    }

    int init();

    lcm::LCM lcm;

    configure_server()
    {
        if (init() == 1)
            exit(1);
    }

    void main_thread()
    {
        while(0 == lcm.handle());
    }
};

#endif //RAPIDFDM_AERODYNAMICS_CONFIGURE_SERVER_H
