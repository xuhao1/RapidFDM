//
// Created by xuhao on 2016/11/5.
//

#include <RapidFDM/network_protocol/serial_port.h>
using namespace RapidFDM::NetworkProtocol;

int main()
{
    SerialPort serial;
    serial.start("/dev/cu.usbmodem1411",230400);
    serial.io_service_.run();
}