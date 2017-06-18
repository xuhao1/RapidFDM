//
// Created by xuhao on 2016/12/15.
//

#ifndef RAPIDFDM_UDP_CLIENT_H
#define RAPIDFDM_UDP_CLIENT_H
#include <boost/asio.hpp>
#include <thread>
using boost::asio::ip::udp;
namespace RapidFDM
{
    namespace NetworkProtocol
    {
        class UDPClient
        {
        private:
            udp::endpoint remote_endpoint_;
            udp::socket * socket_ = nullptr;
            boost::asio::io_service io_service;
            udp::endpoint sender_endpoint;
            uint8_t read_buffer[1024*1024] = {0};
            void start_receive()
            {
                socket_->async_receive_from(
                        boost::asio::buffer(read_buffer,1024*1024), remote_endpoint_,
                        boost::bind(&UDPClient::handle_receive, this,
                                    boost::asio::placeholders::error,
                                    boost::asio::placeholders::bytes_transferred));
            }

            void handle_receive(const boost::system::error_code& error,
                                std::size_t size)
            {
                if (!error || error == boost::asio::error::message_size)
                {
                    on_receive_data(read_buffer,size);
                    start_receive();
                } else{
                    std::cerr << "Error on receive data" << std::endl;
                }
            }
        protected:
            virtual void on_receive_data(uint8_t *data,size_t size ) {
                printf("receive from QGC:%s$\n",data);
            }
        public:
            UDPClient(std::string ip,int port)
            {
                socket_ = new udp::socket(this->io_service);
                boost::asio::ip::address addr;
                addr.from_string(ip);
                remote_endpoint_ = udp::endpoint(addr,port);
                socket_->open(udp::v4());
                printf("Link2 %s port %d\n",ip.c_str(),port);
                start_receive();
                new std::thread([&]{
                    io_service.run();
                });
            }

            void write_to_server(uint8_t * buffer,size_t size)
            {
                if(socket_)
                {
                    if (socket_->send_to(boost::asio::buffer(buffer,size), remote_endpoint_) <= 0)
                    {
                        std::wcerr << "Send to QGC error " << size <<std::endl;
                    }
                }
            }



        };
    }
}

#endif //RAPIDFDM_UDP_CLIENT_H
