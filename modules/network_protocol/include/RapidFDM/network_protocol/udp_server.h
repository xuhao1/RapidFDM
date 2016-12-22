//
// Created by xuhao on 2016/12/21.
//

#ifndef RAPIDFDM_UDP_SERVER_H
#define RAPIDFDM_UDP_SERVER_H

#include <boost/asio.hpp>
using boost::asio::ip::udp;
namespace RapidFDM
{
    namespace NetworkProtocol
    {
        class UDPServer
        {
        private:
            udp::endpoint remote_endpoint_;
            udp::socket * socket_ = nullptr;
            boost::asio::io_service io_service;
            uint8_t read_buffer[1024*1024] = {0};
            bool link_online = false;
            void start_receive()
            {
                socket_->async_receive_from(
                        boost::asio::buffer(read_buffer,1024*1024), remote_endpoint_,
                        boost::bind(&UDPServer::handle_receive, this,
                                    boost::asio::placeholders::error,
                                    boost::asio::placeholders::bytes_transferred));
            }

            void handle_receive(const boost::system::error_code& error,
                                std::size_t size)
            {
                if (!error || error == boost::asio::error::message_size)
                {
                    link_online = true;
                    udp_server_on_receive_data(read_buffer,size);
                    start_receive();
                } else{
                    std::cerr << "Error on receive data" << std::endl;
                }
            }
        protected:
            virtual void udp_server_on_receive_data(uint8_t *data,size_t size ) {
                printf("receive from Client:%s$\n",data);
                write_to_client(data,size);
            }
        public:
            UDPServer(int port)
            {
                socket_ = new udp::socket(this->io_service, udp::endpoint(udp::v4(),port));
                printf("Listen port %d\n",port);
                start_receive();
                boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service));
            }

            void write_to_client(uint8_t * buffer,size_t size)
            {
                if(socket_ && link_online)
                {
                    socket_->send_to(boost::asio::buffer(buffer,size), remote_endpoint_);
                }
            }



        };
    }
}
#endif //RAPIDFDM_UDP_SERVER_H
