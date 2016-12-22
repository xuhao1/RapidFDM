//
// Created by Hao Xu on 16/8/16.
//

#ifndef RAPIDFDM_SERIAL_PORT_H
#define RAPIDFDM_SERIAL_PORT_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <string>
#include <vector>


#define SERIAL_PORT_READ_BUF_SIZE 1024

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;


namespace RapidFDM
{
    namespace NetworkProtocol
    {
        class SerialPort
        {
        protected:
            serial_port_ptr port_;
            boost::mutex mutex_;
        
            char read_buf_raw_[SERIAL_PORT_READ_BUF_SIZE];
            std::string read_buf_str_;
        
        private:
            SerialPort(const SerialPort &p);
            SerialPort &operator=(const SerialPort &p);
    
        public:
            boost::asio::io_service io_service_;
            SerialPort(void);
            virtual ~SerialPort(void);
        
            virtual bool start(std::string com_port_name, int baud_rate=230400);
            virtual void stop();
        
            int write_some(const char *buf, const int &size);
        
        protected:
            virtual void async_read_some_();
            virtual void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred);

            virtual void on_receive_char(char c) {
                printf("%c",c);
            };

        };
    }
}

#endif //RAPIDFDM_SERIAL_PORT_H
