//
// Created by Hao Xu on 16/8/16.
//

#include <RapidFDM/network_protocol/serial_port.h>


namespace RapidFDM
{
    namespace NetworkProtocol
    {
        SerialPort::SerialPort(void)
        {
        }
        
        SerialPort::~SerialPort(void)
        {
            stop();
        }
        
        bool SerialPort::start(std::string com_port_name, int baud_rate)
        {
            boost::system::error_code ec;
            std::cout << "try to open" << com_port_name << std::endl;
            if (port_) {
                std::cout << "error : port is already opened..." << std::endl;
                return false;
            }
            
            port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
            port_->open(com_port_name.c_str(), ec);
            if (ec) {
                std::cout << "error : port_->open() failed...com_port_name="
                          << com_port_name << ", e=" << ec.message().c_str() << std::endl;
                return false;
            }
            
            // option settings...
            port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
            port_->set_option(boost::asio::serial_port_base::character_size(8));
            port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            port_->set_option(
                    boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            
            boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service_));
    
            printf("Successful open port,start receieve data \n");
            async_read_some_();
            
            return true;
        }
        
        void SerialPort::stop()
        {
            boost::mutex::scoped_lock look(mutex_);
            
            if (port_) {
                port_->cancel();
                port_->close();
                port_.reset();
            }
            io_service_.stop();
            io_service_.reset();
        }
        
        int SerialPort::write_some(const char *buf, const int &size)
        {
            boost::system::error_code ec;
            
            if (!port_) return -1;
            if (size == 0) return 0;
            
            return port_->write_some(boost::asio::buffer(buf, size), ec);
        }
        
        void SerialPort::async_read_some_()
        {
            if (port_.get() == NULL || !port_->is_open()) return;
            
            port_->async_read_some(
                    boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
                    boost::bind(
                            &SerialPort::on_receive_,
                            this, boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
        }
        
        void SerialPort::on_receive_(const boost::system::error_code &ec, size_t bytes_transferred)
        {
            boost::mutex::scoped_lock look(mutex_);
            
            if (port_.get() == NULL || !port_->is_open()) return;
            if (ec) {
                async_read_some_();
                return;
            }
            
            for (unsigned int i = 0; i < bytes_transferred; ++i) {
                char c = read_buf_raw_[i];
                printf("%c",c);
            }
            
            async_read_some_();
        }
        
    }
}