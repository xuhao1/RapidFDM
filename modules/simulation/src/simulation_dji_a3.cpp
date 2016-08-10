//
// Created by Hao Xu on 16/8/11.
//

#include "RapidFDM/simulation/simulation_dji_a3_adapter.h"

namespace RapidFDM
{
    namespace Simulation
    {
        simulation_dji_a3_adapter::simulation_dji_a3_adapter(AircraftNode * aircraft):
            interval(10)
        {
            this->aircraft = aircraft;
            
            std::string root_uri = "ws://localhost:19870/general";
    
            c_root.set_access_channels(websocketpp::log::alevel::none);
            c_root.clear_access_channels(websocketpp::log::alevel::all);
            
            c_root.init_asio();
            
            c_root.set_message_handler(bind(&simulation_dji_a3_adapter::on_message_root, this, &c_root, ::_1, ::_2));
            
            websocketpp::lib::error_code ec;
            client::connection_ptr con = c_root.get_connection(root_uri, ec);
            if (ec) {
                std::cout << "could not create connection to A3 because: " << ec.message() << std::endl;
                std::abort();
            }
            
            // Note that connect here only requests a connection. No network messages are
            // exchanged until the event loop starts running in the next line.
            c_root.connect(con);
    
    
            timer = new boost::asio::deadline_timer(c_root.get_io_service(), interval);
    
        }
        
        void simulation_dji_a3_adapter::on_message_root(client *c, websocketpp::connection_hdl hdl, message_ptr msg)
        {
    
            rapidjson::Document d;
            d.Parse(msg->get_payload().c_str());
            if (!d.IsObject() || !d.HasMember("FILE") || !d["FILE"].IsString()) {
                std::wcerr << "Parse a3 assiant document failed!" << std::endl;
                return;
            }
            
            std::string file_name = d["FILE"].GetString();
            std::cout << "Try to connect simulator on " << file_name << std::endl;
            
            std::string uri = "ws://localhost:19870/controller/simulator/" + file_name;
                websocketpp::lib::error_code ec;
                std::cout << uri << std::endl;
                c_simulator = new client;
                c_simulator->set_access_channels(websocketpp::log::alevel::none);
                c_simulator->clear_access_channels(websocketpp::log::alevel::all);
                c_simulator->init_asio(&c_root.get_io_service());
                
                
                c_simulator->set_message_handler(
                        bind(&simulation_dji_a3_adapter::on_message_simulator, this, c_simulator, ::_1, ::_2));
                c_simulator->set_open_handler(
                        bind(&simulation_dji_a3_adapter::on_simulator_link_open, this, c_simulator, ::_1));
    
                client::connection_ptr con = c_simulator->get_connection(uri, ec);
                
                c_simulator->connect(con);
    
        }
        
        void simulation_dji_a3_adapter::on_simulator_link_open(client *c, websocketpp::connection_hdl hdl)
        {
            rapidjson::Document d;
            d.SetObject();
            d.AddMember("SEQ","FUCK",d.GetAllocator());
            d.AddMember("CMD","start_sim",d.GetAllocator());
            d.AddMember("latitude",0,d.GetAllocator());
            d.AddMember("longitude",0,d.GetAllocator());
            d.AddMember("frequency",50,d.GetAllocator());
            d.AddMember("only_aircraft",1,d.GetAllocator());
            sim_connection_hdl = hdl;
            int32_t size;
            const char * str = json_to_buffer(d,size);
            c->send(hdl, str, size, websocketpp::frame::opcode::text);
    
    
            timer->async_wait([&](const boost::system::error_code &) {
                this->tick();
            });
        }
        
        void simulation_dji_a3_adapter::tick()
        {
    
            timer->expires_at(timer->expires_at() + interval);
            
            rapidjson::Document d;
            d.SetObject();
            d.SetObject();
            d.AddMember("SEQ","FUCK",d.GetAllocator());
            d.AddMember("CMD","set_status",d.GetAllocator());
            
            //TODO:Fix convention
            Eigen::Vector3d acc = Eigen::Vector3d(0,0,0);
            add_value(d,acc.x(),d,"pos0");
            add_value(d,acc.y(),d,"pos1");
            add_value(d,acc.z(),d,"pos2");
            
            Eigen::Vector3d vel = aircraft->get_ground_velocity();
            add_value(d,vel.x(),d,"vel0");
            add_value(d,vel.y(),d,"vel1");
            add_value(d,vel.z(),d,"vel2");
    
            Eigen::Vector3d w = aircraft->get_angular_velocity();
            add_value(d,w.x(),d,"w0");
            add_value(d,w.y(),d,"w1");
            add_value(d,w.z(),d,"w2");
            
            Eigen::Quaterniond quat = (Eigen::Quaterniond)aircraft->get_ground_transform().rotation();
            add_value(d,quat.w(),d,"q0");
            add_value(d,quat.x(),d,"q1");
            add_value(d,quat.y(),d,"q2");
            add_value(d,quat.z(),d,"q3");
            
            int32_t size;
            const char * str = json_to_buffer(d,size);
            c_simulator->send(sim_connection_hdl, str, size, websocketpp::frame::opcode::text);
    
            timer->async_wait([&](const boost::system::error_code &) {
                this->tick();
            });
        }
        
        void simulation_dji_a3_adapter::on_message_simulator(client *c, websocketpp::connection_hdl hdl,
                                                             message_ptr msg)
        {
//            std::cout << "on_message_simulator called with hdl: " << hdl.lock().get()
//                      << " and message: " << msg->get_payload()
//                      << std::endl;
        }
    }
}
