//
//  AppDelegate.swift
//  RapidFDM-App
//
//  Created by Hao Xu on 16/5/9.
//  Copyright (c) 2016年 xuhao. All rights reserved.
//

import Cocoa
func hander(rbuf:UnsafePointer<lcm_recv_buf_t>,
            channel:UnsafePointer<CChar>,
            msg:UnsafePointer<rapidfdmlcm_airstate>,
            user:UnsafeMutablePointer<Void>
    )->Void
{
    print("receieve");
}

@NSApplicationMain
class AppDelegate: NSObject, NSApplicationDelegate {
    
    @IBOutlet weak var window: NSWindow!
    /*
     7 my_handler(const lcm_recv_buf_t *rbuf, const char * channel,
     8         const exlcm_example_t * msg, void * user)
     9 {
     10     int i;
     11     printf("Received message on channel \"%s\":\n", channel);
     12     printf("  timestamp   = %"PRId64"\n", msg->timestamp);
     13     printf("  position    = (%f, %f, %f)\n",
     14             msg->position[0], msg->position[1], msg->position[2]);
     15     printf("  orientation = (%f, %f, %f, %f)\n",
     16             msg->orientation[0], msg->orientation[1], msg->orientation[2],
     17             msg->orientation[3]);
     18     printf("  ranges:");
     19     for(i = 0; i < msg->num_ranges; i++)
     20         printf(" %d", msg->ranges[i]);
     21     printf("\n");
     22     printf("  name        = '%s'\n", msg->name);
     23     printf("  enabled     = %d\n", msg->enabled);
     24 }
     25
     */
   
    func applicationDidFinishLaunching(aNotification: NSNotification) {
        // Insert code here to initialize your application
        print("test-lcm")
        let lcm_t = lcm_create(nil)
        if (lcm_t != nil)
        {
            print("Init lcm successful\n")
        }
        else
        {
            print("Init lcm failed\n")
        }
        var airstate = rapidfdmlcm_airstate();
        //rapid
        airstate.rho = 1.29
        airstate.wind_speed = (1,1,1)
    
        rapidfdmlcm_airstate_subscribe(lcm_t, "EXAMPLE", hander, nil)
        
        rapidfdmlcm_airstate_publish(lcm_t, "EXAMPLE", &airstate)
        
    }
    
}
