//
// Simple example to demonstrate how to use the Dronecode SDK.
//
// Author: Julian Oes <julian@oes.ch>

#include <iostream>
#include <chrono>
#include <ctime>
#include <cstdint>
#include <thread>
#include <stdlib.h> /* abs */
#include <stdio.h> /* printf */
#include <math.h> /* sin */

#include <dronecode_sdk/dronecode_sdk.h>
#include <dronecode_sdk/plugins/action/action.h>
#include <dronecode_sdk/plugins/telemetry/telemetry.h>
#include <dronecode_sdk/plugins/offboard/offboard.h>

using namespace dronecode_sdk;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour


//Threshold Box: center area in Image Frame
#define X_THRESHOLD 5
#define Y_THRESHOLD 5


void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

int sgn(double v)
{
    if (v < 0) return -1;
    if (v > 0) return 1;
    return 0;
}

//MAIN

int main(int argc, char **argv)
{
    DronecodeSDK        dc;
    std::string         connection_url;
    ConnectionResult    connection_result;

    //------------------------------<Connect to a Vehicle>------------------------------//
    //----------------------------------------------------------------------------------//
    //bool discovered_system;
    if (argc == 2)
    {
        connection_url      =       argv[1];
        connection_result   =       dc.add_any_connection(connection_url);
    }
    else
    {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::SUCCESS)
    {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Connection failed: " << connection_result_str(connection_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    //added
    // Wait for the system to connect via heartbeat
    while (!dc.is_connected()) {
        std::cout << "Wait for system to connect via heartbeat" << std::endl;
        sleep_for(seconds(1));
    }

    // System got discovered.
    System &system =    dc.system();

//    std::cout << "Waiting to discover system..." << std::endl;
//    dc.register_on_discover([&discovered_system](uint64_t uuid) {
//        std::cout << "Discovered system with UUID: " << uuid << std::endl;
//        discovered_system = true;
//    });

//    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
//    // seconds.
//    sleep_for(seconds(2));

//    if (!discovered_system) {
//        std::cout << ERROR_CONSOLE_TEXT << "No system found, exiting." << NORMAL_CONSOLE_TEXT
//                  << std::endl;
//        return 1;
//    }


    //----------------------------------------------------------------------------------//
    //----------------------------------------------------------------------------------//

    //-------------------------<Monitor Important Variables>----------------------------//

    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);


    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Setting rate failed:" << Telemetry::result_str(set_rate_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry->position_async([](Telemetry::Position position) {
        std::cout << TELEMETRY_CONSOLE_TEXT // set to blue
                  << "Altitude: " << position.relative_altitude_m << " m"
                  << NORMAL_CONSOLE_TEXT // set to default color again
                  << std::endl;
    });

    //----------------------------------------------------------------------------------//
    //----------------------------------------------------------------------------------//

    //--------------------------------<Arm and Takeoff>---------------------------------//
    // Check if vehicle is ready to arm
    while (telemetry->health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "System is ready" << std::endl;

    // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action->arm();

    if (arm_result != Action::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << Action::result_str(arm_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }
    std::cout << "Armed" << std::endl;

    // Take off
    std::cout << "Taking off..." << std::endl;
    const Action::Result takeoff_result = action->takeoff();
    if (takeoff_result != Action::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Takeoff failed:" << Action::result_str(takeoff_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }
    std::cout << "In Air..." << std::endl;

    // Let it hover for a bit before landing again.
    sleep_for(seconds(5));
    //----------------------------------------------------------------------------------//
    //----------------------------------------------------------------------------------//

    //------------------------------<Variables of Interest>-----------------------------//

    float xTargetCenterInImageFrame;      // Given by detection algorithm
    float yTargetCenterInImageFrame;      // Given by detection algorithm
    bool detected = true;                  // Given by detection algorithm
    bool net = true;                       // Given by listening on SERVO_OUTPUT_RAW message if Servo is plugged on PXH AND listening on the MAV_FRAME parameter (Quadcopter / Hexacopter)
    //bool track = false;                     // Given by Task control
    float l1;                         // tuning parameter
    float l2;                         //
    float n;
    float e;
    float d;
    //float dronecode_sdk::Telemetry::EulerAngle::yaw_deg;
    const double pi = M_PI;
    Offboard::Result offboard_result;
    float phi;

    //----------------------------------------------------------------------------------//
    //----------------------------------------------------------------------------------//
    //-------------------------------------<Algorithm>----------------------------------//

    //TO MODIFY

    // Start offboard mode.
    offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 0.0f});
    offboard_result = offboard->start();
    if (offboard_result != Offboard::Result::SUCCESS) {
        std::cerr << "Offboard::start() failed: "
        << Offboard::result_str(offboard_result) << std::endl;
        }
    if (offboard_result == Offboard::Result::SUCCESS) {
        std::cerr << "Offboard::start() success: "
        << Offboard::result_str(offboard_result) << std::endl;
        }

    auto start_time = std::chrono::system_clock::now();  // start chrono

    while (offboard_result == Offboard::Result::SUCCESS)
    {
        auto actual_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = actual_time-start_time;

        std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n\n";

        if (elapsed_seconds.count() < 35)  //faire x=x-v*t
        {
            //xTargetCenterInImageFrame = -10.0f+(elapsed_seconds.count());      // Dynamic input for testing
            //yTargetCenterInImageFrame = -3.0f+(elapsed_seconds.count());
            xTargetCenterInImageFrame = 10.0f-(elapsed_seconds.count());      // Dynamic input for testing
            yTargetCenterInImageFrame = 0.0f ;//-3.0f+(elapsed_seconds.count());
            std::cout << "x relative: " << xTargetCenterInImageFrame << "\n";
            std::cout << "y relative: " << yTargetCenterInImageFrame << "\n";
        }
        else
        {
            xTargetCenterInImageFrame = 0;      // After a time: go tracking
            yTargetCenterInImageFrame = 0;
        }


        if(detected)
        {

            std::cout << "Target Detected:" << std::endl;
            phi = telemetry->attitude_euler_angle().yaw_deg;
            std::cout << "Yaw angle: " << phi << "s\n";

            if (!net)
            {
                std::cout << "No net, avoid" << std::endl;
                l1=-sgn(xTargetCenterInImageFrame)*2;   // to be tuned
                l2=sgn(yTargetCenterInImageFrame)*10;
                n = -l1*sin(phi*pi/180);
                e = l1*cos(phi*pi/180);
                d = l2;
                offboard->set_velocity_ned({n, e, d, 0.0f});
                sleep_for(seconds(2));
//                std::cout << "Turn to face East" << std::endl;
//                offboard->set_velocity_ned({0.0f, 0.0f, 0.0f, 90.0f});
//                sleep_for(seconds(1)); // Let yaw settle.

//                {
//                    const float step_size = 0.01f;
//                    const float one_cycle = 2.0f * (float)M_PI;
//                    const unsigned steps = 2 * unsigned(one_cycle / step_size);

//                    std::cout << "Go North and back South" << std::endl;
//                    for (unsigned i = 0; i < steps; ++i) {
//                        float vx = 5.0f * sinf(i * step_size);
//                        offboard->set_velocity_ned({vx, 0.0f, 0.0f, 90.0f});
//                        sleep_for(milliseconds(10));
//                    }
//                }
            }




            // DON'T FORGET THE NET DISTANCE !!!!!


            else if (abs(xTargetCenterInImageFrame) < X_THRESHOLD && abs(yTargetCenterInImageFrame) < Y_THRESHOLD)
            {
                std::cout << "Track, target within center area of Image Frame" << std::endl;
                n = 0;
                e = 0;
                d = 0;
                offboard->set_velocity_ned({n, e, d, 0.0f});    // ici plutot set_velocity_body non???
                sleep_for(seconds(2));


                // ici plutot set_velocity_body non???
                //n = -l1*sin(phi*pi/180);
                //e = l1*cos(phi*pi/180);
                //d = l2;
            }
            else if (abs(xTargetCenterInImageFrame) > X_THRESHOLD && abs(yTargetCenterInImageFrame) > Y_THRESHOLD)
            {
                std::cout << "Track, target in one of the corners of the Image Frame" << std::endl;
                l1=sgn(xTargetCenterInImageFrame)*2;   // to be tuned
                l2=-sgn(yTargetCenterInImageFrame)*10;
                n = -l1*sin(phi*pi/180);
                e = l1*cos(phi*pi/180);
                d = l2;
                offboard->set_velocity_ned({n, e, d, 0.0f});
                sleep_for(seconds(2));
            }
            else if (abs(xTargetCenterInImageFrame) > X_THRESHOLD && abs(yTargetCenterInImageFrame) < Y_THRESHOLD)
            {
                std::cout << "Track, target is Y-centered in the Image Frame" << std::endl;
                l1=sgn(xTargetCenterInImageFrame)*2;   // to be tuned
                n = -l1*sin(phi*pi/180);
                e = l1*cos(phi*pi/180);
                d = 0;
                offboard->set_velocity_ned({n, e, d, 0.0f});
                sleep_for(seconds(2));
            }
            else
            {
                std::cout << "Track, target is X-centered in the Image Frame" << std::endl;
                l2=-sgn(yTargetCenterInImageFrame)*10;   // to be tuned
                n = 0;
                e = 0;
                d = l2;
                offboard->set_velocity_ned({n, e, d, 0.0f});
                sleep_for(seconds(2));
            }
            offboard->set_velocity_ned({50.0f, 10.0f, 0.0f, 40.0f});
            // a changer
        }
        //sleep_for(seconds(4));
        //faire un set general ici?
    }

    offboard->set_velocity_ned({n, e, d, 45.0f});

    //Stop offboard mode
    offboard_result = offboard->stop();
    if (offboard_result != Offboard::Result::SUCCESS) {
            std::cerr << "Offboard::stop() failed: "
            << Offboard::result_str(offboard_result) << std::endl;
        }


    //----------------------------------------------------------------------------------//
    //----------------------------------------------------------------------------------//

    //--------------------------------<Land and disarm>---------------------------------//

    std::cout << "Landing..." << std::endl;
    const Action::Result land_result = action->land();
    if (land_result != Action::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Land failed:" << Action::result_str(land_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    return EXIT_SUCCESS;
}
