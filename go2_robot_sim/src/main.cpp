/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <memory>

// ROS 2 specific headers
#include "rclcpp/rclcpp.hpp"

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"

#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h" // Update the IOROS header to ROS 2
#endif // COMPILE_WITH_ROS

std::shared_ptr<rclcpp::Node> node;
bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
    rclcpp::shutdown();
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("unitree_controller");

    /* set real-time process */
    setProcessScheduler();

    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;

#ifdef COMPILE_WITH_SIMULATION
    ioInter = new IOROS(node); // Pass the node handle to the constructor
    ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_SIMULATION

#ifdef COMPILE_WITH_REAL_ROBOT
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002; // run at 500hz
    ctrlComp->running = &running;

    // ... other initialization ...

    ctrlComp->geneObj();

    ControlFrame ctrlFrame(ctrlComp);

    signal(SIGINT, ShutDown);

    while (rclcpp::ok() && running)
    {
        ctrlFrame.run();
        rclcpp::spin_some(node);
    }

    delete ctrlComp;
    return 0;
}
