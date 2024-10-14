/*
 * cpp_main.cpp
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#include "cpp_main.h"

#include "lwip.h"
#include "eth_csmbus.h"

#include "app/can_csmbus/can_csmbus.hpp"
#include "app/odrive.hpp"
#include "app/robomas.hpp"
#include <vector>


void cpp_main(void)
{
    static csmbus::Robomas robomas1(ESPort_1);
    static csmbus::CanSMBus can_csmbus1(ESPort_1);
    // static csmbus::Robomas robomas2(ESPort_2);
	// static csmbus::CanSMBus can_csmbus2(ESPort_2);
    static csmbus::Odrive odrive2(ESPort_2);

    std::array<csmbus::AppBase*, 3>  apps = {
        &robomas1,
        &can_csmbus1,
		// &robomas2,
        // &can_csmbus2
        &odrive2,
    };

    for(size_t i = 0; i < apps.size(); i++)
    {
        apps[i]->init();
    }

    // ESBackdoor_enablePanel(ESPort_1, CSId_8);

    while (1)
    {
        for(size_t i = 0; i < apps.size(); i++)
        {
            apps[i]->process();
        }

        ESType_bool_t is_safety_on = ESCtrl_isSafetyOn();
        MX_LWIP_Process();
        ESId_process(is_safety_on);
        ESBackdoor_process(is_safety_on);
        ESCtrl_process();
        ESCan_process();
        ESLed_process(is_safety_on);
    }
}
