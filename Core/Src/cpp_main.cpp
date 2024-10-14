#include "cpp_main.h"

#include "lwip.h"
#include "eth_smbus.h"

#include "app/can_smbus/can_smbus.hpp"
#include "app/odrive.hpp"
#include "app/robomas.hpp"
#include <vector>


void cpp_main(void)
{
    static smbus::Robomas robomas1(ESPort_1);
    static smbus::CanSMBus can_smbus1(ESPort_1);
    // static smbus::Robomas robomas2(ESPort_2);
	// static smbus::CanSMBus can_smbus2(ESPort_2);
    static smbus::Odrive odrive2(ESPort_2);

    std::array<smbus::AppBase*, 3>  apps = {
        &robomas1,
        &can_smbus1,
		// &robomas2,
        // &can_smbus2
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
