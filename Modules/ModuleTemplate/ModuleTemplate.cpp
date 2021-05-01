/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "ModuleTemplate.hpp"
#include "cmsis_os.h" // for processing task

#include <Debug/Debug.h>

ModuleTemplate::ModuleTemplate(uint32_t moduleTaskPriority)
    : _moduleTaskHandle(0)
{
    xTaskCreate(ModuleTemplate::ModuleTemplateThread, (char*)"Module Template", MODULE_TEMPLATE_THREAD_STACK,
                (void*)this, moduleTaskPriority, &_moduleTaskHandle);
}

ModuleTemplate::~ModuleTemplate()
{
    if (_moduleTaskHandle)
        vTaskDelete(_moduleTaskHandle); // stop task
}

void ModuleTemplate::ModuleTemplateThread(void* pvParameters)
{
    ModuleTemplate* module = (ModuleTemplate*)pvParameters;

    while (1) {
        osDelay(1000);
    }
}
