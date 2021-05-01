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
 
#ifndef MODULES_MODULETEMPLATE_H
#define MODULES_MODULETEMPLATE_H

#include "cmsis_os.h" // for processing task

class ModuleTemplate
{
	private:
		const uint32_t MODULE_TEMPLATE_THREAD_STACK = 256;

	public:
		ModuleTemplate(uint32_t moduleTaskPriority);
		~ModuleTemplate();

	private:
		TaskHandle_t _moduleTaskHandle;

	private:
		static void ModuleTemplateThread(void * pvParameters);
		
};
	
	
#endif
