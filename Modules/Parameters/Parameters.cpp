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
 
#include "Parameters.h"
#include "EEPROM.h"
#include "Debug.h"

// Create global parameter variable in project scope
static Parameters * paramsGlobal = 0;

Parameters::Parameters(EEPROM * eeprom, LSPC * com) : eeprom_(0), com_(0), readSemaphore_(0), writeSemaphore_(0), changeCounter_(0)
{
	if (!paramsGlobal) { // first parameter object being created
		// Create global object to hold all parameters
		paramsGlobal = (Parameters *)1; // needs to set this to a value, since "new Parameters" will call the constructor again
		paramsGlobal = new Parameters;

		paramsGlobal->readSemaphore_ = xSemaphoreCreateBinary();
		if (paramsGlobal->readSemaphore_ == NULL) {
			ERROR("Could not create Parameters read semaphore");
			return;
		}
		vQueueAddToRegistry(paramsGlobal->readSemaphore_, "Parameters read");
		xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give the resource the first time

		paramsGlobal->writeSemaphore_ = xSemaphoreCreateBinary();
		if (paramsGlobal->writeSemaphore_ == NULL) {
			ERROR("Could not create Parameters write semaphore");
			return;
		}
		vQueueAddToRegistry(paramsGlobal->writeSemaphore_, "Parameters write");
		xSemaphoreGive( paramsGlobal->writeSemaphore_ ); // give the resource the first time

		if (eeprom) {
			AttachEEPROM(eeprom);
		}

		if (com) {
			AttachLSPC(com);
		}
	}

	ParametersSize = PARAMETERS_LENGTH;

	if ((uint32_t)paramsGlobal > 1) { // global object exist - load parameters from this
		Refresh(); // get parameters from global object into this
	}
}

Parameters::~Parameters()
{
	if (paramsGlobal && this == paramsGlobal) {
		/* Delete semaphores */
		if (readSemaphore_) {
			vQueueUnregisterQueue(readSemaphore_);
			vSemaphoreDelete(readSemaphore_);
		}
		if (writeSemaphore_) {
			vQueueUnregisterQueue(writeSemaphore_);
			vSemaphoreDelete(writeSemaphore_);
		}

		if (com_) {
			/* Unregister message callbacks */
			com_->unregisterCallback(lspc::MessageTypesFromPC::GetParameter);
			com_->unregisterCallback(lspc::MessageTypesFromPC::SetParameter);
			com_->unregisterCallback(lspc::MessageTypesFromPC::StoreParameters);
			com_->unregisterCallback(lspc::MessageTypesFromPC::DumpParameters);
		}
	}
}

/* This function attaches an EEPROM to the global object and loads the content */
void Parameters::AttachEEPROM(EEPROM * eeprom)
{
	if (!eeprom || !paramsGlobal || paramsGlobal->eeprom_) return;
	paramsGlobal->eeprom_ = eeprom;
	//paramsGlobal->eeprom_->EnableSection(paramsGlobal->eeprom_->sections.parameters, PARAMETERS_LENGTH);

	if (paramsGlobal->ForceDefaultParameters) {  // store default parameters into EEPROM, since forced default is enabled
		paramsGlobal->StoreParameters(); // Initialize EEPROM with default values
	}
	else { // load parameters from EEPROM and ensure they fit the current firmware (parameter size etc.)
		LoadParametersFromEEPROM(eeprom); // we load the EEPROM parameters into the non-global object initially, to verify that the parameters are valid
		if (ParametersSize == PARAMETERS_LENGTH) {
			// If the parameters are valid, we load them into the global
			paramsGlobal->LoadParametersFromEEPROM(eeprom);
		} else { // if the parameter size has changed or been reorganized we will have to reinitialize the EEPROM with default values
			paramsGlobal->StoreParameters(); // Initialize EEPROM with default values
		}
	}
}

void Parameters::AttachLSPC(LSPC * com)
{
	if (!com || !paramsGlobal || paramsGlobal->com_) return;
	paramsGlobal->com_ = com;

	/* Register message type callbacks */
	paramsGlobal->com_->registerCallback(lspc::MessageTypesFromPC::GetParameter, &GetParameter_Callback, (void *)paramsGlobal);
	paramsGlobal->com_->registerCallback(lspc::MessageTypesFromPC::SetParameter, &SetParameter_Callback, (void *)paramsGlobal);
	paramsGlobal->com_->registerCallback(lspc::MessageTypesFromPC::StoreParameters, &StoreParameters_Callback, (void *)paramsGlobal);
	paramsGlobal->com_->registerCallback(lspc::MessageTypesFromPC::DumpParameters, &DumpParameters_Callback, (void *)paramsGlobal);
}

/* Get the latest parameters from the global/master object */
void Parameters::Refresh(void)
{
	if (!paramsGlobal) return;

	ParametersSize = PARAMETERS_LENGTH;

	if (xSemaphoreTake( paramsGlobal->readSemaphore_, ( TickType_t ) 0) == pdTRUE) { // ensure we are allowed to copy the parameters without anybody changing them (corrupting it) meanwhile
		if (changeCounter_ != paramsGlobal->changeCounter_) { // only reload parameters if they have been changed
			changeCounter_ = paramsGlobal->changeCounter_;
			memcpy((uint8_t *)&ForceDefaultParameters, (uint8_t *)&paramsGlobal->ForceDefaultParameters, PARAMETERS_LENGTH); // copy global parameters into this object
		}
		xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give back the protection semaphore
	}
}

uint32_t Parameters::getParameterSizeBytes()
{
	if (!paramsGlobal) return 0;
	return PARAMETERS_LENGTH;
}

void Parameters::LockForChange(void)
{
	if (!paramsGlobal) return;
	xSemaphoreTake( paramsGlobal->writeSemaphore_, ( TickType_t ) portMAX_DELAY);
	xSemaphoreTake( paramsGlobal->readSemaphore_, ( TickType_t ) portMAX_DELAY);
	memcpy((uint8_t *)&ForceDefaultParameters, (uint8_t *)&paramsGlobal->ForceDefaultParameters, PARAMETERS_LENGTH); // load latest parameters into current object
	paramsGlobal->changeCounter_++;
	changeCounter_ = paramsGlobal->changeCounter_;
}

void Parameters::UnlockAfterChange(void)
{
	if (!paramsGlobal) return;

	memcpy((uint8_t *)&paramsGlobal->ForceDefaultParameters, (uint8_t *)&ForceDefaultParameters, PARAMETERS_LENGTH); // copy changed parameters (from current object) into global parameters object
	xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give back the protection semaphore since we are now finished with changes

 	//paramsGlobal->StoreParameters(); // store the newly update global parameters in EEPROM (if it exists)
	xSemaphoreGive( paramsGlobal->writeSemaphore_ ); // give back the EEPROM storing protection semaphore
}

void Parameters::LoadParametersFromEEPROM(EEPROM * eeprom)
{
	if (!eeprom) return; // EEPROM not configured

	/* Lock for change */
	xSemaphoreTake( paramsGlobal->writeSemaphore_, ( TickType_t ) portMAX_DELAY);
	xSemaphoreTake( paramsGlobal->readSemaphore_, ( TickType_t ) portMAX_DELAY);

	eeprom->ReadData(eeprom->sections.parameters, (uint8_t *)&ForceDefaultParameters, PARAMETERS_LENGTH);

	xSemaphoreGive( paramsGlobal->writeSemaphore_ ); // give back the EEPROM storing protection semaphore

	xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give back the protection semaphore since we are now finished with changes
	xSemaphoreGive( paramsGlobal->writeSemaphore_ ); // give back the EEPROM storing protection semaphore
}

void Parameters::StoreParameters(void)
{
	if (!eeprom_) return; // EEPROM not configured

	xSemaphoreTake( paramsGlobal->writeSemaphore_, ( TickType_t ) portMAX_DELAY);

	eeprom_->WriteData(eeprom_->sections.parameters, (uint8_t *)&ForceDefaultParameters, PARAMETERS_LENGTH);

	xSemaphoreGive( paramsGlobal->writeSemaphore_ ); // give back the EEPROM storing protection semaphore
}


void Parameters::SetParameter_Callback(void * param, const std::vector<uint8_t>& payload)
{
	Parameters * params = (Parameters *)param;
	if (!params) return;
	if (params != paramsGlobal) return;

	lspc::MessageTypesFromPC::SetParameter_t msg;
	if (payload.size() <= sizeof(msg)) return; // package is too short (missing parameter value)
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));
	uint16_t paramValueLengthBytes = payload.size() - sizeof(msg);
	void * paramValuePtr = (uint8_t *)(payload.data() + sizeof(msg));

	/* Lock for change */
	xSemaphoreTake( paramsGlobal->writeSemaphore_, ( TickType_t ) portMAX_DELAY);
	xSemaphoreTake( paramsGlobal->readSemaphore_, ( TickType_t ) portMAX_DELAY);
	paramsGlobal->changeCounter_++; // increase change counter to indicate a change

	/* Change/set the given parameter */
	bool acknowledged = false;
	void * paramPtr;
	lspc::ParameterLookup::ValueType_t valueType;
	uint8_t arraySize;
	paramsGlobal->LookupParameter(msg.type, msg.param, &paramPtr, valueType, arraySize);
	if (valueType != lspc::ParameterLookup::_unknown) {
		uint8_t copyLength = 0;
		if (valueType == lspc::ParameterLookup::_bool) copyLength = 1;
		else if (valueType == lspc::ParameterLookup::_float) copyLength = 4;
		else if (valueType == lspc::ParameterLookup::_uint8) copyLength = 1;
		else if (valueType == lspc::ParameterLookup::_uint16) copyLength = 2;
		else if (valueType == lspc::ParameterLookup::_uint32) copyLength = 4;

		// Determine and verify length
		if (arraySize == msg.arraySize && valueType == msg.valueType && arraySize*copyLength == paramValueLengthBytes) {
			// Update the parameter
			memcpy((uint8_t *)paramPtr, (uint8_t *)paramValuePtr, arraySize*copyLength);
			acknowledged = true;
		}
	}

	/* Send acknowledge response back to PC */
	lspc::MessageTypesToPC::SetParameterAck_t msgAck;
	msgAck.type = msg.type;
	msgAck.param = msg.param;
	msgAck.acknowledged = acknowledged;
	paramsGlobal->com_->TransmitAsync(lspc::MessageTypesToPC::SetParameterAck, (uint8_t *)&msgAck, sizeof(msgAck));

	/* Unlock after change */
	xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give back the protection semaphore since we are now finished with changes
	xSemaphoreGive( paramsGlobal->writeSemaphore_ ); // give back the EEPROM storing protection semaphore
}

void Parameters::GetParameter_Callback(void * param, const std::vector<uint8_t>& payload)
{
	Parameters * params = (Parameters *)param;
	if (!params) return;
	if (params != paramsGlobal) return;

	lspc::MessageTypesFromPC::GetParameter_t msg;
	if (payload.size() != sizeof(msg)) return;
	memcpy((uint8_t *)&msg, payload.data(), sizeof(msg));

	/* Lock for reading */
	xSemaphoreTake( paramsGlobal->readSemaphore_, ( TickType_t ) portMAX_DELAY);

	/* Change/set the given parameter */
	void * paramPtr;
	lspc::ParameterLookup::ValueType_t valueType;
	uint8_t arraySize;
	paramsGlobal->LookupParameter(msg.type, msg.param, &paramPtr, valueType, arraySize);
	if (valueType != lspc::ParameterLookup::_unknown) {
		// Read parameter and send response
		uint8_t copyLength = 0;
		if (valueType == lspc::ParameterLookup::_bool) copyLength = 1;
		else if (valueType == lspc::ParameterLookup::_float) copyLength = 4;
		else if (valueType == lspc::ParameterLookup::_uint8) copyLength = 1;
		else if (valueType == lspc::ParameterLookup::_uint16) copyLength = 2;
		else if (valueType == lspc::ParameterLookup::_uint32) copyLength = 4;

		lspc::MessageTypesToPC::GetParameter_t response;
		response.type = msg.type;
		response.param = msg.param;
		response.valueType = valueType;
		response.arraySize = arraySize;

		uint16_t responseLengthBytes = sizeof(response) + copyLength*arraySize;
		uint8_t * msgBuf = (uint8_t *)pvPortMalloc(responseLengthBytes);
		if (msgBuf) {
			memcpy(msgBuf, &response, sizeof(response));
			memcpy(&msgBuf[sizeof(response)], (uint8_t *)paramPtr, arraySize*copyLength);
			paramsGlobal->com_->TransmitAsync(lspc::MessageTypesToPC::GetParameter, msgBuf, responseLengthBytes);
			vPortFree(msgBuf);
		}
	}
	else { // send unknown parameter response
		lspc::MessageTypesToPC::GetParameter_t response;
		response.type = lspc::ParameterLookup::unknown;
		response.param = 0;
		response.valueType = lspc::ParameterLookup::_unknown;
		response.arraySize = 0;
		paramsGlobal->com_->TransmitAsync(lspc::MessageTypesToPC::GetParameter, (uint8_t *)&response, sizeof(response));
	}

	/* Unlock after reading */
	xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give back the read protection semaphore
}

void Parameters::StoreParameters_Callback(void * param, const std::vector<uint8_t>& payload)
{
	Parameters * params = (Parameters *)param;
	if (!params) return;
	if (params != paramsGlobal) return;

	lspc::MessageTypesToPC::StoreParametersAck_t msgAck;

	/* Lock for change */
	xSemaphoreTake( paramsGlobal->writeSemaphore_, ( TickType_t ) portMAX_DELAY);
	xSemaphoreTake( paramsGlobal->readSemaphore_, ( TickType_t ) portMAX_DELAY);

	/* Store the parameters into EEPROM */
	if (paramsGlobal->eeprom_) {
		if (paramsGlobal->eeprom_->WriteData(paramsGlobal->eeprom_->sections.parameters, (uint8_t *)&paramsGlobal->ForceDefaultParameters, PARAMETERS_LENGTH) == EEPROM::EEPROM_FLASH_COMPLETE)
			msgAck.acknowledged = true;
		else
			msgAck.acknowledged = false;
	} else {
		msgAck.acknowledged = false; // EEPROM not configured
	}

	/* Unlock after change */
	xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give back the protection semaphore since we are now finished with changes
	xSemaphoreGive( paramsGlobal->writeSemaphore_ ); // give back the EEPROM storing protection semaphore

	/* Send acknowledge to PC */
	paramsGlobal->com_->TransmitAsync(lspc::MessageTypesToPC::StoreParametersAck, (uint8_t *)&msgAck, sizeof(msgAck));
}

void Parameters::DumpParameters_Callback(void * param, const std::vector<uint8_t>& payload)
{
	Parameters * params = (Parameters *)param;
	if (!params) return;
	if (params != paramsGlobal) return;

	/* Lock for reading */
	xSemaphoreTake( paramsGlobal->readSemaphore_, ( TickType_t ) portMAX_DELAY);

	/* Transmit first package to PC indicating parameter length, and hence how many packages that will be sent */
	lspc::MessageTypesToPC::DumpParameters_t msg;
	msg.parameters_size_bytes = PARAMETERS_LENGTH;
	msg.packages_to_follow = (msg.parameters_size_bytes / LSPC_MAXIMUM_PACKAGE_LENGTH) + ((msg.parameters_size_bytes % LSPC_MAXIMUM_PACKAGE_LENGTH) > 0);
	paramsGlobal->com_->TransmitAsync(lspc::MessageTypesToPC::DumpParameters, (uint8_t *)&msg, sizeof(msg));

	/* Send parameter dump to PC */
	uint16_t LeftToTransmit = PARAMETERS_LENGTH;
	uint16_t TransmitLength;
	uint8_t * paramPtr =  (uint8_t *)&paramsGlobal->ForceDefaultParameters;
	while (LeftToTransmit > 0) {
		TransmitLength = LeftToTransmit;
		if (TransmitLength > LSPC_MAXIMUM_PACKAGE_LENGTH) TransmitLength = LSPC_MAXIMUM_PACKAGE_LENGTH;

		paramsGlobal->com_->TransmitAsync(lspc::MessageTypesToPC::DumpParameters, paramPtr, TransmitLength);
		LeftToTransmit -= TransmitLength;
		paramPtr += TransmitLength;
	}

	/* Unlock after reading */
	xSemaphoreGive( paramsGlobal->readSemaphore_ ); // give back the read protection semaphore
}

void Parameters::LookupParameter(uint8_t type, uint8_t param, void ** paramPtr, lspc::ParameterLookup::ValueType_t& valueType, uint8_t& arraySize)
{
	valueType = lspc::ParameterLookup::_unknown;
	*paramPtr = (void *)0;
	arraySize = 1; // arrays not supported yet

	if (type == lspc::ParameterLookup::debug) {
		switch (param) {
			case lspc::ParameterLookup::EnableDumpMessages: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->debug.EnableDumpMessages; return;
			case lspc::ParameterLookup::EnableRawSensorOutput: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->debug.EnableRawSensorOutput; return;
			case lspc::ParameterLookup::UseFilteredIMUinRawSensorOutput: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->debug.UseFilteredIMUinRawSensorOutput; return;
			case lspc::ParameterLookup::DisableMotorOutput: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->debug.DisableMotorOutput; return;
			default: return;
		}
	}
	else if (type == lspc::ParameterLookup::test) {
		switch (param) {
			case lspc::ParameterLookup::tmp: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->test.tmp; return;
			case lspc::ParameterLookup::tmp2: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->test.tmp2; return;
			default: return;
		}
	}
	else if (type == lspc::ParameterLookup::behavioural) {
		switch (param) {
			case lspc::ParameterLookup::IndependentHeading: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->behavioural.IndependentHeading; return;
			case lspc::ParameterLookup::YawVelocityBraking: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->behavioural.YawVelocityBraking; return;
			case lspc::ParameterLookup::StepTestEnabled: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->behavioural.StepTestEnabled; return;
			case lspc::ParameterLookup::SineTestEnabled: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->behavioural.SineTestEnabled; return;
			case lspc::ParameterLookup::CircleTestEnabled: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->behavioural.CircleTestEnabled; return;
			case lspc::ParameterLookup::PowerButtonMode: valueType = lspc::ParameterLookup::_uint8; *paramPtr = (void *)&this->behavioural.PowerButtonMode; return;
			default: return;
		}
	}
	else if (type == lspc::ParameterLookup::controller) {
		switch (param) {
			case lspc::ParameterLookup::ControllerSampleRate: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.SampleRate; return;
			case lspc::ParameterLookup::mode: valueType = lspc::ParameterLookup::_uint8; *paramPtr = (void *)&this->controller.mode; return;
			case lspc::ParameterLookup::type: valueType = lspc::ParameterLookup::_uint8; *paramPtr = (void *)&this->controller.type; return;
			case lspc::ParameterLookup::EnableTorqueLPF: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->controller.EnableTorqueLPF; return;
			case lspc::ParameterLookup::MotorFailureDetection: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->controller.MotorFailureDetection; return;
			case lspc::ParameterLookup::DisableQdot: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->controller.DisableQdot; return;
			case lspc::ParameterLookup::DisableQdotInEquivalentControl: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->controller.DisableQdotInEquivalentControl; return;
			case lspc::ParameterLookup::DisableOmegaXYInEquivalentControl: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->controller.DisableOmegaXYInEquivalentControl; return;
			case lspc::ParameterLookup::ManifoldType: valueType = lspc::ParameterLookup::_uint8; *paramPtr = (void *)&this->controller.ManifoldType; return;
			case lspc::ParameterLookup::ContinousSwitching: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->controller.ContinousSwitching; return;
			case lspc::ParameterLookup::EquivalentControl: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->controller.EquivalentControl; return;
			case lspc::ParameterLookup::eta: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.eta[0]; arraySize = 3;  return;
			case lspc::ParameterLookup::epsilon: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.epsilon[0]; arraySize = 3; return;
			case lspc::ParameterLookup::K: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.K[0]; arraySize = 3; return;
			case lspc::ParameterLookup::Kx: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.K[0]; return;
			case lspc::ParameterLookup::Ky: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.K[1]; return;
			case lspc::ParameterLookup::Kz: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.K[2]; return;
			case lspc::ParameterLookup::Kv_x: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.Kv[0]; return;
			case lspc::ParameterLookup::Kv_y: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.Kv[1]; return;
			case lspc::ParameterLookup::Kvi_x: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.Kvi[0]; return;
			case lspc::ParameterLookup::Kvi_y: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.Kvi[1]; return;
			case lspc::ParameterLookup::gamma: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.gamma; return;
			case lspc::ParameterLookup::VelocityControl_AccelerationLimit: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.VelocityControl_AccelerationLimit; return;
			case lspc::ParameterLookup::VelocityControl_UseOmegaRef: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->controller.VelocityControl_UseOmegaRef; return;
			case lspc::ParameterLookup::VelocityController_MaxTilt: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.VelocityController_MaxTilt; return;
			case lspc::ParameterLookup::VelocityController_MaxIntegralCorrection: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.VelocityController_MaxIntegralCorrection; return;
			case lspc::ParameterLookup::VelocityController_VelocityClamp: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.VelocityController_VelocityClamp; return;
			case lspc::ParameterLookup::VelocityController_IntegralGain: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.VelocityController_IntegralGain; return;
			case lspc::ParameterLookup::VelocityController_AngleLPFtau: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.VelocityController_AngleLPFtau; return;
			case lspc::ParameterLookup::VelocityController_OmegaLPFtau: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.VelocityController_OmegaLPFtau; return;
			case lspc::ParameterLookup::AngularVelocityClampsEnabled: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->controller.AngularVelocityClampsEnabled; return;
			case lspc::ParameterLookup::AngularVelocityClamps: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->controller.AngularVelocityClamps[0]; arraySize = 3; return;
			default: return;
		}
	}
	else if (type == lspc::ParameterLookup::estimator) {
		switch (param) {
			case lspc::ParameterLookup::EstimatorSampleRate: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->estimator.SampleRate; return;
			case lspc::ParameterLookup::EnableSensorLPFfilters: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.EnableSensorLPFfilters; return;
			case lspc::ParameterLookup::EnableSoftwareLPFfilters: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.EnableSoftwareLPFfilters; return;
			case lspc::ParameterLookup::CreateQdotFromQDifference: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.CreateQdotFromQDifference; return;
			case lspc::ParameterLookup::UseMadgwick: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.UseMadgwick; return;
			case lspc::ParameterLookup::SensorDrivenQEKF: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.SensorDrivenQEKF; return;
			case lspc::ParameterLookup::UseCoRvelocity: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.UseCoRvelocity; return;
			case lspc::ParameterLookup::UseVelocityEstimator: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.UseVelocityEstimator; return;
			case lspc::ParameterLookup::UseQdotInVelocityEstimator: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.UseQdotInVelocityEstimator; return;
			case lspc::ParameterLookup::EstimateCOM: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.EstimateCOM; return;
			case lspc::ParameterLookup::EstimateBias: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.EstimateBias; return;
			case lspc::ParameterLookup::EnableVelocityLPF: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.EnableVelocityLPF; return;
			case lspc::ParameterLookup::EnableWheelSlipDetector: valueType = lspc::ParameterLookup::_bool; *paramPtr = (void *)&this->estimator.EnableWheelSlipDetector; return;
			case lspc::ParameterLookup::sigma2_bias: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->estimator.sigma2_bias; return;
			case lspc::ParameterLookup::sigma2_omega: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->estimator.sigma2_omega; return;
			case lspc::ParameterLookup::sigma2_heading: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->estimator.sigma2_heading; return;
			case lspc::ParameterLookup::GyroscopeTrustFactor: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->estimator.GyroscopeTrustFactor; return;
			case lspc::ParameterLookup::eta_encoder: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->estimator.eta_encoder; return;
			case lspc::ParameterLookup::eta_accelerometer: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->estimator.eta_accelerometer; return;
			case lspc::ParameterLookup::var_acc_bias: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->estimator.var_acc_bias; return;
			case lspc::ParameterLookup::var_acceleration: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->estimator.var_acceleration; return;
			default: return;
		}
	}
	else if (type == lspc::ParameterLookup::model) {
		switch (param) {
			case lspc::ParameterLookup::l: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->model.l; return;
			case lspc::ParameterLookup::CoR: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->model.CoR; return;
			case lspc::ParameterLookup::Mk: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->model.Mk; return;
			case lspc::ParameterLookup::Mb: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->model.Mb; return;
			case lspc::ParameterLookup::Bvk: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->model.Bvk; return;
			case lspc::ParameterLookup::Bvm: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->model.Bvm; return;
			case lspc::ParameterLookup::Bvb: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->model.Bvb; return;
			case lspc::ParameterLookup::SaturationTorqueOfMaxOutputTorque: valueType = lspc::ParameterLookup::_float; *paramPtr = (void *)&this->model.SaturationTorqueOfMaxOutputTorque; return;
			default: return;
		}
	}
}


/*
void Parameters::StoreThread(void)
{
	// Make a thread in the global pararameter object which checks for changes and writes to the EEPROM (eg. checking every 10 seconds)
}
*/

#if 0
Parameters& Parameters::Get()
{
	if (!paramsGlobal) // first time initializes the global parameters objects
		paramsGlobal = new Parameters;

	return *paramsGlobal;
}

Parameters& Parameters::Get(EEPROM * eeprom)
{
	if (!eeprom) return Get();

	if (!paramsGlobal) { // first time initializes the global parameters objects
		paramsGlobal = new Parameters;
		if (paramsGlobal->ForceDefaultParameters)
			return *paramsGlobal; // do not proceed in loading EEPROM parameters, since forced default is enabled

		eeprom->EnableSection(eeprom->sections.parameters, PARAMETERS_LENGTH);
		paramsGlobal->LoadParametersFromEEPROM(eeprom);
		if (paramsGlobal->ParametersSize != PARAMETERS_LENGTH) { // ensure that parameter size has not changed/been reorganized, as we will then have to reinitialize the EEPROM with default values
			delete(paramsGlobal);
			paramsGlobal = new Parameters; // this loads the default parameters

			paramsGlobal->eeprom_ = eeprom;
			paramsGlobal->ParametersSize = PARAMETERS_LENGTH;
			paramsGlobal->StoreParameters(); // Initialize EEPROM with default values
		}
	}

	return *paramsGlobal;
}
#endif
