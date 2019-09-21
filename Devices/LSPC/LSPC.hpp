#ifndef LSPC_TEMPLATED_HPP
#define LSPC_TEMPLATED_HPP

#include "Packet.hpp"
#include "Serializable.hpp"
#include "SocketBase.hpp"
#include "Debug.h"
#include "cmsis_os.h" // for task creation
#include "USBCDC.h"
#include "UART.h"
#include "MessageTypes.h"

#define LSPC_MAX_ASYNCHRONOUS_PACKAGE_SIZE			100  // bytes
#define LSPC_MAXIMUM_PACKAGE_LENGTH					250
#define LSPC_ASYNCHRONOUS_QUEUE_LENGTH				100   // maximum 100 asynchronous packages in queue
#define LSPC_RX_PROCESSING_THREAD_STACK_SIZE		1024
#define LSPC_TX_TRANSMITTER_THREAD_STACK_SIZE		512

namespace lspc
{

typedef struct LSPC_Async_Package_t {
	uint8_t type;
	std::vector<uint8_t> * payloadPtr;
} LSPC_Async_Package_t;

template <class COM>
class Socket : public SocketBase
{
public:
  Socket(COM * com, uint32_t processingTaskPriority, uint32_t transmitterTaskPriority) : com(com), _processingTaskHandle(0), _transmitterTaskHandle(0)
  {
		_TXqueue = xQueueCreate( LSPC_ASYNCHRONOUS_QUEUE_LENGTH, sizeof(LSPC_Async_Package_t) );
		if (_TXqueue == NULL) {
			ERROR("Could not create asynchronous LSPC TX queue");
			return;
		}
		vQueueAddToRegistry(_TXqueue, "LSPC TX");

		xTaskCreate(Socket::ProcessingThread, (char *)"LSPC processing", LSPC_RX_PROCESSING_THREAD_STACK_SIZE, (void*) this, processingTaskPriority, &_processingTaskHandle);
		xTaskCreate(Socket::TransmitterThread, (char *)"LSPC transmitter", LSPC_TX_TRANSMITTER_THREAD_STACK_SIZE, (void*) this, transmitterTaskPriority, &_transmitterTaskHandle);
  };

private:
  using SocketBase::send;

  // Send a package with lspc
  //
  // @brief Sends a packaged buffer over the USB serial link.
  //
  // @param type The message type. This is user specific; any type between 1-255.
  // @param payload A vector with the serialized payload to be sent.
  //
  // @return True if the packet was sent.
  bool send(uint8_t type, const std::vector<uint8_t> &payload) override
  {
    Packet outPacket(type, payload);

    // Send it
    if (outPacket.encodedDataSize() ==
    		com->Write(outPacket.encodedDataPtr(), outPacket.encodedDataSize()))
      return true;
    else
      return false;
  };


  // Process incoming data on serial link
  //
  // @brief Reads the serial buffer and dispatches the received payload to the
  // relevant message handling callback function.
  void processSerial()
  {
	int16_t readChar = 0;
	while (com->Available())
	{
		readChar = com->Read();
		if (readChar >= 0)
			processIncomingByte(readChar);
	}
	return;
  };


public:
  void TransmitAsync(uint8_t type, const uint8_t * payload, uint16_t payloadLength)
  {
	  LSPC_Async_Package_t package;
	  if (payloadLength > LSPC_MAXIMUM_PACKAGE_LENGTH) return; // payload size is too big
	  if (uxQueueSpacesAvailable(_TXqueue) == 0) {
		  return; // no space in queue
	  }

	  package.type = type;
	  package.payloadPtr = new std::vector<uint8_t>(payloadLength);
	  if (!package.payloadPtr) return;
	  memcpy(package.payloadPtr->data(), payload, payloadLength);
	  if (xQueueSend(_TXqueue, (void *)&package, (TickType_t) 0) != pdTRUE) {
		  delete(package.payloadPtr); // could not add package to queue, probably because it is full
	  }
  }

  bool Connected(void)
  {
	  return com->Connected();
  }


private:
  static void ProcessingThread(void * pvParameters)
  {
  	Socket<COM> * lspc = (Socket<COM> *)pvParameters;

  	lspc->incoming_data.reserve(LSPC_MAXIMUM_PACKAGE_LENGTH);

	// LSPC incoming data processing loop
	while (1)
	{
		if (lspc->com->WaitForNewData(portMAX_DELAY))
			lspc->processSerial();
	}
  }

  static void TransmitterThread(void * pvParameters)
  {
	  Socket<COM> * lspc = (Socket<COM> *)pvParameters;
	  LSPC_Async_Package_t package;

	  while (1)
	  {
		  if (lspc->Connected()) {
			  // LSPC outgoing (transmission) data loop
			  while (lspc->Connected())
			  {
					if ( xQueueReceive( lspc->_TXqueue, &package, ( TickType_t ) portMAX_DELAY ) == pdPASS ) {
						// Send it if possible
						Packet * outPacket = new Packet(package.type, *package.payloadPtr);
						if (!outPacket) continue;

						if (outPacket->encodedDataSize() ==
								lspc->com->WriteBlocking(outPacket->encodedDataPtr(), outPacket->encodedDataSize())) {
							delete(package.payloadPtr); // clear memory used for payload data
						}
						else { // if not, re-add it to the queue
							//xQueueSend(lspc->_TXqueue, (void *)&package, (TickType_t) 1); // re-add it to the queue is probably not a good idea
							delete(package.payloadPtr); // clear memory used for payload data
						}
						delete(outPacket);
					}
			  }
		  }
		  osDelay(100);
	  }
  }

public:
  COM * com;

private:
  TaskHandle_t _processingTaskHandle;
  TaskHandle_t _transmitterTaskHandle;
  SemaphoreHandle_t _newTransmitDataSemaphore;
  QueueHandle_t _TXqueue;

};

} // namespace lspc

using LSPC = lspc::Socket<USBCDC>; // define whether to use USB or UART

#endif // LSPC_TEMPLATED_HPP
