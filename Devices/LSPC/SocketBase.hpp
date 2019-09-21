#ifndef LSPC_SOCKETBASE_HPP
#define LSPC_SOCKETBASE_HPP

#include "Packet.hpp"
#include "Serializable.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <vector>

namespace lspc
{

class SocketBase
{
  // Members for receiving data on the serial link.
  uint8_t incoming_length = 0;

  // FSM for receiving
  enum class LookingFor {header, type, length, data};
  LookingFor fsr_state = LookingFor::header;

  // Map of callback functions to handle the incoming messages.
  typedef struct callback_t {
	  void (*handler)(void * param, const std::vector<uint8_t>&);
	  void * param;
  } callback_t;
  std::map<uint8_t, callback_t> type_handlers;

public:
  // Made public such that it can be initialized (reserve memory) during construction
  std::vector<uint8_t> incoming_data;

protected:
  void processIncomingByte(uint8_t incoming_byte)
  {
	switch (fsr_state)
	{
	  case LookingFor::header:
		if (incoming_byte == 0x00)
		{
		  incoming_data.push_back(incoming_byte);
		  fsr_state = LookingFor::type;
		}
		break;
	  case LookingFor::type:
		if (incoming_byte != 0x00)
		{
		  incoming_data.push_back(incoming_byte);
		  fsr_state = LookingFor::length;
		}
		break;
	  case LookingFor::length:
		incoming_length = incoming_byte;
		incoming_data.push_back(incoming_byte);
		fsr_state = LookingFor::data;
		break;
	  case LookingFor::data:
		// Record the data
		incoming_data.push_back(incoming_byte);

		// If we got it all, decode it and invoke the handler
		if (size_t(incoming_length + 3) == incoming_data.size())
		{
		  Packet inPacket(incoming_data);
		  auto handler_it = type_handlers.find(inPacket.packetType());
		  if (handler_it != type_handlers.end())
		  {
			handler_it->second.handler(handler_it->second.param, inPacket.payload());
		  }
		  else
		  {
			// We didn't find the handler.
		  }
		  // Reset to receive the next.
		  fsr_state = LookingFor::header;
		  incoming_data.clear();
		}
		break;
	}
  }

public:

  // Send a package with lspc
  //
  // @brief Sends a packaged buffer over the USB serial link.
  //
  // @param type The message type. This is user specific; any type between 1-255.
  // @param payload A vector with the serialized payload to be sent.
  //
  // @return True if the packet was sent.
  virtual bool send(uint8_t type, const std::vector<uint8_t> &payload) = 0;

  // Send a serializable object with lspc
  //
  // @brief Serializes an object and sends it over the USB serial link.
  //
  // @param obj The object to send.
  //
  // @return True if the object was sent.
  bool send(const lspc::Serializable & obj)
  {
	return send(obj.type(), obj.serialize());
  }

  // Register a callback to handle incoming messages
  //
  // @param type The message type to handle. The type is user specific; any
  // number 1-255.
  // @param handler Callback function of the form void
  // callback(uint8_t* payload, size_t len). With payload being a buffer
  // containing the serialized payload and len the lenght of the payload.
  //
  // @return True if the registration succeeded.
  bool registerCallback(uint8_t type, void (*handler)(void * param, const std::vector<uint8_t>&), void * parameter = 0)
  {
	if (type == 0x00)
	{
	  return false;
	}

	if (type_handlers.find(type) != type_handlers.end()) {
		return false; // callback already registered - this ensures that we can not overwrite an existing registered callback
	}

	callback_t callback;
	callback.handler = handler;
	callback.param = parameter;

	type_handlers[type] = callback;

	return true;
  }

  bool unregisterCallback(uint8_t type)
  {
		if (type == 0x00)
		{
		  return false;
		}

		auto handler_it = type_handlers.find(type);
		if (handler_it == type_handlers.end()) {
			return false; // callback not registered already registered - this ensures that we can not overwrite an existing registered callback
		}

		type_handlers.erase(handler_it); // remove/unregister the callback

		return true;
  }
};

} // namespace LSPC

#endif // LSPC_SOCKETBASE_HPP
