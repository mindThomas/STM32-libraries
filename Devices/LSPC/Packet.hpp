#ifndef LSPC_PACKET_HPP
#define LSPC_PACKET_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#ifdef __EXCEPTIONS
#include <stdexcept>
#endif

namespace lspc
{

class Packet
{
  std::vector<uint8_t> encoded_buffer_;
  std::vector<uint8_t> decoded_payload_;
  uint8_t packet_type_;
  bool degenerate_ = false;

  // Encode the payload with COBS
  //
  // @param input A buffer with the raw unencoded data.
  //
  // @return True if encoding succeeded, false if if the input is more than 254
  // bytes.
  void encodePayload(const std::vector<uint8_t> &input)
  {
	size_t output_offset = 3;
	size_t code_idx = output_offset;
	uint8_t code = 1;

	for (size_t b = 0; b < input.size(); ++b, ++code)
	{
	  if (0x00 == input[b])
	  {
		encoded_buffer_[code_idx] = code;
		code_idx = b + 1 + output_offset;
		code = 0;
	  }
	  else
	  {
		encoded_buffer_[b + 1 + output_offset] = input[b];
	  }
	}
	encoded_buffer_[code_idx] = code;

	return;
  };

  // Decode a downstream buffer with COBS.
  //
  // @param input COBS encoded byte-string.
  // @param output A decoded buffer of (input_size - 1) bytes, because the
  // decoder removes one byte of overhead.
  //
  // @return True if decoding succeded, false if the input is zero-size or
  // wrongly encoded.
  bool decodePayload(std::vector<uint8_t> &output)
  {
	size_t dec_payload_size = encoded_buffer_.size() - 4;
	output.resize(dec_payload_size);

	size_t in_index = 3; // the byte after start byte, type and length
	uint8_t code = encoded_buffer_[in_index];
	size_t code_accumulator = code;
	++in_index;

	// Go through the buffer, copying it to the packet. Inside each code block,
	// copy the byte verbatim, then get the offset code and write a 0x00 instead.
	for (size_t out_index = 0; out_index < dec_payload_size; ++out_index, ++in_index)
	{
	  for (int i = 1; i < code && out_index < dec_payload_size; ++i, ++out_index, ++in_index)
	  {
		output[out_index] = encoded_buffer_[in_index];
	  }
	  if (out_index >= dec_payload_size)
		break;
	  output[out_index] = 0x00;
	  code = encoded_buffer_[in_index];
	  code_accumulator += code;
	};

	// The accumulated codes should equate to the length of the input. If this is
	// not true, something has gone wrong.
	if (code_accumulator != dec_payload_size + 1)
	{
	  return false;
	}

	return true;
  };

  public:
  // Initialize from encoded buffer
  Packet(const std::vector<uint8_t>& buffer)
  {
	encoded_buffer_ = buffer;
	if (encoded_buffer_.size() < 4)
	{
	  degenerate_ = true;
	  #ifdef __EXCEPTIONS
		throw std::length_error("Encoded buffer too small.");
	  #endif
	}

	if (encoded_buffer_.size() > 258)
	{
	  degenerate_ = true;
	  #ifdef __EXCEPTIONS
		throw std::length_error("Encoded buffer too big.");
	  #endif
	}

	packet_type_ = encoded_buffer_[1];
	if (packet_type_ == 0x00)
	{
	  degenerate_ = true;
	  #ifdef __EXCEPTIONS
		throw std::runtime_error("Received packet with type 0x00.");
	  #endif
	}

	if (encoded_buffer_.size() != size_t(encoded_buffer_[2] + 3))
	{
	  degenerate_ = true;
	  #ifdef __EXCEPTIONS
		throw std::runtime_error("Packet length field does not match buffer size.");
	  #endif
	}

	if (! decodePayload(decoded_payload_))
	{
	  degenerate_ = true;
	  #ifdef __EXCEPTIONS
		throw std::runtime_error("Decoding failed.");
	  #endif
	}
  }

  // Initialize from serialized payload
  Packet(uint8_t package_type, const std::vector<uint8_t>& payload)
  {
	decoded_payload_ = payload;

	// Encoding the packet:
	// Make space for the content plus header
	encoded_buffer_.resize(payload.size() + 4);
	// The package starts out with a 0x00
	encoded_buffer_[0] = 0x00;

	// The type comes next. It can be 0x01-0xFF but not 0x00.
	packet_type_ = package_type;
	if (package_type == 0x00)
	{
	  degenerate_ = true;
	  #ifdef __EXCEPTIONS
		throw std::runtime_error("Constructed LSPC packet with type 0x00.");
	  #endif
	}
	encoded_buffer_[1] = package_type;

	// The size of the payload is one longer due to COBS encoding. So the
	// payload can be no more than 254 bytes. If it is longer, the rest of the
	// payload is discarded.
	if (payload.size() > 254)
	{
	  encoded_buffer_[2] = 255;
	  degenerate_ = true;
	  #ifdef __EXCEPTIONS
		throw std::length_error("Payload longer than 254 bytes.");
	  #endif
	}
	else
	{
	  encoded_buffer_[2] = payload.size() + 1;
	}

	// Encode the payload
	encodePayload(payload);
  }

  uint8_t* encodedDataPtr()
  {
	return encoded_buffer_.data();
  }

  size_t encodedDataSize()
  {
	return encoded_buffer_.size();
  }

  std::vector<uint8_t> encodedBuffer()
  {
	return encoded_buffer_;
  }

  uint8_t packetType()
  {
	return packet_type_;
  }

  bool isDegenerate()
  {
	return degenerate_;
  }

  std::vector<uint8_t> payload()
  {
	return decoded_payload_;
  }
};

} // namespace LSPC

#endif // LSPC_PACKET_HPP