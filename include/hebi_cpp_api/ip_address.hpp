#pragma once

#include "hebi.h"

#include <iostream>

#ifdef WIN32

#define NOMINMAX
#include <ws2tcpip.h>
#include <stdlib.h>
#define bswap_32(x) _byteswap_ulong(x)
using in_addr_t = in_addr;

#elif defined(__APPLE__)

// Mac OS X / Darwin features
#include <libkern/OSByteOrder.h>
#define bswap_32(x) OSSwapInt32(x)
#include <arpa/inet.h>

#else // Linux

#include <arpa/inet.h>
#include <byteswap.h>

#endif

namespace hebi {

/**
 * \brief A simple wrapper class for IpAddress objects.
 */
class IpAddress final {
public:
  /**
   * \brief Creates IPv4 address 0.0.0.0
   */
  IpAddress() = default;

  // Create IP Address from network order uint32_t
  IpAddress(uint32_t address_raw) : address_network_order_(address_raw) {}

  /**
   * \brief Creates an IpAddress from individual bytes
   */
  static IpAddress fromBytes(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    // TODO: depends on byte order:
    uint32_t tmp = (a << 24) | (b << 16) | (c << 8) | (d);
    return IpAddress(tmp);
  }
  
  /**
   * \brief Creates an IpAddress from a little endian uint32_t
   */
  static IpAddress fromLittleEndian(uint32_t raw) {
    return IpAddress(bswap_32(raw));
  }

  /**
   * \brief Sets the value of the current IpAddress to the value given in
   * 'ip_str'.
   *
   * This value must be a valid string of format a.b.c.d, where 'a' - 'd'
   * are integers from 0-255.
   *
   * \returns 'true' on success (valid ip_str), 'false' on failure.
   */
  bool setToString(const std::string& ip_str) {
    in_addr_t addr;
    if (inet_pton(AF_INET, ip_str.c_str(), &addr) != 1)
      return false;
  #ifdef WIN32
    address_network_order_ = addr.S_un.S_addr;
  #else
    address_network_order_ = addr;
  #endif
    return true;
  }

  /**
   * \brief Returns a string representation of the IP address
   */
  std::string toString() const {
    std::string res;
    res.resize(INET_ADDRSTRLEN);
    inet_ntop(AF_INET, &address_network_order_, &res[0], INET_ADDRSTRLEN);
    return res;
  }

  /**
   * \brief Returns the IP Address as a little endian uint32_t
   */
  uint32_t getLittleEndian() const {
    return bswap_32(address_network_order_);
  }

  /**
   * \brief Returns the IP Address as a big endian (network order) uint32_t
   */
  uint32_t getBigEndian() const {
    return address_network_order_;
  }

private:
  uint32_t address_network_order_{};
};

} // namespace hebi
