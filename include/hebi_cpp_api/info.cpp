#include "info.hpp"

#include <limits>

namespace hebi {

Info::FloatField::FloatField(const HebiInfoRef& internal, HebiInfoFloatField field) : internal_(internal), field_(field) {}

bool Info::FloatField::has() const { return (floatGetter(internal_, field_, nullptr) == HebiStatusSuccess); }

float Info::FloatField::get() const {
  float ret;
  if (floatGetter(internal_, field_, &ret) != HebiStatusSuccess) {
    ret = std::numeric_limits<float>::quiet_NaN();
  }
  return ret;
}

Info::HighResAngleField::HighResAngleField(const HebiInfoRef& internal, HebiInfoHighResAngleField field)
  : internal_(internal), field_(field) {}

bool Info::HighResAngleField::has() const {
  return (highResAngleGetter(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

double Info::HighResAngleField::get() const {
  int64_t revolutions;
  float radian_offset;
  if (highResAngleGetter(internal_, field_, &revolutions, &radian_offset) != HebiStatusSuccess) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (static_cast<double>(revolutions) * 2.0 * M_PI + static_cast<double>(radian_offset));
}

void Info::HighResAngleField::get(int64_t* revolutions, float* radian_offset) const {
  if (highResAngleGetter(internal_, field_, revolutions, radian_offset) != HebiStatusSuccess) {
    *revolutions = 0;
    *radian_offset = std::numeric_limits<float>::quiet_NaN();
  }
}

Info::IpAddressField::IpAddressField(const HebiInfoRef& internal, HebiInfoUInt64Field field)
  : internal_(internal), field_(field) {}

bool Info::IpAddressField::has() const {
  return (uint64Getter(internal_, field_, nullptr) == HebiStatusSuccess);
}

IpAddress Info::IpAddressField::get() const {
  uint64_t ret;
  if (uint64Getter(internal_, field_, &ret) != HebiStatusSuccess) {
    ret = 0;
  }
  return IpAddress::fromLittleEndian(static_cast<uint32_t>(ret));
}

Info::BoolField::BoolField(const HebiInfoRef& internal, HebiInfoBoolField field) : internal_(internal), field_(field) {}

bool Info::BoolField::has() const { return (boolGetter(internal_, field_, nullptr) == HebiStatusSuccess); }

bool Info::BoolField::get() const {
  bool ret{};
  boolGetter(internal_, field_, &ret);
  return static_cast<bool>(ret);
}

Info::StringField::StringField(HebiInfoPtr internal, HebiInfoStringField field) : internal_(internal), field_(field) {}

bool Info::StringField::has() const {
  return (hebiInfoGetString(internal_, field_, nullptr, nullptr) == HebiStatusSuccess);
}

std::string Info::StringField::get() const {
  // Get the size first
  size_t length;
  if (hebiInfoGetString(internal_, field_, nullptr, &length) != HebiStatusSuccess || length == 0) {
    // String field doesn't exist -- return an empty string
    return "";
  }
  std::string tmp(length - 1, 0);
  hebiInfoGetString(internal_, field_, &*tmp.begin(), &length);
  return tmp;
}

Info::FlagField::FlagField(const HebiInfoRef& internal, HebiInfoFlagField field) : internal_(internal), field_(field) {}

bool Info::FlagField::has() const { return (flagGetter(internal_, field_) == 1); }

Info::IoBank::IoBank(HebiInfoPtr internal, HebiInfoRef& internal_ref, HebiInfoIoPinBank bank) : internal_(internal), internal_ref_(internal_ref), bank_(bank) {}

bool Info::IoBank::hasLabel(size_t pinNumber) const {
  return (hebiInfoGetIoLabelString(internal_, bank_, pinNumber,  nullptr, nullptr) == HebiStatusSuccess);
}

std::string Info::IoBank::getLabel(size_t pinNumber) const {
  // Get the size first
  size_t length;
  if (hebiInfoGetIoLabelString(internal_, bank_, pinNumber, nullptr, &length) != HebiStatusSuccess || length == 0) {
    // String field doesn't exist -- return an empty string
    return "";
  }
  std::string tmp(length - 1, 0);
  hebiInfoGetIoLabelString(internal_, bank_, pinNumber, &*tmp.begin(), &length);
  return tmp;
}

Info::LedField::LedField(const HebiInfoRef& internal, HebiInfoLedField field) : internal_(internal), field_(field) {}

bool Info::LedField::hasColor() const {
  return ledGetter(internal_, field_, nullptr, nullptr, nullptr, nullptr) == HebiStatusSuccess;
}

Color Info::LedField::getColor() const {
  uint8_t r, g, b, a;
  if (ledGetter(internal_, field_, &r, &g, &b, &a) != HebiStatusSuccess) {
    r = 0;
    g = 0;
    b = 0;
    a = 0;
  }
  return Color(r, g, b, a);
}

Info::Info(HebiInfoPtr info)
  : internal_(info),
    io_(internal_, internal_ref_),
    settings_(internal_, internal_ref_),
    actuator_(internal_ref_),
    serial_(internal_, HebiInfoStringSerial),
    led_(internal_ref_, HebiInfoLedLed) {
  hebiInfoGetReference(internal_, &internal_ref_);
}

Info::Info(Info&& other)
  : internal_(other.internal_),
    io_(internal_, internal_ref_),
    settings_(internal_, internal_ref_),
    actuator_(internal_ref_),
    serial_(internal_, HebiInfoStringSerial),
    led_(internal_ref_, HebiInfoLedLed) {
  // NOTE: it would be nice to also cleanup the actual internal pointer of other
  // but alas we cannot change a const variable.
  hebiInfoGetReference(internal_, &internal_ref_);
}

} // namespace hebi
