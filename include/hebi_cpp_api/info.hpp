#pragma once

#include "hebi.h"

#include <string>

#include "color.hpp"
#include "gains.hpp"
#include "ip_address.hpp"
#include "message_helpers.hpp"
#include "util.hpp"

namespace hebi {

/// \brief Info objects have various fields representing the module state; which
/// fields are populated depends on the module type and various other settings.
///
/// This object has a hierarchical structure -- there are some direct general-purpose
/// fields at the top level, and many more specific fields contained in different
/// nested subobjects.
///
/// The subobjects contain references to the parent info object, and so should not be
/// used after the parent object has been destroyed.
///
/// The fields in the info object are typed; generally, these are optional-style
/// read-only fields (i.e., have the concept of has/get), although the return types
/// and exact interface vary slightly between fields. Where appropriate, the explicit
/// bool operator has been overridden so that you can shortcut @c if(field.has()) by
/// calling @c if(field).
///
/// Although this header file can be used to look at the hierarchy of the messages,
/// in general the online documentation at apidocs.hebi.us presents this information.
/// in a more readable form.
class Info final {
public:
  enum class ControlStrategy {
    /// The motor is not given power (equivalent to a 0 PWM value)
    Off,
    /// A direct PWM value (-1 to 1) can be sent to the motor (subject to onboard safety limiting).
    DirectPWM,
    /// A combination of the position, velocity, and effort loops with P and V feeding to T; documented on docs.hebi.us
    /// under "Control Modes"
    Strategy2,
    /// A combination of the position, velocity, and effort loops with P, V, and T feeding to PWM; documented on
    /// docs.hebi.us under "Control Modes"
    Strategy3,
    /// A combination of the position, velocity, and effort loops with P feeding to T and V feeding to PWM; documented
    /// on docs.hebi.us under "Control Modes"
    Strategy4,
  };

  enum class CalibrationState {
    /// The module has been calibrated; this is the normal state
    Normal,
    /// The current has not been calibrated
    UncalibratedCurrent,
    /// The factory zero position has not been set
    UncalibratedPosition,
    /// The effort (e.g., spring nonlinearity) has not been calibrated
    UncalibratedEffort,
  };

  enum class MstopStrategy {
    /// Triggering the M-Stop has no effect.
    Disabled,
    /// Triggering the M-Stop results in the control strategy being set to 'off'. Remains 'off' until changed by user.
    MotorOff,
    /// Triggering the M-Stop results in the motor holding the motor position. Operations resume to normal once trigger is released.
    HoldPosition,
  };

  enum class PositionLimitStrategy {
    /// Exceeding the position limit results in the actuator holding the position. Needs to be manually set to 'disabled' to recover.
    HoldPosition,
    /// Exceeding the position limit results in a virtual spring that pushes the actuator back to within the limits.
    DampedSpring,
    /// Exceeding the position limit results in the control strategy being set to 'off'. Remains 'off' until changed by user.
    MotorOff,
    /// Exceeding the position limit has no effect.
    Disabled,
  };

protected:
  /// \brief A message field representable by a single-precision floating point value.
  class FloatField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    FloatField(const HebiInfoRef& internal, HebiInfoFloatField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Info::FloatField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns that value; otherwise,
    /// returns a default.
    float get() const;

    HEBI_DISABLE_COPY_MOVE(FloatField)
  private:
    const HebiInfoRef& internal_;
    HebiInfoFloatField const field_;
  };

  /// \brief A message field for an angle measurement which does not lose
  /// precision at very high angles.
  ///
  /// This field is represented as an int64_t for the number of revolutions
  /// and a float for the radian offset from that number of revolutions.
  class HighResAngleField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    HighResAngleField(const HebiInfoRef& internal, HebiInfoHighResAngleField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Info::HighResAngleField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns that value as a double;
    /// otherwise, returns a default.
    ///
    /// Note that some precision might be lost converting to a double at
    /// very high number of revolutions.
    double get() const;
    /// \brief If the field has a value, returns that value in the int64
    /// and float parameters passed in; otherwise, returns a default.
    ///
    /// Note that this maintains the full precision of the underlying angle
    /// measurement, even for very large numbers of revolutions.
    ///
    /// \param revolutions The number of full revolutions
    ///
    /// \param radian_offset The offset from the given number of full
    /// revolutions.  Note that this is usually between 0 and @c 2*M_PI, but
    /// callers should not assume this.
    void get(int64_t* revolutions, float* radian_offset) const;

    HEBI_DISABLE_COPY_MOVE(HighResAngleField)
  private:
    const HebiInfoRef& internal_;
    HebiInfoHighResAngleField const field_;
  };

  /// \brief A message field representable by an unsigned 64 bit integer value.
  class IpAddressField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    IpAddressField(const HebiInfoRef& internal, HebiInfoUInt64Field field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Info::IpAddressField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns that value; otherwise,
    /// returns 0.0.0.0.
    IpAddress get() const;

    HEBI_DISABLE_COPY_MOVE(IpAddressField)
  private:
    const HebiInfoRef& internal_;
    HebiInfoUInt64Field const field_;
  };

  /// \brief A message field representable by a bool value.
  class BoolField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    BoolField(const HebiInfoRef& internal, HebiInfoBoolField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns that value; otherwise,
    /// returns false.
    bool get() const;

    HEBI_DISABLE_COPY_MOVE(BoolField)
  private:
    const HebiInfoRef& internal_;
    HebiInfoBoolField const field_;
  };

  /// \brief A message field representable by a std::string.
  class StringField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    StringField(HebiInfoPtr internal, HebiInfoStringField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Info::StringField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const;
    /// \brief If the field has a value, returns a copy of that value;
    /// otherwise, returns a default.
    std::string get() const;

    HEBI_DISABLE_COPY_MOVE(StringField)
  private:
    HebiInfoPtr const internal_;
    HebiInfoStringField const field_;
  };

  /// \brief A two-state message field (either set/true or cleared/false).
  class FlagField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    FlagField(const HebiInfoRef& internal, HebiInfoFlagField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the flag is set without
    /// directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Info::FlagField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief Returns @c true if the flag is set, false if it is cleared.
    bool has() const;

    HEBI_DISABLE_COPY_MOVE(FlagField)
  private:
    const HebiInfoRef& internal_;
    HebiInfoFlagField const field_;
  };

  /// \brief A message field representable by an enum of a given type.
  template<typename T>
  class EnumField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    EnumField(const HebiInfoRef& internal, HebiInfoEnumField field) : internal_(internal), field_(field) {}
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the field has a value
    /// without directly calling @c has().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Info::EnumField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has value: " << f.get() << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return has(); }
    /// \brief True if (and only if) the field has a value.
    bool has() const {
      return (enumGetter(internal_, field_, nullptr) == HebiStatusSuccess);
    }
    /// \brief If the field has a value, returns that value; otherwise,
    /// returns a default.
    T get() const {
      int32_t ret{};
      enumGetter(internal_, field_, &ret);
      return static_cast<T>(ret);
    }

    HEBI_DISABLE_COPY_MOVE(EnumField)
  private:
    const HebiInfoRef& internal_;
    HebiInfoEnumField const field_;
  };

  /// \brief A message field for interfacing with a bank of I/O pins.
  class IoBank final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    IoBank(HebiInfoPtr internal, HebiInfoRef& internal_ref, HebiInfoIoPinBank bank);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief True if (and only if) the particular numbered pin in this
    /// bank has a string label set in this message.
    ///
    /// \param pinNumber Which pin to set; valid values for pinNumber
    /// depend on the bank.
    bool hasLabel(size_t pinNumber) const;
    /// \brief If this numbered pin in this bank has a string label value,
    /// returns that value; otherwise returns an empty string.
    ///
    /// \param pinNumber Which pin to get; valid values for pinNumber
    /// depend on the bank.
    std::string getLabel(size_t pinNumber) const;

    HEBI_DISABLE_COPY_MOVE(IoBank)
  private:
    HebiInfoPtr internal_;
    HebiInfoRef& internal_ref_;
    HebiInfoIoPinBank const bank_;
  };
  /// \brief A message field for interfacing with an LED.
  class LedField final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    LedField(const HebiInfoRef& internal, HebiInfoLedField field);
#endif // DOXYGEN_OMIT_INTERNAL
    /// \brief Allows casting to a bool to check if the LED color is set
    /// without directly calling @c hasColor().
    ///
    /// This can be used as in the following (assuming 'parent' is a parent message,
    /// and this field is called 'myField')
    /// \code{.cpp}
    /// Info::LedField& f = parent.myField();
    /// if (f)
    ///   std::cout << "Field has color!" << std::endl;
    /// else
    ///   std::cout << "Field has no value!" << std::endl;
    /// \endcode
    explicit operator bool() const { return hasColor(); }
    /// \brief Returns true if the LED color is set, and false otherwise.
    bool hasColor() const;
    /// \brief Returns the led color.
    Color getColor() const;

    HEBI_DISABLE_COPY_MOVE(LedField)
  private:
    const HebiInfoRef& internal_;
    HebiInfoLedField const field_;
  };

  /// Any available digital or analog output pins on the device.
  class Io final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    Io(HebiInfoPtr internal, HebiInfoRef& internal_ref)
      : a_(internal, internal_ref, HebiInfoIoBankA),
        b_(internal, internal_ref, HebiInfoIoBankB),
        c_(internal, internal_ref, HebiInfoIoBankC),
        d_(internal, internal_ref, HebiInfoIoBankD),
        e_(internal, internal_ref, HebiInfoIoBankE),
        f_(internal, internal_ref, HebiInfoIoBankF) {}
#endif // DOXYGEN_OMIT_INTERNAL

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Subfields ----------------

    /// I/O pin bank a (pins 1-8 available)
    IoBank& a() { return a_; }
    /// I/O pin bank a (pins 1-8 available)
    const IoBank& a() const { return a_; }
    /// I/O pin bank b (pins 1-8 available)
    IoBank& b() { return b_; }
    /// I/O pin bank b (pins 1-8 available)
    const IoBank& b() const { return b_; }
    /// I/O pin bank c (pins 1-8 available)
    IoBank& c() { return c_; }
    /// I/O pin bank c (pins 1-8 available)
    const IoBank& c() const { return c_; }
    /// I/O pin bank d (pins 1-8 available)
    IoBank& d() { return d_; }
    /// I/O pin bank d (pins 1-8 available)
    const IoBank& d() const { return d_; }
    /// I/O pin bank e (pins 1-8 available)
    IoBank& e() { return e_; }
    /// I/O pin bank e (pins 1-8 available)
    const IoBank& e() const { return e_; }
    /// I/O pin bank f (pins 1-8 available)
    IoBank& f() { return f_; }
    /// I/O pin bank f (pins 1-8 available)
    const IoBank& f() const { return f_; }

    HEBI_DISABLE_COPY_MOVE(Io)
  private:
    IoBank a_;
    IoBank b_;
    IoBank c_;
    IoBank d_;
    IoBank e_;
    IoBank f_;
  };

  using InfoGains = Gains<HebiInfoRef, FloatField, BoolField, HebiInfoFloatField, HebiInfoBoolField>;

  /// Module settings that are typically changed at a slower rate.
  class Settings final {
  protected:
    /// Actuator-specific settings, such as controller gains.
    class Actuator final {
    public:
#ifndef DOXYGEN_OMIT_INTERNAL
      Actuator(const HebiInfoRef& internal)
        : position_gains_(internal, HebiInfoFloatPositionKp, HebiInfoBoolPositionDOnError),
          velocity_gains_(internal, HebiInfoFloatVelocityKp, HebiInfoBoolVelocityDOnError),
          effort_gains_(internal, HebiInfoFloatEffortKp, HebiInfoBoolEffortDOnError),
          spring_constant_(internal, HebiInfoFloatSpringConstant),
          velocity_limit_min_(internal, HebiInfoFloatVelocityLimitMin),
          velocity_limit_max_(internal, HebiInfoFloatVelocityLimitMax),
          effort_limit_min_(internal, HebiInfoFloatEffortLimitMin),
          effort_limit_max_(internal, HebiInfoFloatEffortLimitMax),
          position_limit_min_(internal, HebiInfoHighResAnglePositionLimitMin),
          position_limit_max_(internal, HebiInfoHighResAnglePositionLimitMax),
          control_strategy_(internal, HebiInfoEnumControlStrategy),
          mstop_strategy_(internal, HebiInfoEnumMstopStrategy),
          min_position_limit_strategy_(internal, HebiInfoEnumMinPositionLimitStrategy),
          max_position_limit_strategy_(internal, HebiInfoEnumMaxPositionLimitStrategy) {}
#endif // DOXYGEN_OMIT_INTERNAL

      // With all submessage and field getters: Note that the returned reference
      // should not be used after the lifetime of this parent.

      // Submessages ----------------

      /// Controller gains for the position PID loop.
      const InfoGains& positionGains() const { return position_gains_; }
      /// Controller gains for the velocity PID loop.
      const InfoGains& velocityGains() const { return velocity_gains_; }
      /// Controller gains for the effort PID loop.
      const InfoGains& effortGains() const { return effort_gains_; }

      // Subfields ----------------

      /// The spring constant of the module.
      const FloatField& springConstant() const { return spring_constant_; }
      /// The firmware safety limit for the minimum allowed velocity.
      const FloatField& velocityLimitMin() const { return velocity_limit_min_; }
      /// The firmware safety limit for the maximum allowed velocity.
      const FloatField& velocityLimitMax() const { return velocity_limit_max_; }
      /// The firmware safety limit for the minimum allowed effort.
      const FloatField& effortLimitMin() const { return effort_limit_min_; }
      /// The firmware safety limit for the maximum allowed effort.
      const FloatField& effortLimitMax() const { return effort_limit_max_; }
      /// The firmware safety limit for the minimum allowed position.
      const HighResAngleField& positionLimitMin() const { return position_limit_min_; }
      /// The firmware safety limit for the maximum allowed position.
      const HighResAngleField& positionLimitMax() const { return position_limit_max_; }
      /// How the position, velocity, and effort PID loops are connected in order to control motor PWM.
      const EnumField<ControlStrategy>& controlStrategy() const { return control_strategy_; }
      /// The motion stop strategy for the actuator
      const EnumField<MstopStrategy>& mstopStrategy() const { return mstop_strategy_; }
      /// The position limit strategy (at the minimum position) for the actuator
      const EnumField<PositionLimitStrategy>& minPositionLimitStrategy() const { return min_position_limit_strategy_; }
      /// The position limit strategy (at the maximum position) for the actuator
      const EnumField<PositionLimitStrategy>& maxPositionLimitStrategy() const { return max_position_limit_strategy_; }

      HEBI_DISABLE_COPY_MOVE(Actuator)
    private:
      InfoGains position_gains_;
      InfoGains velocity_gains_;
      InfoGains effort_gains_;

      FloatField spring_constant_;
      FloatField velocity_limit_min_;
      FloatField velocity_limit_max_;
      FloatField effort_limit_min_;
      FloatField effort_limit_max_;
      HighResAngleField position_limit_min_;
      HighResAngleField position_limit_max_;
      EnumField<ControlStrategy> control_strategy_;
      EnumField<MstopStrategy> mstop_strategy_;
      EnumField<PositionLimitStrategy> min_position_limit_strategy_;
      EnumField<PositionLimitStrategy> max_position_limit_strategy_;
    };

    /// IMU-specific settings.
    class Imu final {
    public:
#ifndef DOXYGEN_OMIT_INTERNAL
      Imu(const HebiInfoRef& internal)
        : accel_includes_gravity_(internal, HebiInfoBoolAccelIncludesGravity) {}
#endif // DOXYGEN_OMIT_INTERNAL

      // With all submessage and field getters: Note that the returned reference
      // should not be used after the lifetime of this parent.

      // Subfields ----------------

      /// Whether to include acceleration due to gravity in acceleration feedback.
      const BoolField& accelIncludesGravity() const { return accel_includes_gravity_; }

      HEBI_DISABLE_COPY_MOVE(Imu)
    private:
      BoolField accel_includes_gravity_;
    };

  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    Settings(HebiInfoPtr internal_ptr, const HebiInfoRef& internal)
      : actuator_(internal),
        imu_(internal),
        name_(internal_ptr, HebiInfoStringName),
        family_(internal_ptr, HebiInfoStringFamily),
        electrical_type_(internal_ptr, HebiInfoStringElectricalType),
        electrical_revision_(internal_ptr, HebiInfoStringElectricalRevision),
        mechanical_type_(internal_ptr, HebiInfoStringMechanicalType),
        mechanical_revision_(internal_ptr, HebiInfoStringMechanicalRevision),
        firmware_type_(internal_ptr, HebiInfoStringFirmwareType),
        firmware_revision_(internal_ptr, HebiInfoStringFirmwareRevision),
        user_settings_bytes_1_(internal_ptr, HebiInfoStringUserSettingsBytes1),
        user_settings_bytes_2_(internal_ptr, HebiInfoStringUserSettingsBytes2),
        user_settings_bytes_3_(internal_ptr, HebiInfoStringUserSettingsBytes3),
        user_settings_bytes_4_(internal_ptr, HebiInfoStringUserSettingsBytes4),
        user_settings_bytes_5_(internal_ptr, HebiInfoStringUserSettingsBytes5),
        user_settings_bytes_6_(internal_ptr, HebiInfoStringUserSettingsBytes6),
        user_settings_bytes_7_(internal_ptr, HebiInfoStringUserSettingsBytes7),
        user_settings_bytes_8_(internal_ptr, HebiInfoStringUserSettingsBytes8),
        user_settings_float_1_(internal, HebiInfoFloatUserSettingsFloat1),
        user_settings_float_2_(internal, HebiInfoFloatUserSettingsFloat2),
        user_settings_float_3_(internal, HebiInfoFloatUserSettingsFloat3),
        user_settings_float_4_(internal, HebiInfoFloatUserSettingsFloat4),
        user_settings_float_5_(internal, HebiInfoFloatUserSettingsFloat5),
        user_settings_float_6_(internal, HebiInfoFloatUserSettingsFloat6),
        user_settings_float_7_(internal, HebiInfoFloatUserSettingsFloat7),
        user_settings_float_8_(internal, HebiInfoFloatUserSettingsFloat8),
        ip_address_(internal, HebiInfoUInt64IpAddress),
        subnet_mask_(internal, HebiInfoUInt64SubnetMask),
        default_gateway_(internal, HebiInfoUInt64DefaultGateway),
        save_current_settings_(internal, HebiInfoFlagSaveCurrentSettings) {}
#endif // DOXYGEN_OMIT_INTERNAL

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Submessages ----------------

    /// Actuator-specific settings, such as controller gains.
    const Actuator& actuator() const { return actuator_; }
    /// IMU-specific settings.
    const Imu& imu() const { return imu_; }

    // Subfields ----------------

    /// Gets the name for this module.
    const StringField& name() const { return name_; }
    /// Gets the family for this module.
    const StringField& family() const { return family_; }
    /// Gets the electrical type of this module
    const StringField& electricalType() const { return electrical_type_; }
    /// Gets the electrical revision of this module
    const StringField& electricalRevision() const { return electrical_revision_; }
    /// Gets the mechanical type of this module
    const StringField& mechanicalType() const { return mechanical_type_; }
    /// Gets the mechanical revision of this module
    const StringField& mechanicalRevision() const { return mechanical_revision_; }
    /// Gets the firmware type of this module
    const StringField& firmwareType() const { return firmware_type_; }
    /// Gets the firmware revision of this module
    const StringField& firmwareRevision() const { return firmware_revision_; }
    /// Gets the given byte array user setting; valid for entry number 1-8.
    /// Throws out of range if given an invalid index
    const StringField& userSettingsBytes(size_t number) const
    {
      switch (number) {
        case 1: return user_settings_bytes_1_;
        case 2: return user_settings_bytes_2_;
        case 3: return user_settings_bytes_3_;
        case 4: return user_settings_bytes_4_;
        case 5: return user_settings_bytes_5_;
        case 6: return user_settings_bytes_6_;
        case 7: return user_settings_bytes_7_;
        case 8: return user_settings_bytes_8_;
      }
      throw std::out_of_range("Invalid option for bytes array user setting entry!");
    }
    /// Gets the given float user setting; valid for entry number 1-8.
    /// Throws out of range if given an invalid index
    const FloatField& userSettingsFloat(size_t number) const
    {
      switch (number) {
        case 1: return user_settings_float_1_;
        case 2: return user_settings_float_2_;
        case 3: return user_settings_float_3_;
        case 4: return user_settings_float_4_;
        case 5: return user_settings_float_5_;
        case 6: return user_settings_float_6_;
        case 7: return user_settings_float_7_;
        case 8: return user_settings_float_8_;
      }
      throw std::out_of_range("Invalid option for float user setting entry!");
    }
    /// Gets the IP address for this module.
    const IpAddressField& ipAddress() const { return ip_address_; }
    /// Gets the subnet mask for this module.
    const IpAddressField& subnetMask() const { return subnet_mask_; }
    /// Gets the default gateway for this module.
    const IpAddressField& defaultGateway() const { return default_gateway_; }
    /// Indicates if the module should save the current values of all of its settings.
    const FlagField& saveCurrentSettings() const { return save_current_settings_; }

    HEBI_DISABLE_COPY_MOVE(Settings)
  private:
    Actuator actuator_;
    Imu imu_;
    StringField name_;
    StringField family_;
    StringField electrical_type_;
    StringField electrical_revision_;
    StringField mechanical_type_;
    StringField mechanical_revision_;
    StringField firmware_type_;
    StringField firmware_revision_;
    StringField user_settings_bytes_1_;
    StringField user_settings_bytes_2_;
    StringField user_settings_bytes_3_;
    StringField user_settings_bytes_4_;
    StringField user_settings_bytes_5_;
    StringField user_settings_bytes_6_;
    StringField user_settings_bytes_7_;
    StringField user_settings_bytes_8_;
    FloatField user_settings_float_1_;
    FloatField user_settings_float_2_;
    FloatField user_settings_float_3_;
    FloatField user_settings_float_4_;
    FloatField user_settings_float_5_;
    FloatField user_settings_float_6_;
    FloatField user_settings_float_7_;
    FloatField user_settings_float_8_;
    IpAddressField ip_address_;
    IpAddressField subnet_mask_;
    IpAddressField default_gateway_;
    FlagField save_current_settings_;
  };

  /// Actuator-specific information.
  class Actuator final {
  public:
#ifndef DOXYGEN_OMIT_INTERNAL
    Actuator(const HebiInfoRef& internal) : calibration_state_(internal, HebiInfoEnumCalibrationState) {}
#endif // DOXYGEN_OMIT_INTERNAL

    // With all submessage and field getters: Note that the returned reference
    // should not be used after the lifetime of this parent.

    // Subfields ----------------

    /// The calibration state of the module
    const EnumField<CalibrationState>& calibrationState() const { return calibration_state_; }

    HEBI_DISABLE_COPY_MOVE(Actuator)
  private:
    EnumField<CalibrationState> calibration_state_;
  };

private:
  /**
   * C-style object; managed by parent.
   * NOTE: this should not be used except by internal library functions!
   */
  HebiInfoPtr internal_;
  HebiInfoRef internal_ref_;

public:
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * \brief Wraps an existing C-style object that is managed by its parent.
   * NOTE: this should not be used except by internal library functions!
   */
  Info(HebiInfoPtr);
#endif // DOXYGEN_OMIT_INTERNAL
  /**
   * \brief Move constructor (necessary for containment in STL template classes)
   */
  Info(Info&& other);

  // With all submessage and field getters: Note that the returned reference
  // should not be used after the lifetime of this parent.

  // Submessages -------------------------------------------------------------

  /// Any available digital or analog output pins on the device.
  Io& io() { return io_; }
  /// Any available digital or analog output pins on the device.
  const Io& io() const { return io_; }
  /// Module settings that are typically changed at a slower rate.
  const Settings& settings() const { return settings_; }
  /// Actuator-specific information.
  const Actuator& actuator() const { return actuator_; }

  // Subfields -------------------------------------------------------------

  /// Gets the serial number for this module (e.g., X5-0001).
  const StringField& serial() const { return serial_; }
  /// The module's LED.
  const LedField& led() const { return led_; }

  /**
   * Disable copy constructor/assignment operators
   */
  HEBI_DISABLE_COPY(Info)

  /* Disable move assigment operator. */
  Info& operator=(Info&& other) = delete;

private:
  Io io_;
  Settings settings_;
  Actuator actuator_;

  StringField serial_;
  LedField led_;
};

} // namespace hebi
