#pragma once

#include <map>

namespace sapphire::backend {

class EnvSensor final {
 public:
  enum Type {
    Undefined = 0,       ///< Uninitialized / unknown channel.
    AmbientLight,        ///< Ambient illuminance (lx).
  };

 public:
  EnvSensor() : type_(Undefined), value_(0.0), stamp_(0.0) {}

  EnvSensor(const Type& type, const double& value, const double& stamp = 0) : type_(type), value_(value), stamp_(stamp) {}

  ~EnvSensor() = default;

  const Type& type() const { return type_; }
  const double& value() const { return value_; }
  const double& stamp() const { return stamp_; }

 private:
  Type type_;
  double value_;
  double stamp_;
};

using EnvSensors = std::map<EnvSensor::Type, EnvSensor>;


}  // namespace sapphire::backend
