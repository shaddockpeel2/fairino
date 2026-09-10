// Read-only projector/protection state for one Orbbec device selected by serial.
#include <libobsensor/ObSensor.hpp>

#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: laser_state_probe SERIAL\n";
    return 2;
  }
  try {
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_ERROR);
    ob::Context context;
    auto devices = context.queryDeviceList();
    auto device = devices->getDeviceBySN(argv[1]);
    auto info = device->getDeviceInfo();
    std::cout << "name=" << info->name() << " serial=" << info->serialNumber() << '\n';
    const auto print_bool = [&device](OBPropertyID property, const char *name) {
      if (!device->isPropertySupported(property, OB_PERMISSION_READ)) {
        std::cout << name << "=unsupported\n";
        return;
      }
      std::cout << name << '=' << (device->getBoolProperty(property) ? "true" : "false") << '\n';
    };
    const auto print_int = [&device](OBPropertyID property, const char *name) {
      if (!device->isPropertySupported(property, OB_PERMISSION_READ)) {
        std::cout << name << "=unsupported\n";
        return;
      }
      std::cout << name << '=' << device->getIntProperty(property) << '\n';
    };
    const auto print_range = [&device](OBPropertyID property, const char *name) {
      if (!device->isPropertySupported(property, OB_PERMISSION_READ)) {
        std::cout << name << "=unsupported\n";
        return;
      }
      const auto range = device->getIntPropertyRange(property);
      std::cout << name << "={cur:" << range.cur << ",min:" << range.min
                << ",max:" << range.max << ",step:" << range.step
                << ",default:" << range.def << "}\n";
    };
    print_bool(OB_PROP_LASER_BOOL, "laser_bool");
    print_int(OB_PROP_LASER_CONTROL_INT, "laser_control");
    print_bool(OB_PROP_LDP_BOOL, "ldp");
    print_bool(OB_PROP_LDP_STATUS_BOOL, "ldp_protection");
    print_int(OB_PROP_LDP_MEASURE_DISTANCE_INT, "ldp_distance_mm");
    print_range(OB_PROP_DEPTH_EXPOSURE_INT, "depth_exposure");
    print_range(OB_PROP_DEPTH_GAIN_INT, "depth_gain");
    print_range(OB_PROP_IR_EXPOSURE_INT, "ir_exposure");
    print_range(OB_PROP_IR_GAIN_INT, "ir_gain");
    print_range(OB_PROP_LASER_ENERGY_LEVEL_INT, "laser_energy");
  } catch (const ob::Error &error) {
    std::cerr << error.getMessage() << '\n';
    return 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
