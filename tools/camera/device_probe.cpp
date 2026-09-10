// Read-only SDK device identity and video profile enumeration; no stream or firmware writes.
#include <libobsensor/ObSensor.hpp>
#include <iostream>
int main() {
  try {
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_ERROR);
    ob::Context ctx;
    auto devices = ctx.queryDeviceList();
    std::cout << "devices=" << devices->deviceCount() << '\n';
    if (devices->deviceCount() != 1) return 2;
    auto dev = devices->getDevice(0);
    auto info = dev->getDeviceInfo();
    std::cout << "name=" << info->name() << " serial=" << info->serialNumber()
              << " firmware=" << info->firmwareVersion() << " pid=" << info->pid()
              << " connection=" << info->connectionType() << '\n';
    auto sensors = dev->getSensorList();
    for (unsigned i=0;i<sensors->count();++i) {
      auto sensor = sensors->getSensor(i);
      auto profiles = sensor->getStreamProfileList();
      for (unsigned j=0;j<profiles->count();++j) {
        auto p=profiles->getProfile(j)->as<ob::VideoStreamProfile>();
        std::cout << "sensor=" << sensor->type() << " " << p->width() << "x" << p->height()
                  << " fps=" << p->fps() << " format=" << p->format() << '\n';
      }
    }
  } catch (ob::Error &e) { std::cerr << e.getMessage() << '\n'; return 1; }
    catch (std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
