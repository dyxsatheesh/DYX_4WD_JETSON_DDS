#include "dyx_rpp_cpp17_core/stationary_detector.hpp"
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

int main(int argc, char **argv) {
  try {
    if (argc != 5) throw std::invalid_argument("expected 4 stationary detector limits");
    using dyx::rpp::PositionWindowStationaryDetector;
    const auto detector = std::make_unique<PositionWindowStationaryDetector>(
        std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]),
        std::stod(argv[4]));
    std::cout << std::setprecision(17);
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line == "reset") { detector->reset(); continue; }
      std::istringstream iss(line);
      std::string op;
      iss >> op;
      if (op != "update") throw std::invalid_argument("unsupported operation");
      double t{}, x{}, y{}, yaw{};
      if (!(iss >> t >> x >> y >> yaw)) {
        // MSVC stream extraction may reject NaN spellings; std::stod handles them.
        std::istringstream tokens(line);
        std::string op2, ts, xs, ys, rs;
        if (!(tokens >> op2 >> ts >> xs >> ys >> rs))
          throw std::invalid_argument("missing numeric argument");
        t = std::stod(ts); x = std::stod(xs); y = std::stod(ys); yaw = std::stod(rs);
      }
      const auto out = detector->update(t, x, y, yaw);
      std::cout << static_cast<int>(out.stationary) << ' ' << out.held_sec << ' '
                << out.maximum_displacement_m << ' '
                << out.maximum_abs_yaw_rate_radps << ' '
                << out.sample_count << '\n';
    }
  } catch (const std::exception &e) {
    std::cerr << "stationary detector probe error: " << e.what() << '\n';
    return 2;
  }
}
