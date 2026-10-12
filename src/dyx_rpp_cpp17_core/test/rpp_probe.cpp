#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char **argv) {
  try {
    if (argc < 3) throw std::invalid_argument("operation and numeric args required");
    const std::string op(argv[1]);
    auto a = [&](int i) -> double {
      if (i >= argc) throw std::invalid_argument("missing numeric argument");
      return std::stod(argv[i]);
    };
    std::cout << std::setprecision(17);
    using namespace dyx::rpp;
    if (op == "wrap") std::cout << wrap_angle(a(2));
    else if (op == "projection") {
      const Segment seg{7, {a(2), a(3)}, {a(4), a(5)}, 10.0};
      const auto p = project_onto_segment({a(6), a(7)}, seg);
      std::cout << p.t << ' ' << p.foot.x << ' ' << p.foot.y << ' '
                << p.signed_cross_track_m << ' ' << p.distance_m << ' ' << p.s;
    } else if (op == "lookahead") {
      std::cout << adaptive_lookahead_m(
          {a(2), a(3), a(4), a(5)}, a(6), a(7));
    } else if (op == "cone") {
      const auto p = limit_bearing_to_moving_cone(a(2), a(3), a(4));
      std::cout << p.first << ' ' << static_cast<int>(p.second);
    } else if (op == "brake") {
      std::cout << braking_distance(a(2), a(3), a(4), a(5), a(6));
    } else if (op == "allowable") {
      std::cout << allowable_speed_for_distance(a(2), a(3), a(4), a(5), a(6));
    } else throw std::invalid_argument("unknown operation");
    std::cout << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "RPP primitive failure: " << e.what() << '\n';
    return 2;
  }
}
