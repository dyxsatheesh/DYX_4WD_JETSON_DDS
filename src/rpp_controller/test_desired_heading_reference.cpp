#include "desired_heading_reference.hpp"
#include <cassert>
#include <cmath>
#include <stdexcept>
int main() {
 constexpr double pi=3.14159265358979323846;
 const auto east=dyx::rpp::desired_heading_reference(.2,0);
 assert(std::abs(east.east_mps-.2)<1e-9);
 const auto north=dyx::rpp::desired_heading_reference(.2,pi/2);
 assert(std::abs(north.north_mps-.2)<1e-9);
 const auto pivot=dyx::rpp::desired_heading_reference(0,pi);
 assert(std::abs(pivot.north_mps)<1e-9 && std::abs(pivot.east_mps)<1e-9);
 bool reject=false;try {dyx::rpp::desired_heading_reference(-.1,0);}catch(const std::invalid_argument&){reject=true;}assert(reject);
}
