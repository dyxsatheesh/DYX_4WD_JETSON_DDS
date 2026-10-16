#include "dyx_h6a/launch_gate.hpp"
#include <iostream>
int main(int argc, char** argv) {
  if (argc!=2) { std::cerr << "Usage: dyx_h6a_gate <repo-root>\n"; return 64; }
  const auto result=dyx::h6a::inspect(argv[1]);
  for(const auto& msg:result.errors) std::cerr << "H6A BLOCKED: " << msg << '\n';
  if (!result.pass) return 2;
  std::cout << "H6A OBSERVER SOURCE GATE PASSED (OFFLINE ONLY)\n"
            << "NOT CERTIFIED: ROS2/DDS runtime and hardware tests still required.\n";
  return 0;
}
