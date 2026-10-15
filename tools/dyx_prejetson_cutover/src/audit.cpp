#include "dyx_cutover/readiness.hpp"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  dyx::cutover::fs::path root;
  dyx::cutover::fs::path report;
  bool enforce = false;
  for (int i=1;i<argc;++i) {
    const std::string arg(argv[i]);
    if (arg=="--root" && i+1<argc) root=argv[++i];
    else if (arg=="--json" && i+1<argc) report=argv[++i];
    else if (arg=="--enforce") enforce=true;
    else { std::cerr<<"Unknown or incomplete option: "<<arg<<"\n"; return 64; }
  }
  if (root.empty()) {std::cerr<<"Required: --root <repo> [--json path] [--enforce]\n"; return 64;}
  try {
    const auto results=dyx::cutover::scan(root);
    for (const auto& r:results)
      std::cout<<dyx::cutover::label(r.disposition)<<" | "<<r.id<<" | "<<r.evidence<<"\n";
    std::cout<<"NOT CERTIFIED: source inventory is not functional feature-parity or hardware proof.\n";
    if (!report.empty()) {
      std::ofstream out(report, std::ios::binary|std::ios::trunc);
      if (!out) throw std::runtime_error("cannot write report");
      out << dyx::cutover::as_json(results);
      out.flush();
      if (!out) throw std::runtime_error("failed writing report");
    }
    // No implicit 'success' from absence of Python. Enforce is intentionally
    // fail-closed until signed functional, ROS2 and hardware acceptance exists.
    return enforce ? 2 : 0;
  } catch (const std::exception& e) {
    std::cerr << "AUDIT ERROR: " << e.what() << "\n";
    return 1;
  }
}
