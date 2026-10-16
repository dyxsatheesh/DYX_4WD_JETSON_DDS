#include "dyx_h6e/route_inventory.hpp"
#include <iostream>
int main(int argc,char**argv) {
  dyx::h6e::fs::path root,report;
  bool enforce=false;
  for(int i=1;i<argc;++i){
    const std::string v=argv[i];
    if(v=="--root" && i+1<argc)root=argv[++i];
    else if(v=="--json" && i+1<argc)report=argv[++i];
    else if(v=="--enforce")enforce=true;
    else {std::cerr<<"Usage: --root <repo> [--json file] [--enforce]\n";return 64;}
  }
  if(root.empty()){std::cerr<<"Missing --root\n";return 64;}
  try{
    const auto a=dyx::h6e::scan(root);
    std::cout<<"H6E1 LEGACY ROUTES: "<<a.legacy.size()<<"; NATIVE MODEL ROUTES: "
             <<a.native.size()<<"; EXACT PATH/METHOD MATCHES: "<<a.exact_matches<<'\n';
    std::cout<<"Socket.IO declared: "<<(a.socketio_declared?"YES":"NO")<<'\n';
    std::cout<<"NOT CERTIFIED: Static route inventory is not Android API parity.\n";
    if(!report.empty()){
      std::ofstream out(report,std::ios::binary|std::ios::trunc);
      if(!out)throw std::runtime_error("cannot create JSON report");
      out<<dyx::h6e::json(a);out.flush();
      if(!out)throw std::runtime_error("failed writing JSON report");
    }
    return enforce?2:0;
  }catch(const std::exception& e){std::cerr<<"H6E1 AUDIT ERROR: "<<e.what()<<'\n';return 1;}
}
