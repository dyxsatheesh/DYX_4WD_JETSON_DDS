#include "dyx_h6e/route_inventory.hpp"
#include <cassert>
#include <iostream>
int main(int argc,char**argv){
  if(argc!=2)return 64;
  try{
    const auto a=dyx::h6e::scan(argv[1]);
    if(a.legacy.size()!=71 || a.native.size()!=8 || a.exact_matches!=0 ||
       a.unmapped.size()!=71 || !a.socketio_declared || a.production_ready)
      throw std::runtime_error("route inventory drift; review API before release");
    std::cout<<"H6E1 repository API baseline: 71 legacy, 8 native, 0 exact matches\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
