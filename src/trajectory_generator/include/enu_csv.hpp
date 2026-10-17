#pragma once
#include "dyx_rpp_cpp17_core/primitives.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dyx::trajectory {
inline std::string trim(std::string s) {
  auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return {};
  auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}
inline std::vector<std::string> fields(const std::string &line) {
  std::vector<std::string> v;
  std::string cur;
  bool quoted = false;
  for (std::size_t i=0; i<line.size(); ++i) {
    char c=line[i];
    if (c=='"') {
      if (quoted && i+1<line.size() && line[i+1]=='"') {cur+='"'; ++i;}
      else quoted=!quoted;
    } else if (c==',' && !quoted) {v.push_back(trim(cur));cur.clear();}
    else cur+=c;
  }
  if(quoted) throw std::invalid_argument("CSV unterminated quotation");
  v.push_back(trim(cur));
  return v;
}
inline double number(const std::string &v) {
  if(v.empty()) throw std::invalid_argument("CSV empty coordinate");
  char *end=nullptr;
  errno=0;
  const double n=std::strtod(v.c_str(),&end);
  if(errno==ERANGE || end==v.c_str() || *end!='\0' || !std::isfinite(n) || std::abs(n)>10000.0)
    throw std::invalid_argument("CSV coordinate invalid or outside local ENU limits");
  return n;
}
inline std::vector<dyx::rpp::Point2D> load_local_enu_csv(const std::string &filename) {
  if(filename.empty()) throw std::invalid_argument("Local ENU CSV path empty");
  std::ifstream file(filename);
  if(!file) throw std::runtime_error("Cannot read local ENU CSV");
  std::string line;
  if(!std::getline(file,line)) throw std::invalid_argument("CSV header missing");
  auto header=fields(line);
  if(!header.empty() && header[0].size()>=3 &&
     static_cast<unsigned char>(header[0][0])==0xef &&
     static_cast<unsigned char>(header[0][1])==0xbb &&
     static_cast<unsigned char>(header[0][2])==0xbf) header[0].erase(0,3);
  int east=-1,north=-1;
  for(std::size_t i=0;i<header.size();++i){
    if(header[i]=="east_m" || header[i]=="x_m") east=static_cast<int>(i);
    if(header[i]=="north_m" || header[i]=="y_m") north=static_cast<int>(i);
    if(header[i]=="latitude" || header[i]=="longitude" || header[i]=="lat" || header[i]=="lon")
      throw std::invalid_argument("Geodetic CSV cannot be interpreted as ENU: PX4 origin conversion pending");
  }
  if(east<0 || north<0 || east==north) throw std::invalid_argument("CSV requires east_m,north_m or x_m,y_m columns");
  std::vector<dyx::rpp::Point2D> result;
  while(std::getline(file,line)){
    if(trim(line).empty()) continue;
    const auto row=fields(line);
    if(row.size()!=header.size()) throw std::invalid_argument("CSV row has wrong column count");
    result.push_back({number(row[static_cast<std::size_t>(east)]), number(row[static_cast<std::size_t>(north)])});
    if(result.size()>10000) throw std::invalid_argument("CSV marking count exceeds 10000");
  }
  if(!file.eof()) throw std::runtime_error("CSV read failed");
  if(result.size()<2) throw std::invalid_argument("CSV needs at least two marking points");
  return result;
}
}
