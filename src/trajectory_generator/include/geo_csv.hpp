#pragma once
#include "enu_csv.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>
namespace dyx::trajectory {
struct GeoPoint { double latitude_deg,longitude_deg; };
struct GeoMission { std::vector<GeoPoint> survey; std::vector<dyx::rpp::Point2D> enu; };
inline void check_geo(double lat,double lon) {
  if(!std::isfinite(lat)||!std::isfinite(lon)||std::abs(lat)>90.0||std::abs(lon)>180.0)
    throw std::invalid_argument("Invalid geographic latitude or longitude");
}
inline double geographic_number(const std::string& s){
  if(s.empty()) throw std::invalid_argument("Empty geographic field");
  char* end=nullptr;errno=0;const double v=std::strtod(s.c_str(),&end);
  if(errno==ERANGE||end==s.c_str()||*end!='\0'||!std::isfinite(v))
    throw std::invalid_argument("Invalid geographic number");
  return v;
}
// Mirrors archived Python localization_frame.py: PX4 MapProjection radius=6371000 m.
// Output East/North equals MAVROS NED->ENU horizontal conversion.
inline dyx::rpp::Point2D project_geo(GeoPoint origin,GeoPoint target){
  check_geo(origin.latitude_deg,origin.longitude_deg);
  check_geo(target.latitude_deg,target.longitude_deg);
  constexpr double pi=3.1415926535897932384626433832795;
  constexpr double radius=6371000.0;
  const double d=pi/180.0;
  const double lat=target.latitude_deg*d,lon=target.longitude_deg*d;
  const double rlat=origin.latitude_deg*d,rlon=origin.longitude_deg*d;
  const double sd=std::sin(lat),cd=std::cos(lat),sr=std::sin(rlat),cr=std::cos(rlat);
  const double dl=lon-rlon,cl=std::cos(dl);
  const double arg=std::clamp(sr*sd+cr*cd*cl,-1.0,1.0);
  const double angle=std::acos(arg);
  const double scale=(angle==0.0)?1.0:angle/std::sin(angle);
  if(!std::isfinite(scale)) throw std::invalid_argument("Geographic projection singularity");
  const double north=scale*(cr*sd-sr*cd*cl)*radius;
  const double east=scale*cd*std::sin(dl)*radius;
  if(!std::isfinite(north)||!std::isfinite(east)||std::abs(north)>10000.0||std::abs(east)>10000.0)
    throw std::invalid_argument("Projected mission outside 10 km local bounds");
  return {east,north};
}
inline GeoMission load_geographic_csv(const std::string& filename,GeoPoint origin){
  check_geo(origin.latitude_deg,origin.longitude_deg);
  if(filename.empty()) throw std::invalid_argument("Empty geographic CSV file");
  std::ifstream file(filename);if(!file)throw std::runtime_error("Cannot open geographic CSV");
  std::string line;if(!std::getline(file,line))throw std::invalid_argument("Geographic CSV missing header");
  auto header=fields(line);
  if(!header.empty()&&header[0].size()>=3&&
    static_cast<unsigned char>(header[0][0])==0xef&&
    static_cast<unsigned char>(header[0][1])==0xbb&&
    static_cast<unsigned char>(header[0][2])==0xbf)header[0].erase(0,3);
  int lat=-1,lon=-1;
  for(std::size_t i=0;i<header.size();++i){
    if(header[i]=="latitude"||header[i]=="lat"||header[i]=="gps_lat"||header[i]=="gps_latitude"){
      if(lat>=0) throw std::invalid_argument("Duplicate latitude column");
      lat=static_cast<int>(i);
    }
    if(header[i]=="longitude"||header[i]=="lon"||header[i]=="lng"||header[i]=="gps_lon"||header[i]=="gps_longitude"){
      if(lon>=0) throw std::invalid_argument("Duplicate longitude column");
      lon=static_cast<int>(i);
    }
  }
  if(lat<0||lon<0)throw std::invalid_argument("Geographic CSV requires latitude,longitude");
  GeoMission result;
  while(std::getline(file,line)){
    if(trim(line).empty())continue;
    const auto row=fields(line);
    if(row.size()!=header.size())throw std::invalid_argument("Geographic CSV row column mismatch");
    GeoPoint p{geographic_number(row[static_cast<std::size_t>(lat)]),geographic_number(row[static_cast<std::size_t>(lon)])};
    check_geo(p.latitude_deg,p.longitude_deg);
    result.survey.push_back(p);
    result.enu.push_back(project_geo(origin,p));
    if(result.enu.size()>10000)throw std::invalid_argument("Geographic mission exceeds 10000 marks");
  }
  if(!file.eof())throw std::runtime_error("Geographic CSV read failure");
  if(result.enu.size()<2)throw std::invalid_argument("Geographic mission requires two marks");
  return result;
}
inline std::string survey_json(const std::vector<GeoPoint>& pts){
  std::ostringstream out;out<<std::setprecision(15)<<"[";
  for(std::size_t i=0;i<pts.size();++i){if(i)out<<",";out<<"{\"latitude\":"<<pts[i].latitude_deg
    <<",\"longitude\":"<<pts[i].longitude_deg<<"}";}
  out<<"]";return out.str();
}
}
