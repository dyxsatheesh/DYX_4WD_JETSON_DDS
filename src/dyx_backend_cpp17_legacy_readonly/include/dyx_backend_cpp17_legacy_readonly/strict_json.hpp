#pragma once
// H6E2A: bounded strict JSON validator. Never turns JSON into rover commands.
#include <cctype>
#include <cstddef>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::h6e2 {
struct JsonValue {
  enum class Type { null_value, boolean, number, string, object, array };
  Type type{Type::null_value};
  std::map<std::string, JsonValue> fields{};
  std::vector<JsonValue> items{};
  std::string scalar{};
  const JsonValue* find(const std::string& key) const {
    if (type != Type::object) return nullptr;
    const auto it=fields.find(key);
    return it == fields.end() ? nullptr : &it->second;
  }
  bool is(Type t) const {return type==t;}
};

inline bool utf8_valid(const std::string& s) {
  for (std::size_t i=0;i<s.size();) {
    const auto c=static_cast<unsigned char>(s[i]);
    if (c<0x80) {++i;continue;}
    unsigned n=0; unsigned cp=0;
    if (c>=0xc2 && c<=0xdf){n=1;cp=c&0x1f;}
    else if(c>=0xe0 && c<=0xef){n=2;cp=c&0x0f;}
    else if(c>=0xf0 && c<=0xf4){n=3;cp=c&0x07;}
    else return false;
    if (n>=s.size()-i) return false;
    for(unsigned j=1;j<=n;++j){
      auto b=static_cast<unsigned char>(s[i+j]);
      if((b&0xc0)!=0x80)return false;
      cp=(cp<<6)|(b&0x3f);
    }
    if((n==1 && cp<0x80)||(n==2 && cp<0x800)||(n==3 && cp<0x10000)||
       (cp>=0xd800&&cp<=0xdfff)||cp>0x10ffff)return false;
    i+=n+1;
  }
  return true;
}

class StrictJson final {
 public:
  explicit StrictJson(const std::string& text):s_(text) {}
  JsonValue parse() {
    if (s_.size()>262144 || !utf8_valid(s_)) throw std::invalid_argument("JSON_TOO_LARGE_OR_BAD_UTF8");
    whitespace();
    auto v=value(0);
    whitespace();
    if (pos_!=s_.size()) throw std::invalid_argument("JSON_TRAILING_DATA");
    return v;
  }
 private:
  [[noreturn]] static void bad() {throw std::invalid_argument("MALFORMED_OR_DUPLICATE_JSON");}
  void whitespace(){while(pos_<s_.size() && (s_[pos_]==' '||s_[pos_]=='\n'||s_[pos_]=='\r'||s_[pos_]=='\t'))++pos_;}
  bool eat(char c){if(pos_<s_.size()&&s_[pos_]==c){++pos_;return true;}return false;}
  void must(char c){if(!eat(c))bad();}
  void literal(const char* p){while(*p){if(pos_>=s_.size()||s_[pos_++]!=*p++)bad();}}
  static int digit(char c) {
    if(c>='0'&&c<='9') return c-'0';
    if(c>='a'&&c<='f') return c-'a'+10;
    if(c>='A'&&c<='F') return c-'A'+10;
    return -1;
  }
  unsigned hex4(){unsigned v=0;for(int k=0;k<4;++k){if(pos_>=s_.size())bad();int d=digit(s_[pos_++]);if(d<0)bad();v=(v<<4)|static_cast<unsigned>(d);}return v;}
  std::string string() {
    must('"'); std::string v;
    while(pos_<s_.size()) {
      unsigned char c=static_cast<unsigned char>(s_[pos_++]);
      if(c=='"')return v;
      if(c<0x20)bad();
      if(c!='\\'){v.push_back(static_cast<char>(c));continue;}
      if(pos_==s_.size())bad();
      const char e=s_[pos_++];
      if(e=='"'||e=='\\'||e=='/')v.push_back(e);
      else if(e=='b')v.push_back('\b'); else if(e=='f')v.push_back('\f');
      else if(e=='n')v.push_back('\n'); else if(e=='r')v.push_back('\r');
      else if(e=='t')v.push_back('\t');
      else if(e=='u'){
        unsigned cp=hex4();
        if(cp>=0xdc00&&cp<=0xdfff)bad();
        if(cp>=0xd800&&cp<=0xdbff){
          must('\\');must('u');const auto low=hex4();
          if(low<0xdc00||low>0xdfff)bad();
          cp=0x10000+((cp-0xd800)<<10)+(low-0xdc00);
        }
        // ASCII keys must compare exactly, including escaped representations.
        if(cp<0x80)v.push_back(static_cast<char>(cp));
        else if(cp<0x800){v.push_back(static_cast<char>(0xc0|(cp>>6)));v.push_back(static_cast<char>(0x80|(cp&63)));}
        else if(cp<0x10000){v.push_back(static_cast<char>(0xe0|(cp>>12)));v.push_back(static_cast<char>(0x80|((cp>>6)&63)));v.push_back(static_cast<char>(0x80|(cp&63)));}
        else {v.push_back(static_cast<char>(0xf0|(cp>>18)));v.push_back(static_cast<char>(0x80|((cp>>12)&63)));v.push_back(static_cast<char>(0x80|((cp>>6)&63)));v.push_back(static_cast<char>(0x80|(cp&63)));}
      }else bad();
    }
    bad();
  }
  std::string number(){
    const auto begin=pos_;
    (void)eat('-');
    if(eat('0')){if(pos_<s_.size()&&std::isdigit(static_cast<unsigned char>(s_[pos_])))bad();}
    else {if(pos_>=s_.size()||s_[pos_]<'1'||s_[pos_]>'9')bad();
      while(pos_<s_.size()&&std::isdigit(static_cast<unsigned char>(s_[pos_])))++pos_;}
    if(eat('.')){if(pos_>=s_.size()||!std::isdigit(static_cast<unsigned char>(s_[pos_])))bad();
      while(pos_<s_.size()&&std::isdigit(static_cast<unsigned char>(s_[pos_])))++pos_;}
    if(eat('e')||eat('E')){if(!eat('+'))(void)eat('-');
      if(pos_>=s_.size()||!std::isdigit(static_cast<unsigned char>(s_[pos_])))bad();
      while(pos_<s_.size()&&std::isdigit(static_cast<unsigned char>(s_[pos_])))++pos_;}
    return s_.substr(begin,pos_-begin);
  }
  JsonValue value(unsigned depth) {
    if(depth>24 || pos_>=s_.size())bad();
    JsonValue v;
    const char c=s_[pos_];
    if(c=='{') {
      v.type=JsonValue::Type::object; ++pos_;whitespace();
      if(eat('}'))return v;
      do{
        whitespace();if(pos_==s_.size()||s_[pos_]!='"')bad();
        auto key=string();whitespace();must(':');whitespace();
        auto next=value(depth+1);
        if(!v.fields.emplace(std::move(key),std::move(next)).second)bad();
        whitespace();if(eat('}'))return v;
        must(',');whitespace();
      }while(true);
    }
    if(c=='['){
      v.type=JsonValue::Type::array;++pos_;whitespace();if(eat(']'))return v;
      do{v.items.push_back(value(depth+1));whitespace();if(eat(']'))return v;must(',');whitespace();}while(true);
    }
    if(c=='"'){v.type=JsonValue::Type::string;v.scalar=string();return v;}
    if(c=='t'){literal("true");v.type=JsonValue::Type::boolean;v.scalar="true";return v;}
    if(c=='f'){literal("false");v.type=JsonValue::Type::boolean;v.scalar="false";return v;}
    if(c=='n'){literal("null");v.type=JsonValue::Type::null_value;return v;}
    if(c=='-'||(c>='0'&&c<='9')){v.type=JsonValue::Type::number;v.scalar=number();return v;}
    bad();
  }
  const std::string& s_;
  std::size_t pos_{0};
};
} // namespace dyx::h6e2
