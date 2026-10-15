#pragma once
// H5W: Real local-only HMAC verifier. Not TLS or Android-facing authentication.
#include <openssl/crypto.h>
#include <openssl/hmac.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>
namespace dyx::preflight {
struct SignedRequest {
  std::string session, nonce;
  std::uint64_t sequence{0};
};
inline bool safe_id(const std::string& v,std::size_t min,std::size_t max) {
  if(v.size()<min||v.size()>max) return false;
  for(unsigned char c:v) if(!std::isalnum(c)&&c!='-'&&c!='_')return false;
  return true;
}
inline bool digits_u64(const std::string& x,std::uint64_t& v){
  if(x.empty()||x.size()>20||x[0]=='0') return false;
  std::uint64_t n=0;
  for(char c:x){if(c<'0'||c>'9'||n>(UINT64_MAX-static_cast<unsigned>(c-'0'))/10)return false;
    n=n*10+static_cast<unsigned>(c-'0');}
  v=n;return n>0;
}
inline bool load_owned_secret(const std::string& path,std::vector<unsigned char>& key){
  key.clear();if(path.empty()||path[0]!='/')return false;
  int fd=open(path.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW);
  if(fd<0)return false;
  struct stat st{};
  bool ok=fstat(fd,&st)==0 && S_ISREG(st.st_mode) &&
          st.st_uid==geteuid() && (st.st_mode&0077)==0 &&
          st.st_size>=32 && st.st_size<=256;
  if(ok){key.resize(static_cast<size_t>(st.st_size));
    size_t offset=0;while(offset<key.size()){
      ssize_t n=read(fd,key.data()+offset,key.size()-offset);
      if(n<=0){ok=false;break;}offset+=static_cast<size_t>(n);}}
  close(fd);if(!ok){if(!key.empty())OPENSSL_cleanse(key.data(),key.size());key.clear();}
  return ok;
}
inline std::string hex_hmac(const std::vector<unsigned char>& key,
                            const std::string& content){
  if(key.size()<32||key.size()>256)return {};
  unsigned char mac[EVP_MAX_MD_SIZE]{};unsigned int len=0;
  if(!HMAC(EVP_sha256(),key.data(),static_cast<int>(key.size()),
           reinterpret_cast<const unsigned char*>(content.data()),content.size(),mac,&len))return {};
  const char* digits="0123456789abcdef";std::string out;
  for(unsigned int i=0;i<len;++i){out.push_back(digits[mac[i]>>4]);out.push_back(digits[mac[i]&15]);}
  OPENSSL_cleanse(mac,sizeof(mac));return out;
}
inline bool constant_hex_equal(const std::string& a,const std::string& b){
  if(a.size()!=64||b.size()!=64)return false;
  unsigned char ca[64]{},cb[64]{};
  for(size_t i=0;i<64;++i){
    unsigned char x=static_cast<unsigned char>(a[i]);
    unsigned char y=static_cast<unsigned char>(b[i]);
    if(!std::isxdigit(x)||!std::isxdigit(y)||std::isupper(x)||std::isupper(y))return false;
    ca[i]=x;cb[i]=y;
  }
  return CRYPTO_memcmp(ca,cb,64)==0;
}
inline std::string canonical(const SignedRequest& r){
  return "GET\n/health\n"+r.session+"\n"+std::to_string(r.sequence)+"\n"+r.nonce;
}
inline bool verify_request(const std::string& http,
                           const std::vector<unsigned char>& key,SignedRequest& out){
  if(http.size()>2048||http.size()<15)return false;
  const auto cut=http.find("\r\n\r\n");
  if(cut==std::string::npos||cut+4!=http.size())return false; // no request body
  if(http.substr(0,http.find("\r\n"))!="GET /health HTTP/1.1")return false;
  std::map<std::string,std::string> headers;
  std::size_t pos=http.find("\r\n")+2;
  while(pos<cut){
    auto end=http.find("\r\n",pos);if(end==std::string::npos||end>cut)return false;
    std::string line=http.substr(pos,end-pos);
    auto sep=line.find(": ");if(sep==std::string::npos)return false;
    std::string name=line.substr(0,sep),value=line.substr(sep+2);
    if(name.empty()||value.empty()||value.size()>256)return false;
    for(auto& c:name){if(c>='A'&&c<='Z')c=static_cast<char>(c+('a'-'A'));}
    if(!headers.emplace(name,value).second)return false; // duplicate headers
    pos=end+2;
  }
  for(auto name:{"content-length","transfer-encoding","proxy-authorization","upgrade"})
    if(headers.count(name))return false;
  if(!headers.count("host")||!headers.count("x-dyx-session")||
     !headers.count("x-dyx-sequence")||!headers.count("x-dyx-nonce")||
     !headers.count("authorization"))return false;
  if(headers["host"]!="localhost"&&headers["host"]!="dyx-local")return false;
  SignedRequest req;
  req.session=headers["x-dyx-session"];
  req.nonce=headers["x-dyx-nonce"];
  if(!safe_id(req.session,8,128)||!safe_id(req.nonce,16,64)||
     !digits_u64(headers["x-dyx-sequence"],req.sequence))return false;
  const auto& auth=headers["authorization"];
  if(auth.compare(0,9,"DYX-HMAC ")!=0)return false;
  const auto mac=hex_hmac(key,canonical(req));
  if(!constant_hex_equal(auth.substr(9),mac))return false;
  out=std::move(req);return true;
}
} // namespace dyx::preflight
