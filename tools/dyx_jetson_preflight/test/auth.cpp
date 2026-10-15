#include "verified_http.hpp"
#include <cassert>
#include <string>
int main(){
  using namespace dyx::preflight;
  std::vector<unsigned char> k(32,'K');
  SignedRequest r{"operator_01","abcdef0123456789",1};
  const auto good="GET /health HTTP/1.1\r\nHost: localhost\r\nX-DYX-Session: "+r.session+
    "\r\nX-DYX-Sequence: 1\r\nX-DYX-Nonce: "+r.nonce+
    "\r\nAuthorization: DYX-HMAC "+hex_hmac(k,canonical(r))+"\r\n\r\n";
  SignedRequest out;
  assert(verify_request(good,k,out)&&out.sequence==1);
  auto bad=good;bad.replace(bad.find("Sequence: 1"),11,"Sequence: 2");
  assert(!verify_request(bad,k,out));
  assert(!verify_request(good+"BAD",k,out));
  assert(!verify_request(good.substr(0,good.size()-2)+"Transfer-Encoding: chunked\r\n\r\n",k,out));
  assert(!digits_u64("18446744073709551616",out.sequence));
  assert(!digits_u64("01",out.sequence));
  assert(!constant_hex_equal(std::string(64,'z'),std::string(64,'z')));
}
