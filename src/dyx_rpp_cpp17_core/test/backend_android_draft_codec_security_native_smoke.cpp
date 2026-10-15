#include "backend_android_draft_codec_fixture.hpp"
#include <limits>
int main() {
  NativeOfflineAndroidDraftHttpCodec codec;
  auto good=codec_proposal(kBackendMissionUploadPath,1);
  assert(codec.encode(good).valid);
  auto p=good;
  p.prepared=false;assert(!codec.encode(p).valid);
  p=good;p.method="GET";assert(!codec.encode(p).valid);
  p=good;p.path="/api/v1/mission/start";assert(!codec.encode(p).valid);
  p=good;p.sequence=0;assert(!codec.encode(p).valid);
  p=good;p.mission_run_id="RUN\\u0001";assert(!codec.encode(p).valid);
  p=good;p.hardware_command_emitted=true;assert(!codec.encode(p).valid);
  p=good;p.network_io_performed=true;assert(!codec.encode(p).valid);
  p=good;p.backend_authorization_required=false;assert(!codec.encode(p).valid);
  p=good;p.extension_distance_m=std::numeric_limits<double>::infinity();
  assert(!codec.encode(p).valid);
  p=good;p.marking_points[1].x=std::numeric_limits<double>::quiet_NaN();
  assert(!codec.encode(p).valid);
  p=good;p.marking_points.resize(1);assert(!codec.encode(p).valid);
  p=good;p.expected_revision=1;assert(!codec.encode(p).valid);
  p=codec_proposal(kBackendMissionUpdatePath,2);assert(!codec.encode(p).valid);
  p=codec_proposal(kBackendMissionLoadPath,3,2,std::string(64,'0'));
  p.marking_points={{0,0}};assert(!codec.encode(p).valid);
  p.marking_points.clear();assert(codec.encode(p).valid);
  assert(p.backend_authorization_required && !p.hardware_command_emitted);
}
