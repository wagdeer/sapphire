// Read compatibility remains after removal of the old geometry-fusion experiment.
#include "database/compression.hpp"
#include "common/se3_state.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace sapphire;
void require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
void legacy_support_codec() {
  auto ordinary=std::make_shared<GaussianCloud>(1); auto& point=ordinary->front(); point.N=8; point.mean={1,2,3}; point.covariance=Eigen::Vector3f(.01,.02,.03).asDiagonal(); point.radius=.5; point.voxel_key={1,2,3,0};GaussianCloud historical{ordinary->front()};
  auto support=std::make_shared<GaussianSupport>();
  support->covariance_lower=historical.front().covariance.cast<double>();
  support->plane_upper=support->covariance_lower/.0025;
  support->box_axes=Eigen::Matrix3d::Identity();support->box_center=historical.front().mean.cast<double>();support->box_half=Eigen::Vector3d(.01,.02,.03);
  historical.front().support=support;
  auto archive=lidar::encode(historical);require(std::memcmp(archive.data,"SAPLID02",8)==0,"manual historical fixture uses support archive version");
  auto metadata=lidar::decoded_size(archive.data,std::min(size_t(24),archive.total()),archive.data+archive.total()-12,12);
  require(metadata.points==1 && metadata.supports==1 && metadata.bytes()==sizeof(GaussianPoint)+sizeof(GaussianSupport),"legacy support size admitted before decode");
  auto restored=lidar::decode(archive);auto reencoded=lidar::encode(*restored);
  require(archive.total()==reencoded.total() && !std::memcmp(archive.data,reencoded.data,archive.total()),"legacy support archive roundtrips all fields bit-exactly");
  const auto pose=SE3f::fromRotationTranslation(Eigen::AngleAxisf(.3f,Eigen::Vector3f::UnitZ()).toRotationMatrix(),Eigen::Vector3f(.5f,-.4f,.2f)); auto transformed=std::make_shared<GaussianCloud>(); for(const auto& p:*restored) transformed->push_back(p.transformed(pose.rotationMatrix().cast<double>(),pose.translationVector().cast<double>()));const auto& after=*transformed->front().support;
  const Eigen::Matrix3d R=pose.rotationMatrix().cast<double>();
  require(after.covariance_lower.isApprox(R*support->covariance_lower*R.transpose(),1e-12) && after.plane_upper.isApprox(R*support->plane_upper*R.transpose(),1e-12) && after.box_center.isApprox(R*support->box_center+pose.translationVector().cast<double>(),1e-12),"historical codec geometry retains rigid-transform semantics");
  auto rejects=[](auto&& action){try{action();return false;}catch(const std::exception&){return true;}};
  auto bad=archive.clone();bad.data[8]^=1;require(rejects([&]{lidar::decode(bad);}),"inconsistent legacy count rejected");
  require(rejects([&]{lidar::decode(archive.colRange(0,23));}),"truncated legacy support archive rejected");
  auto raw=uncompressData(archive.colRange(24,archive.cols));
  auto damaged=[&](cv::Mat data){auto compressed=compressData(data);cv::Mat a(1,24+compressed.cols,CV_8UC1);std::memcpy(a.data,archive.data,24);std::memcpy(a.data+24,compressed.data,compressed.total());return a;};
  auto flags=raw.clone();flags.data[84]=4;require(rejects([&]{lidar::decode(damaged(flags));}),"invalid historical record flags rejected");
  auto covariance=raw.clone();double negative=-1;std::memcpy(covariance.data+85,&negative,sizeof(double));require(rejects([&]{lidar::decode(damaged(covariance));}),"nonpositive historical covariance rejected");
  auto copy=restored->front();require(rejects([&]{copy.regularize();}),"regularization cannot erase historical scatter");require(rejects([&]{copy+=ordinary->front();}),"ordinary averaging cannot discard old support");
  auto unchanged=lidar::encode(*restored);require(archive.total()==unchanged.total() && !std::memcmp(archive.data,unchanged.data,archive.total()),"support refusal leaves historical input intact");
}

int main() try { legacy_support_codec(); std::cout<<"PASS historical Gaussian archive compatibility\n"; } catch(const std::exception& e){ std::cerr<<e.what()<<"\n"; return 1; }
