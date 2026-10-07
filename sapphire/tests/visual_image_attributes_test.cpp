#include <sys/wait.h>
#include <unistd.h>
#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include "backend/visual/feature/visual_keyframe.hpp"
#include "backend/storage/visual_observation_archive.hpp"
#include "backend/storage/memory.hpp"
#include "backend/graph/pose_graph.hpp"
using namespace sapphire;
void check(bool v,const char *m){if(!v)throw std::runtime_error(m);}
CameraParameters camera(){CameraParameters c;c.width=360;c.height=270;c.intrinsics={260,260,179.5,134.5};c.distortion={0,0,0,0,0};return c;}
cv::Mat pixels(){cv::Mat a(270,360,CV_8UC1);cv::RNG rng(81);rng.fill(a,cv::RNG::UNIFORM,0,256);return a;}
Eigen::Isometry3d pose(int id){auto t=Eigen::Isometry3d::Identity();t.linear()=(Eigen::AngleAxisd(.31,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-.12,Eigen::Vector3d::UnitY())).toRotationMatrix();t.translation()=Eigen::Vector3d(2+.1*id,-3,1);return t;}
VisualFrame image(int id,int cam){ImageMeas m(id+1,pixels());m.camera_id=cam;auto t=pose(id);t.translation()+=t.linear()*Eigen::Vector3d(.2,cam*.1,.05);return makeImageAttribute(m,camera(),t);}
SubmapFrame submap(int id){
 auto cloud=std::make_shared<GaussianCloud>();
 for(int i=0;i<120;++i){int x=i%5,y=(i/5)%6,z=i/30;GaussianPoint p;p.N=20;p.voxel_key.x=i;
  p.mean={.37f*x+.03f*y*y,.43f*y+.02f*x*z,.39f*z+.02f*x*y};p.covariance=Eigen::Vector3f(.001,.02,.04).asDiagonal();p.regularize();cloud->push_back(p);}
 cpu::VoxelMaps vox;vox.create_voxelmaps(cloud->size(),[&](size_t i){return (*cloud)[i].mean;});
 LioFrame lio;lio.timestamp=id+1;lio.T_odom_base=pose(id);lio.pcd=cloud;
 std::vector<VisualFrame> images;images.push_back(image(id,0));images.push_back(image(id,1));
 return SubmapFrame(id,std::move(lio),id,id,1,0,0,vox.release_data(),{{double(id+1),pose(id)}},NavigationPath{},std::move(images));
}
PoseGraphParameters config(){PoseGraphParameters p;p.scene_refresh=true;p.visual.enabled=true;p.visual.attributes_only=true;return p;}
void inspect(const std::string &path,int count){
 auto p=config();NaviMapParameters n;n.enabled=false;
 Memory m(path,{},p.storage,map_config_identity(p,n),"resume");int nodes=0;
 m.visitHistoricalNodes([&](int id,const auto&,const auto&,const auto&){++nodes;auto frames=m.loadVisualFrames(id-1);
  if(!m.sceneActive(id-1)){check(frames.empty(),"retired scene drops image payload");return;}
  check(frames.size()==2,"two image attributes retained");
  for(auto &f:frames){check(f.has_image()&&!f.has_features()&&f.points().empty()&&f.descriptors().empty(),"image only, no features");
   auto decoded=cv::imdecode(f.image_png(),cv::IMREAD_UNCHANGED);check(cv::norm(decoded,pixels(),cv::NORM_INF)==0,"saved pixels survive separate process");
   const auto bytes=database_detail::packVisualObservation(f);check(bytes[4]==4,"image payload version4");}});
 check(nodes==count,"graph node count after continuation");check(m.historicalLinks().size()==std::size_t(count-1),"attachment and odometry graph factors");
}
void child(const char *exe,const std::string &path,const char *mode){pid_t pid=fork();check(pid>=0,"fork");if(!pid){execl(exe,exe,mode,path.c_str(),nullptr);_exit(127);}int status;check(waitpid(pid,&status,0)==pid&&WIFEXITED(status)&&WEXITSTATUS(status)==0,"cross-process storage phase");}
int main(int argc,char **argv){try{
 cv::setNumThreads(2);
 if(argc==3){auto p=config();NaviMapParameters n;n.enabled=false;std::string mode=argv[1],path=argv[2];
  if(mode=="write"){PoseGraphBackend b(p,n,path,{});b.addFrame(submap(0));b.finish();}
  else if(mode=="continue"){inspect(path,1);p.map_mode="resume";PoseGraphBackend b(p,n,path,{});
   auto result=b.attachFreshSession(submap(10),LocalGrid{},1,Eigen::Isometry3d::Identity(),77);if(result.status!=AttachmentStatus::Attached) throw std::runtime_error("image-only fresh attachment: "+result.message);
   b.beginContinuation(77);auto q=submap(11);LocalGrid g;check(b.submitContinuation(q,g,77)==ContinuationAdmission::Accepted,"image-only continuation");b.finish();}
  else inspect(path,3);return 0;
 }
 auto frame=image(2,1);const auto png=frame.image_png();auto encoded=database_detail::packVisualObservation(frame);
 auto restored=database_detail::readVisualObservation(frame.timestamp(),1,0,encoded.data(),encoded.size(),{});
 check(restored.image_png()==png&&restored.T_odom_camera()->matrix()==frame.T_odom_camera()->matrix(),"image/pose roundtrip");
 for(int corruption:{0,4,8,12,184,200}){auto bad=encoded;bad[corruption]^=1;bool failed=false;try{database_detail::readVisualObservation(3,1,0,bad.data(),bad.size(),{});}catch(...){failed=true;}check(failed,"corrupt image rejected");}
 auto badpng=png;badpng[16]=127;bool bad=false;try{frame.set_image_png(std::move(badpng));}catch(...){bad=true;}check(bad,"oversize PNG rejected before decoding");
 VisualLoopParameters v;v.left=v.right=camera();v.max_features=500;v.keyframe_selection=true;v.attributes_only=true;
 VisualKeyframeSelector left(v,0),right(v,1);auto raw=pixels();ImageMeas l(1,raw),rr(1.025,raw);rr.camera_id=1;
 check(left.evaluate(l,pose(0)).selected&&right.evaluate(rr,pose(0)).selected,"independent camera frame selection without stereo sync");left.accept();right.accept();
 cv::Mat blank(270,360,CV_8UC1,cv::Scalar(0));ImageMeas poor(2,blank);auto moved=pose(100);check(!left.evaluate(poor,moved).selected,"motion cannot bypass texture quality");
 left.beginSubmap();ImageMeas again(3,raw);check(left.evaluate(again,pose(0)).selected,"new submap can obtain representative");
 SubmapFrameBuffer buffer(4.,15.,20.);for(int i=0;i<12;++i){buffer.push_visual(image(i,0),3);buffer.push_visual(image(i,1),3);}check(buffer.buffered_visual_frame_count()==6,"bounded image residency");
 const std::string path="/tmp/sapphire-images-"+std::to_string(getpid())+".db";child(argv[0],path,"write");child(argv[0],path,"continue");child(argv[0],path,"read");
 std::filesystem::remove(path);std::cout<<"image attributes PASS\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
