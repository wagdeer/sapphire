#include "backend/visual/feature/visual_features.hpp"
#include "common/camera/camera.hpp"
#include "backend/visual/feature/visual_tracker.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <unistd.h>

using namespace sapphire;
namespace {
void check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
template<class F> void invalid(F f) { bool caught=false; try { f(); } catch(const std::invalid_argument &) { caught=true; } check(caught,"invalid camera contract must throw"); }
CameraParameters camera() {
  CameraParameters c; c.width=720; c.height=540;
  // Official Hilti 2022 cam0 numeric fixture, independent of future bag input.
  c.intrinsics={351.31400364193297,351.4911744656785,367.8522793375995,253.8402144980996};
  c.distortion_model="equidistant";
  c.distortion={-.03696737352869157,-.008917880497032812,.008912969593422046,-.0037685977496087313};
  c.camera_to_imu_rotation={1,0,0,0,1,0,0,0,1};c.camera_to_imu_translation={0,0,0};
  return c;
}
cv::Mat oracleMatrix(const CameraParameters &c) {
  return (cv::Mat_<double>(3,3) << c.intrinsics[0],0,c.intrinsics[2],0,c.intrinsics[1],c.intrinsics[3],0,0,1);
}
void projectionTest() {
  auto c=camera(); const auto K=oracleMatrix(c);
  std::vector<cv::Point2d> rays={{0,0},{.01,-.02},{.7,.5},{-1.4,.6},{.1,-1.1}};
  for(bool zero:{false,true}) {
    if(zero)c.distortion={0,0,0,0};
    std::vector<cv::Point2d> expected; cv::fisheye::distortPoints(rays,expected,K,c.distortion);
    std::vector<cv::Point2f> raw; for(std::size_t i=0;i<rays.size();++i) {
      check(cv::norm(cv::Point2d(CameraModel(c).distortNormalized(cv::Point2f(rays[i])))-expected[i])<1e-4,"angular projection matches independent OpenCV oracle");
      raw.emplace_back(expected[i]);
    }
    std::vector<cv::Point2f> normalized,rectified;
    CameraModel(c).undistortPixels(raw,normalized);CameraModel(c).undistortPixels(raw,rectified,true);
    for(std::size_t i=0;i<rays.size();++i) {
      check(cv::norm(cv::Point2d(normalized[i])-rays[i])<2e-6,"center/off-axis inverse roundtrip");
      check(cv::norm(cv::Point2d(rectified[i])-cv::Point2d(c.intrinsics[0]*rays[i].x+c.intrinsics[2],c.intrinsics[1]*rays[i].y+c.intrinsics[3]))<.001,"rectified pixel retains pinhole bearing");
    }
    if(zero)check(cv::norm(expected[3]-cv::Point2d(c.intrinsics[0]*rays[3].x+c.intrinsics[2],c.intrinsics[1]*rays[3].y+c.intrinsics[3]))>50,"zero fisheye coefficients do not mean a rectified pinhole");
  }
  std::vector<cv::Point2f> out;CameraModel(c).undistortPixels({{1e7f,1e7f}},out);
  check(!std::isfinite(out.front().x),"noninvertible/clipped points cannot provide a bearing");
  c.distortion_model="radtan";c.distortion={-.1,.02,.001,-.002,.003};
  std::vector<cv::Point3d> object; for(auto ray:rays)object.emplace_back(ray.x,ray.y,1);
  std::vector<cv::Point2d> expected;cv::projectPoints(object,cv::Vec3d(),cv::Vec3d(),K,c.distortion,expected);
  for(std::size_t i=0;i<rays.size();++i)check(cv::norm(cv::Point2d(CameraModel(c).distortNormalized(cv::Point2f(rays[i])))-expected[i])<1e-4,"legacy radial/tangential forward projection preserved");
}
void extrinsicTest() {
  auto c = camera();
  const Eigen::Matrix3d R = Eigen::AngleAxisd(.47, Eigen::Vector3d(1, 2, -3).normalized()).toRotationMatrix();
  const Eigen::Vector3d t(.3, -.8, .15), point(.2, .6, 4.1);
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) c.camera_to_imu_rotation[i*3+j] = R(i,j);
  c.camera_to_imu_translation = {t.x(), t.y(), t.z()};
  const CameraModel model(c);
  const Eigen::Isometry3d transform = model.cameraToImu().cast<double>();
  check((transform * point - (R * point + t)).norm() < 1e-6, "extrinsic maps camera into IMU with row-major calibration");
  check((transform.inverse() * (R * point + t) - point).norm() < 1e-6, "inverse returns IMU point into optical camera");
  c.intrinsics[0] *= 2;
  c.camera_to_imu_translation[0] += 1;
  check(model.matrix().at<float>(0,0) != float(c.intrinsics[0]) &&
        (model.cameraToImu().translation().cast<double>()-t).norm() < 1e-6, "model owns calibration independent of caller changes");
  c.camera_to_imu_rotation.clear();
  invalid([&] { CameraModel(c).cameraToImu(); });
  c = camera(); c.camera_to_imu_rotation[0] = 2;
  invalid([&] { CameraModel(c).cameraToImu(); });
  c = camera(); c.intrinsics.clear();
  invalid([&] { CameraModel model(c); });
}
void concreteModelsTest() {
  auto c = camera();
  Kb4CameraModel kb4(c);
  check(kb4.matrix().type() == CV_32FC1, "camera calibration stores float matrix");
  invalid([&] { PinholeCameraModel wrong(c); });
  float max_error = 0;
  for (int kind = 0; kind < 2; ++kind) {
    if (kind) { c.distortion_model = "radtan"; c.distortion = {-.1,.02,.001,-.002,.003}; }
    const CameraModel model(c);
    std::vector<cv::Point3d> points;
    for (int y = -8; y <= 8; ++y) for (int x = -12; x <= 12; ++x) points.emplace_back(x*.1, y*.1, 1.);
    std::vector<cv::Point2d> expected;
    if (kind) cv::projectPoints(points,cv::Vec3d(),cv::Vec3d(),oracleMatrix(c),c.distortion,expected);
    else cv::fisheye::projectPoints(points,expected,cv::Vec3d(),cv::Vec3d(),oracleMatrix(c),c.distortion);
    for (std::size_t i = 0; i < points.size(); ++i) {
      const Eigen::Vector3f point(float(points[i].x), float(points[i].y), 1.0F);
      const auto pixel = model.project(point);
      check(bool(pixel), "finite forward point projects");
      const float error = float(cv::norm(cv::Point2d(pixel->x(), pixel->y())-expected[i]));
      max_error = std::max(max_error, error);
      check(error < .0002F, "float projection agrees with independent double OpenCV oracle below .0002 pixels");
      const auto ray = model.unproject({pixel->x(),pixel->y()});
      check(ray && (point.normalized()-*ray).norm() < .0001F, "raw pixels return unit bearings");
    }
    check(!model.project(Eigen::Vector3f(0,0,0)) && !model.project(Eigen::Vector3f(1,0,-1)), "invalid depth cannot project");
    check(!model.unproject({NAN,0}), "nonfinite pixel cannot provide bearing");
  }
  c.distortion.clear(); PinholeCameraModel pinhole(c);
  const Eigen::Vector3f point(.3F,-.2F,2.0F);
  auto pixel = pinhole.project(point);
  check(pixel && (pinhole.rectifiedToBearing({pixel->x(),pixel->y()}).value()-point.normalized()).norm() < 1e-6F,
        "distortion-free pinhole bearing roundtrip");
  invalid([&] { Kb4CameraModel wrong(c); });
  std::cout << "float camera maximum projection error_px=" << max_error << '\n';
}
void validationTest() {
  auto c=camera();check(valid_camera_distortion(c),"official coefficient layout accepted");
  for(int type=0;type<4;++type) {
    auto bad=c;
    if(type==0)bad.distortion_model="fishye";
    if(type==1)bad.distortion.push_back(0);
    if(type==2)bad.distortion.clear();
    if(type==3)bad.distortion[0]=std::numeric_limits<double>::infinity();
    invalid([&]{VisualTracker tracker(bad,0);});
    SapphireParameters p;p.pose_graph.visual.enabled=true;p.pose_graph.visual.left=bad;
    invalid([&]{validate_parameters(p);});
  }
  c.distortion_model="radtan";invalid([&]{VisualTracker tracker(c,0);});
  char name[]="/tmp/sapphire-camera-model-XXXXXX";int fd=mkstemp(name);check(fd>=0,"temporary config");close(fd);
  std::ofstream file(name); file<<"[visual_loop]\nenabled=true\nmode=\"stereo\"\n";
  for(const auto *side:{"left","right"}) file<<"[visual_loop."<<side<<"]\nwidth=720\nheight=540\nintrinsics=[350,351,360,270]\ndistortion_model=\"equidistant\"\ndistortion=[0,0,0,0]\ncamera_to_imu_rotation=[1,0,0,0,1,0,0,0,1]\ncamera_to_imu_translation=[0,0,0]\n";
  file.close();const auto p=load_parameters(name);unlink(name);
  check(p.pose_graph.visual.left.distortion_model=="equidistant" && p.pose_graph.visual.right.distortion_model=="equidistant","both camera model TOML fields loaded");
}
void triangulationTest() {
  auto c=camera();std::array<VisualRayObservation,3> observations;
  const Eigen::Vector3d point(2.4,-.6,4);
  for(int i=0;i<3;++i) {
    auto &o=observations[i];o.timestamp=i;
    o.T_odom_camera=Eigen::Isometry3d::Identity();o.T_odom_camera.translation()=Eigen::Vector3d(.18*i,-.03*i,.02*i);
    o.T_odom_camera.linear()=Eigen::AngleAxisd(.03*i,Eigen::Vector3d(1,2,-3).normalized()).toRotationMatrix();
    const Eigen::Vector3d pc=o.T_odom_camera.inverse()*point;
    std::vector<cv::Point3d> object={{pc.x(),pc.y(),pc.z()}};std::vector<cv::Point2d> raw;
    cv::fisheye::projectPoints(object,raw,cv::Vec3d(),cv::Vec3d(),oracleMatrix(c),c.distortion);
    std::vector<cv::Point2f> rays;CameraModel(c).undistortPixels({cv::Point2f(raw[0])},rays);
    o.normalized_pixel=Eigen::Vector2d(rays[0].x,rays[0].y);
  }
  const auto g=triangulateVisualTrack(observations,c.intrinsics[0],c.intrinsics[1],{});
  check(g && (g->position_camera-observations.back().T_odom_camera.inverse()*point).norm()<.001,"off-axis distorted pixels yield correct metric optical-camera geometry");
}
cv::Mat planeTexture() {cv::Mat t(1536,1536,CV_8UC1);cv::RNG rng(74823);rng.fill(t,cv::RNG::UNIFORM,0,256);cv::GaussianBlur(t,t,cv::Size(3,3),.6);return t;}
cv::Mat render(const CameraParameters &c,const cv::Mat &texture,double x) {
  std::vector<cv::Point2f> raw,rays;
  for(int v=0;v<c.height;++v)for(int u=0;u<c.width;++u)raw.emplace_back(u,v);
  cv::fisheye::undistortPoints(raw,rays,oracleMatrix(c),c.distortion);
  cv::Mat map(c.height,c.width,CV_32FC2);
  for(int v=0;v<c.height;++v)for(int u=0;u<c.width;++u) {
    const auto &r=rays[v*c.width+u];map.at<cv::Vec2f>(v,u)={float(768+80*(4*r.x+x)),float(768+80*4*r.y)};
  }
  cv::Mat image;cv::remap(texture,image,map,cv::noArray(),cv::INTER_LINEAR,cv::BORDER_CONSTANT);return image;
}
void flowAndExtractorTest() {
  auto c=camera();const auto texture=planeTexture();VisualTracker tracker(c,0);
  int geometry=0,described=0;double max_error=0;std::vector<double> errors;
  for(int i=0;i<30;++i) {
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation().x()=.03*i;
    auto r=tracker.process(ImageMeas(i*.05,render(c,texture,pose.translation().x())),pose);
    if(r.keyframe) {++described;for(const auto &point:r.keyframe->points())if(point.geometry()) {++geometry;const double error=std::abs(point.geometry()->position_camera.z()-4);errors.push_back(error);max_error=std::max(max_error,error);}}
  }
  int retained=0;for(const auto &f:tracker.copySubmapEvidence())for(const auto &point:f.points())if(point.geometry())++retained;
  std::sort(errors.begin(),errors.end());
  const double median=errors.empty()?INFINITY:errors[errors.size()/2];
  const double p95=errors.empty()?INFINITY:errors[errors.size()*95/100];
  std::cout<<"depth median="<<median<<" p95="<<p95<<std::endl;
  std::cout<<"flow descriptions="<<described<<" geometry="<<geometry<<" retained="<<retained<<" max_error="<<max_error<<std::endl;
  // Render interpolation and subpixel LK add observation noise. Exact ray
  // conversion is checked separately; here require depth support with bounded
  // typical and tail error (1.25%, 3.75%, 10% of the known 4m plane).
  check(described>1 && described<30 && geometry>100 && retained>100 && median<.05 && p95<.15 && max_error<.4,"raw fisheye flow supplies bounded temporal depth and anchor enrichment");
  VisualLoopParameters cfg;cfg.enabled=true;cfg.left=c;const auto image=render(c,texture,0);
  const auto result=extractLoopFeatures(ImageMeas(1,image),cfg);
  std::vector<cv::KeyPoint> keypoints;cv::Mat descriptors;cv::ORB::create(cfg.max_features)->detectAndCompute(image,cv::noArray(),keypoints,descriptors);
  std::map<std::string,cv::Point2f> locations;
  for(int i=0;i<descriptors.rows;++i)locations.emplace(std::string((const char*)descriptors.ptr(i),32),keypoints[i].pt);
  check(result.points().size()>100,"legacy extractor accepts equidistant input");
  for(int i=0;i<result.descriptors().rows;++i) {
    const auto raw=locations.at(std::string((const char*)result.descriptors().ptr(i),32));
    std::vector<cv::Point2f> expected;cv::fisheye::undistortPoints(std::vector<cv::Point2f>{raw},expected,oracleMatrix(c),c.distortion,cv::noArray(),oracleMatrix(c));
    check((result.points()[i].pixel()-Eigen::Vector2f(expected[0].x,expected[0].y)).norm()<.001,"extractor keeps descriptor/pixel rows and rectifies exactly once");
  }
  std::cout<<"fisheye descriptions="<<described<<" geometry="<<geometry<<" retained="<<retained<<" max_depth_error_m="<<max_error<<'\n';
}
}
int main() {try {cv::setNumThreads(1);projectionTest();extrinsicTest();concreteModelsTest();validationTest();triangulationTest();flowAndExtractorTest();std::cout<<"visual camera tests passed\n";return 0;}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
