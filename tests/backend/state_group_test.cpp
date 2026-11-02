#include "common/se3_state.hpp"
#include <database/trajectory.hpp>
#include <sstream>
#include <locale>
#include <iomanip>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template <typename Scalar>
void testStateGroupLieOperations() {
  using State = sapphire::SE3State<Scalar>;
  using Vector3 = typename State::Vector3;
  using Tangent6 = typename State::Tangent6;
  using Matrix4 = typename State::Matrix4;
  using Matrix6 = typename State::Matrix6;

  const Scalar tolerance = std::is_same_v<Scalar, float> ? Scalar(2e-4) : Scalar(1e-10);
  require(State().isNull(), "default StateGroup must be null");
  require(!State::identity().isNull() && State::identity().isIdentity(), "identity StateGroup is invalid");
  require(State::identity().isValid(), "identity StateGroup failed SE3 validation");

  Tangent6 tangent;
  tangent << Scalar(1.2), Scalar(-0.4), Scalar(2.1), Scalar(0.2), Scalar(-0.3), Scalar(0.1);
  const State state = State::exp(tangent);
  require((state.log() - tangent).norm() < tolerance, "SE3 exp/log round-trip failed");
  require((State::vee(State::wedge(tangent)) - tangent).norm() < tolerance, "SE3 wedge/vee round-trip failed");

  const Matrix4 identity = Matrix4::Identity();
  require((state.compose(state.inverse()).matrix() - identity).norm() < tolerance, "SE3 inverse/compose identity failed");
  require((State::identity().between(state).matrix() - state.matrix()).norm() < tolerance, "SE3 between failed");

  const Matrix6 left = State::leftJacobian(tangent);
  const Matrix6 inverseLeft = State::invLeftJacobian(tangent);
  const Matrix6 right = State::rightJacobian(tangent);
  const Matrix6 inverseRight = State::invRightJacobian(tangent);
  require((left * inverseLeft - Matrix6::Identity()).norm() < Scalar(10) * tolerance, "SE3 left Jacobian inverse failed");
  require((right * inverseRight - Matrix6::Identity()).norm() < Scalar(10) * tolerance, "SE3 right Jacobian inverse failed");

  Tangent6 smallTangent;
  smallTangent << Scalar(0.2), Scalar(-0.3), Scalar(0.4), Scalar(1e-6), Scalar(-2e-6), Scalar(3e-6);
  const Scalar smallTolerance = std::is_same_v<Scalar, float> ? Scalar(1e-6) : Scalar(1e-12);
  require((State::leftJacobian(smallTangent) * State::invLeftJacobian(smallTangent) - Matrix6::Identity()).norm() < smallTolerance,
          "SE3 small-angle Jacobian inverse failed");
  require((State::exp(smallTangent).log() - smallTangent).norm() < smallTolerance, "SE3 small-angle exp/log failed");

  const Tangent6 adjointTangent = state.adjoint() * tangent;
  const Matrix4 adjointLeft = State::wedge(adjointTangent);
  const Matrix4 adjointRight = state.matrix() * State::wedge(tangent) * state.inverse().matrix();
  require((adjointLeft - adjointRight).norm() < Scalar(10) * tolerance, "SE3 Adjoint identity failed");

  Tangent6 perturbation;
  perturbation << Scalar(1e-3), Scalar(-2e-3), Scalar(3e-3), Scalar(-2e-4), Scalar(1e-4), Scalar(3e-4);
  const State perturbed = state.compose(State::exp(perturbation));
  require((state.between(perturbed).log() - perturbation).norm() < Scalar(10) * tolerance, "right perturbation convention failed");

  const Scalar pi = std::acos(Scalar(-1));
  const Vector3 nearPi(Scalar(0), pi - Scalar(1e-4), Scalar(0));
  require((State::rotationExp(nearPi).rotationLog() - nearPi).norm() < Scalar(20) * tolerance, "SO3 near-pi exp/log failed");

  const Vector3 point(Scalar(0.4), Scalar(-1.5), Scalar(2.0));
  const Vector3 expected = state.rotationMatrix() * point + state.translationVector();
  require((state.transformPoint(point) - expected).norm() < tolerance, "point action failed");
  require((state.rotateVector(point) - state.rotationMatrix() * point).norm() < tolerance, "vector action failed");

  std::array<Scalar, 2> x{point.x(), Scalar(-0.3)};
  std::array<Scalar, 2> y{point.y(), Scalar(0.7)};
  std::array<Scalar, 2> z{point.z(), Scalar(1.1)};
  const Vector3 expectedSecond = state.transformPoint(Vector3(x[1], y[1], z[1]));
  state.transformPointsInPlace(x.data(), y.data(), z.data(), x.size());
  require((Vector3(x[0], y[0], z[0]) - expected).norm() < tolerance &&
              (Vector3(x[1], y[1], z[1]) - expectedSecond).norm() < tolerance,
          "batch point action failed");

  const State midpoint = State::identity().interpolate(Scalar(0.5), state);
  require((midpoint.translationVector() - Scalar(0.5) * state.translationVector()).norm() < tolerance, "translation interpolation failed");

  typename State::Coefficients layout{Scalar(1), Scalar(0), Scalar(0), Scalar(4), Scalar(0), Scalar(1),
                                      Scalar(0), Scalar(5), Scalar(0), Scalar(0), Scalar(1), Scalar(6)};
  const State layoutState(layout);
  require(layoutState.x() == Scalar(4) && layoutState.y() == Scalar(5) && layoutState.z() == Scalar(6), "array12 layout changed");

  layout[0] = std::numeric_limits<Scalar>::quiet_NaN();
  require(State(layout).isNull(), "non-finite StateGroup must be null");
}

using sapphire::PathConstructor;
using Pose = sapphire::SE3f;
Pose pose(float x, float y = 0, float yaw = 0) {
  return Pose::fromRotationTranslation(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix(), {x,y,0});
}

void testTrajectorySerialization() {
 using namespace sapphire;
 const double stamp=1700000000.123456;
 const OdometryRecord record{stamp, {0,-1,0,1.2345678901234567, 1,0,0,2, 0,0,1,3}};
 struct Comma : std::numpunct<char> { char do_decimal_point() const override { return ','; } };
 for (const auto format : {OdomFileFormat::Tum, OdomFileFormat::Txt}) {
   std::ostringstream stream;
   stream.imbue(std::locale(std::locale::classic(),new Comma));
   stream << std::fixed << std::setprecision(3);
   writeOdomFile(stream,format,{record},"{}");
   require(stream.precision()==3 && (stream.flags() & std::ios::fixed),"Writer changed caller formatting");
   require(std::use_facet<std::numpunct<char>>(stream.getloc()).decimal_point()==',',"Writer changed caller locale");
   std::istringstream input(stream.str());
   std::string line;std::getline(input,line);std::getline(input,line);
   double actual;input>>actual;require(actual==stamp,"Timestamp lost precision");
   std::vector<double> values;while(input>>actual)values.push_back(actual);
   if(format==OdomFileFormat::Txt) {
     require(values.size()==12,"TXT columns changed");
     for(size_t i=0;i<12;++i)require(values[i]==record.matrix[i],"TXT matrix changed");
   } else {
     require(values.size()==7,"TUM columns changed");
     require(values[0]==record.matrix[3] && values[1]==2 && values[2]==3,"TUM changed coordinate frame");
     Eigen::Quaterniond q(values[6],values[3],values[4],values[5]);
     Eigen::Matrix3d expected;expected<<0,-1,0,1,0,0,0,0,1;
     require((q.toRotationMatrix()-expected).norm()<1e-12,"TUM quaternion order changed");
   }
 }
 for(int mode=0;mode<5;++mode) {
   auto bad=record;
   if(mode==0)bad.stamp=std::numeric_limits<double>::infinity();
   if(mode==1)bad.matrix[3]=std::numeric_limits<double>::quiet_NaN();
   if(mode==2)bad.matrix.fill(0);
   if(mode==3)bad.matrix[0]=3;
   std::ostringstream output;bool rejected=false;
   try{writeOdomFile(output,OdomFileFormat::Txt, mode==4 ? std::vector<OdometryRecord>{} : std::vector<OdometryRecord>{record,bad});}
   catch(const std::invalid_argument&){rejected=true;}
   require(rejected && output.str().empty(),"Invalid trajectory produced partial output");
 }
 bool rejected=false;try{odometry_format("2");}catch(const std::invalid_argument&){rejected=true;}
 require(rejected,"Legacy numeric format accepted");
 require(odometry_format("tum")==OdomFileFormat::Tum && odometry_format("txt")==OdomFileFormat::Txt,"Named formats rejected");
 std::ostringstream failed;failed.setstate(std::ios::badbit);rejected=false;
 try{writeOdomFile(failed,OdomFileFormat::Tum,{record});}catch(const std::runtime_error&){rejected=true;}
 require(rejected,"Output failure ignored");
}

void testPathConstructor() {
 PathConstructor path;
 require(path.smooth().empty(),"New path not empty");
 for(int i=0;i<50;++i)path.observe(1,10,10+i*.02,pose(i*.03f,i==0||i==49?0:(i%2?.015f:-.015f),i*.01f));
 auto smoothed=path.smooth();require(smoothed.size()>=2&&smoothed.size()<=128,"Unbounded output");
 require((smoothed.front().pose.matrix()-pose(0).matrix()).norm()<1e-7,"Keyframe endpoint moved");
 require((smoothed.back().pose.matrix()-pose(49*.03f,0,.49f).matrix()).norm()<1e-7,"Current endpoint moved");
 double energy=0;for(size_t i=0;i<smoothed.size();++i){const auto&s=smoothed[i];require(s.pose.matrix().allFinite(),"Nonfinite spline");if(i)require(s.stamp>smoothed[i-1].stamp,"Nonmonotonic timestamps");energy+=s.pose.y()*s.pose.y();}
 require(energy/smoothed.size()<.015*.015*.65,"Spline failed to reduce lateral jitter");
 const auto correction=pose(5,-3,.7f);
 for(size_t i=1;i<smoothed.size();++i)require(std::abs(correction.compose(smoothed[i].pose).translationDistance(correction.compose(smoothed[i-1].pose))-smoothed[i].pose.translationDistance(smoothed[i-1].pose))<2e-6,"Loop correction distorted segment");
 const auto before=path.smooth();path.observe(1,10,10.1,pose(99));require(path.smooth().back().stamp==before.back().stamp,"Stale odom accepted");
 path.observe(1,11,11,pose(2));require(path.size()>1&&path.smooth().front().pose.x()==0,"Keyframe switch truncated the rolling window");
 path.observe(2,1,1,pose(8));require(path.size()==1&&path.smooth().front().pose.x()==8,"Session reset/time rollback kept old segment");
 for(int i=1;i<1000;++i)path.observe(2,1,1+i*.01,pose(8,0,.001f*i));
 require(path.size()<=2&&path.smooth().size()==1,"Stationary rotation created a spurious curve");
 require(std::abs(Eigen::AngleAxisf(path.smooth().back().pose.rotationMatrix()).angle()-.999f)<1e-5,"Stationary heading lost");
 path.clear();for(int i=0;i<20000;++i)path.observe(3,1,1+i*.01,pose(.01f*i,.05f*std::sin(.01f*i)));
 auto longpath=path.smooth();require(path.size()<=64&&longpath.size()<=128,"Long segment is unbounded");require(longpath.front().stamp>=longpath.back().stamp-5&&longpath.back().pose.x()==.01f*19999,"Rolling window exceeded duration or lost endpoint");
 path.clear();path.observe(4,1,1,pose(0));path.observe(4,1,2,pose(1));path.observe(4,1,3,pose(0));auto turn=path.smooth();require(turn.size()>2,"Returning path collapsed");for(const auto&s:turn)require(s.pose.matrix().allFinite(),"Turn spline invalid");
 path.observe(4,1,std::numeric_limits<double>::quiet_NaN(),pose(3));require(path.size()==3,"Invalid timestamp accepted");
 path.clear();
 for(int i=0;i<60;++i){float x=(i/3)*.021f+(i%3)*.000002f;path.observe(5,1,1+i*.02,pose(x,.003f*std::sin(float(i))));}
 auto dense=path.smooth();for(const auto&s:dense)require(s.pose.x()>-.001f&&s.pose.x()<.5f&&std::abs(s.pose.y())<.006f,"Near-duplicate odom caused spline overshoot");
 path.setDuration(2);for(int i=0;i<40;++i)path.observe(6,1,1+i*.1,pose(.05f*i));require(path.smooth().front().stamp>=2.9,"Configured duration ignored");
 bool rejected=false;try{path.setDuration(0);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"Invalid duration accepted");
 path.clear();require(path.smooth().empty(),"Reset did not clear path");
}

}  // namespace

int main() {
  testStateGroupLieOperations<float>();
  testStateGroupLieOperations<double>();
  testTrajectorySerialization();
  testPathConstructor();
  return 0;
}
