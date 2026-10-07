#include "backend/grid/rolling_grid.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace sapphire;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
int cell(const RollingGrid& grid, double x, double y) {
  const int col = std::floor((x-grid.originX())/grid.resolution());
  const int row = std::floor((y-grid.originY())/grid.resolution());
  if (col < 0 || row < 0 || col >= int(grid.width()) || row >= int(grid.width())) return -2;
  std::vector<std::int8_t> cells; grid.raster(cells); return cells[row*grid.width()+col];
}
int main() try {
  RollingGrid::Parameters p; p.extent=4; p.resolution=.1; p.max_age=2;
  RollingGrid grid(p);
  const Eigen::Vector3d origin(0,0,.8);
  check(grid.update({{1.05,.05,.5}},origin,1,1), "first scan");
  check(cell(grid,1.05,.05)==100 && cell(grid,.55,.05)==0, "hit endpoint and free ray");
  grid.update({{1.05,.05,.5},{1.85,.05,0}},origin,1.1,1);
  check(cell(grid,1.05,.05)==100, "same-scan hit dominates crossing ground ray");
  for(int i=0;i<4;++i) grid.update({{1.85,.05,0}},origin,1.2+i*.1,1);
  check(cell(grid,1.05,.05)==0, "repeated free observations clear old obstacle");
  grid.update({{1.05,.05,.5}},origin,1.6,1);
  check(cell(grid,1.05,.05)==100, "new obstacle immediately overrides accumulated old free evidence");
  grid.update({},origin,4,1);
  check(cell(grid,1.05,.05)==-1, "expired observations become unknown");
  grid.update({{1.05,.05,.5}},origin,4.1,1);
  grid.update({},Eigen::Vector3d(.55,-.35,.8),4.2,1);
  check(cell(grid,1.05,.05)==100, "window motion preserves shared world cells across negative coordinates");
  grid.update({},Eigen::Vector3d(50,-70,.8),4.3,1);
  check(cell(grid,50,-70)==-1 && cell(grid,1.05,.05)==-2, "large jump discards previous window");
  grid.update({{51.05,-69.95,.5}},Eigen::Vector3d(50,-70,.8),4.4,1);
  check(cell(grid,51.05,-69.95)==100, "new window observation");
  check(!grid.update({},origin,4,1), "out-of-order scan cannot rewind a live domain");
  grid.update({},origin,.1,2);
  check(cell(grid,1.05,.05)==-1, "new session resets history and accepts its own clock");
  grid.update({{10000,.05,.5},{std::numeric_limits<double>::quiet_NaN(),0,0}},origin,.2,2);
  check(cell(grid,1.95,.05)==0, "out-of-window return clips a free ray without invented boundary hit");
  const auto bytes=grid.residentBytes();
  for(int i=0;i<20000;++i) grid.update({},Eigen::Vector3d(i*.031,-i*.047,.8),1+i*.1,2);
  check(grid.residentBytes()==bytes && bytes<1024*1024, "constant resident buffers over prolonged movement");
  p.raytrace=false; RollingGrid merged(p);
  merged.update({{1.05,.05,.5}},origin,1,1);
  check(cell(merged,1.05,.05)==100 && cell(merged,.55,.05)==-1, "unknown merged origins never invent free rays");
  bool invalid=false;try{p.resolution=1e-300;RollingGrid bad(p);}catch(const std::exception&){invalid=true;}
  check(invalid,"reject unrepresentable grid before allocation");
  std::cout<<"PASS local rolling window, clearing, expiry, session identity and bounded storage\n";
  return 0;
} catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
