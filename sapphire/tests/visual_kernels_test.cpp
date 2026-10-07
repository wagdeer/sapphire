#include "backend/visual/utils/gray_image.hpp"
#include "backend/visual/feature/fast_detector.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iostream>
using namespace sapphire;
int main() {
 cv::setNumThreads(2); cv::RNG rng(74);
 for (int width : {2,6,16,18,34,720}) {
  cv::Mat buffer(546,width+9,CV_8UC1); rng.fill(buffer,cv::RNG::UNIFORM,0,256);
  cv::Mat input=buffer(cv::Rect(3,3,width,540)), expected;
  cv::resize(input,expected,{width/2,270},0,0,cv::INTER_AREA);
  assert(cv::norm(expected,halfGrayArea(input),cv::NORM_INF)==0);
 }
 for (int width : {7,17,47,360}) {
  cv::Mat buffer(99,width+11,CV_8UC1);rng.fill(buffer,cv::RNG::UNIFORM,0,256);
  auto input=buffer(cv::Rect(2,3,width,90));
  std::vector<cv::KeyPoint> reference, actual;cv::FAST(input,reference,15,true);
  std::array<std::vector<std::uint8_t>,3> scores;std::array<std::vector<int>,3> corners;
  visual::fast_detail::detect_9_16_cell_nms(input,15,scores,corners,{3,3,width-6,84},width,90,1,
    [&](int x,int y,int score,int){actual.emplace_back(float(x),float(y),7,-1,float(score));});
  assert(reference.size()==actual.size());
  for (size_t i=0;i<actual.size();++i) assert(actual[i].pt==reference[i].pt && actual[i].response==reference[i].response);
 }
 std::cout << "visual kernels PASS\n";
}
