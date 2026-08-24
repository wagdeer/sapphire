#include "PoseLib/pose_estimation.h"

#include "PoseLib/misc/essential.h"
#include "PoseLib/robust/optim/absolute.h"
#include "PoseLib/robust/optim/lm_impl.h"
#include "PoseLib/robust/optim/relative.h"
#include "PoseLib/robust/ransac_impl.h"
#include "PoseLib/robust/sampling.h"
#include "PoseLib/solvers/gp3p.h"
#include "PoseLib/solvers/p3p.h"
#include "PoseLib/solvers/relpose_5pt.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <utility>

namespace poselibplus {
namespace {

bool positiveFinite(double value)
{
    return value > 0.0 && std::isfinite(value);
}

bool finitePose(const CameraPose & pose)
{
    return pose.q.allFinite() &&
            pose.t.allFinite() &&
            pose.q.squaredNorm() > std::numeric_limits<double>::epsilon();
}

template<typename Points>
bool finitePoints(const Points & points)
{
    for(std::size_t i = 0; i < points.size(); ++i)
    {
        if(!points[i].allFinite())
        {
            return false;
        }
    }
    return true;
}

bool validRansacOptions(const poselib::RansacOptions & options)
{
    return options.max_iterations > 0 &&
            options.success_prob > 0.0 &&
            options.success_prob < 1.0 &&
            std::isfinite(options.success_prob) &&
            options.dyn_num_trials_mult > 0.0 &&
            std::isfinite(options.dyn_num_trials_mult);
}

double absoluteScore(
        const CameraPose & pose,
        const std::vector<Point2D> & points2D,
        const std::vector<Point3D> & points3D,
        double squaredThreshold,
        std::size_t * inlierCount,
        std::vector<char> * inliers = 0)
{
    *inlierCount = 0;
    if(!finitePose(pose) || !positiveFinite(squaredThreshold))
    {
        if(inliers)
        {
            inliers->assign(points2D.size(), 0);
        }
        return std::numeric_limits<double>::max();
    }

    double score = 0.0;
    if(inliers)
    {
        inliers->assign(points2D.size(), 0);
    }

    for(std::size_t i = 0; i < points2D.size(); ++i)
    {
        const Eigen::Vector3d projected = pose.apply(points3D[i]);
        double squaredError = std::numeric_limits<double>::infinity();
        if(projected.z() > 0.0)
        {
            squaredError = (projected.hnormalized() - points2D[i]).squaredNorm();
        }

        if(squaredError < squaredThreshold)
        {
            ++(*inlierCount);
            score += squaredError;
            if(inliers)
            {
                (*inliers)[i] = 1;
            }
        }
        else
        {
            score += squaredThreshold;
        }
    }
    return score;
}

double relativeScore(
        const CameraPose & pose,
        const std::vector<Point2D> & points2D1,
        const std::vector<Point2D> & points2D2,
        double squaredThreshold,
        std::size_t * inlierCount,
        std::vector<char> * inliers = 0)
{
    *inlierCount = 0;
    if(!finitePose(pose) ||
       pose.t.squaredNorm() <= std::numeric_limits<double>::epsilon() ||
       !positiveFinite(squaredThreshold))
    {
        if(inliers)
        {
            inliers->assign(points2D1.size(), 0);
        }
        return std::numeric_limits<double>::max();
    }

    double score = 0.0;
    Eigen::Matrix3d essential;
    poselib::essential_from_motion(pose, &essential);

    if(inliers)
    {
        inliers->assign(points2D1.size(), 0);
    }

    for(std::size_t i = 0; i < points2D1.size(); ++i)
    {
        const Eigen::Vector3d x1 = points2D1[i].homogeneous();
        const Eigen::Vector3d x2 = points2D2[i].homogeneous();
        const Eigen::Vector3d ex1 = essential * x1;
        const Eigen::Vector3d etx2 = essential.transpose() * x2;
        const double constraint = x2.dot(ex1);
        const double denominator =
                ex1.head<2>().squaredNorm() + etx2.head<2>().squaredNorm();
        const double squaredError = denominator > 0.0?
                constraint * constraint / denominator :
                std::numeric_limits<double>::infinity();

        bool accepted = squaredError < squaredThreshold;
        if(accepted)
        {
            accepted = poselib::check_cheirality(
                    pose,
                    x1.normalized(),
                    x2.normalized(),
                    0.01);
        }

        if(accepted)
        {
            ++(*inlierCount);
            score += squaredError;
            if(inliers)
            {
                (*inliers)[i] = 1;
            }
        }
        else
        {
            score += squaredThreshold;
        }
    }
    return score;
}

poselib::BundleOptions localOptimizationOptions(
        const PoseEstimationOptions & options,
        double threshold)
{
    poselib::BundleOptions bundle = options.bundle;
    bundle.loss_type = poselib::BundleOptions::TRUNCATED;
    bundle.loss_scale = threshold;
    bundle.max_iterations = options.lo_iterations;
    bundle.verbose = false;
    return bundle;
}

class ScaledAccumulator
{
public:
    ScaledAccumulator() :
        accumulator_(0),
        scale_(1.0)
    {
    }

    ScaledAccumulator(
            poselib::NormalAccumulator * accumulator,
            double scale) :
        accumulator_(accumulator),
        scale_(scale)
    {
    }

    template<int ResidualDim, int ParamsDim>
    void add_jacobian(
            const Eigen::Matrix<double, ResidualDim, 1> & residual,
            const Eigen::Matrix<double, ResidualDim, ParamsDim> & jacobian,
            double weight = 1.0)
    {
        const Eigen::Matrix<double, ResidualDim, 1> scaledResidual =
                scale_ * residual;
        const Eigen::Matrix<double, ResidualDim, ParamsDim> scaledJacobian =
                scale_ * jacobian;
        accumulator_->add_jacobian(
                scaledResidual,
                scaledJacobian,
                weight);
    }

    template<int ResidualDim>
    void add_residual(
            const Eigen::Matrix<double, ResidualDim, 1> & residual,
            double weight = 1.0)
    {
        const Eigen::Matrix<double, ResidualDim, 1> scaledResidual =
                scale_ * residual;
        accumulator_->add_residual(scaledResidual, weight);
    }

    double get_residual() const
    {
        return accumulator_->get_residual();
    }

private:
    poselib::NormalAccumulator * accumulator_;
    double scale_;
};

void refineAbsolute(
        const std::vector<Point2D> & points2D,
        const std::vector<Point3D> & points3D,
        CameraPose * pose,
        const poselib::BundleOptions & options)
{
    if(options.max_iterations == 0 || points2D.size() < 4)
    {
        return;
    }
    poselib::PinholeAbsolutePoseRefiner<> refiner(points2D, points3D);
    poselib::lm_impl(refiner, pose, options);
}

void refineRelative(
        const std::vector<Point2D> & points2D1,
        const std::vector<Point2D> & points2D2,
        CameraPose * pose,
        const poselib::BundleOptions & options)
{
    if(options.max_iterations == 0 || points2D1.size() < 6)
    {
        return;
    }
    poselib::PinholeRelativePoseRefiner<> refiner(points2D1, points2D2);
    poselib::lm_impl(refiner, pose, options);
}

class GeneralizedPinholeRefiner :
        public poselib::RefinerBase<CameraPose, poselib::NormalAccumulator>
{
public:
    GeneralizedPinholeRefiner(
            const std::vector<std::vector<Point2D> > & points2D,
            const std::vector<std::vector<Point3D> > & points3D,
            const std::vector<CameraPose> & cameraExtrinsics,
            const std::vector<double> & residualScales) :
        points2D_(points2D),
        points3D_(points3D),
        cameraExtrinsics_(cameraExtrinsics),
        residualScales_(residualScales)
    {
        num_params = 6;
    }

    double compute_residual(
            poselib::NormalAccumulator & accumulator,
            const CameraPose & pose)
    {
        for(std::size_t cameraIndex = 0;
            cameraIndex < points2D_.size();
            ++cameraIndex)
        {
            const CameraPose fullPose = compose(cameraExtrinsics_[cameraIndex], pose);
            for(std::size_t i = 0; i < points2D_[cameraIndex].size(); ++i)
            {
                const Eigen::Vector3d projected = fullPose.apply(points3D_[cameraIndex][i]);
                if(projected.z() > 0.0)
                {
                    const Eigen::Vector2d residual =
                            projected.hnormalized() - points2D_[cameraIndex][i];
                    const Eigen::Vector2d scaledResidual =
                            residualScales_[cameraIndex] * residual;
                    accumulator.add_residual(scaledResidual);
                }
            }
        }
        return accumulator.get_residual();
    }

    void compute_jacobian(
            poselib::NormalAccumulator & accumulator,
            const CameraPose & pose)
    {
        for(std::size_t cameraIndex = 0;
            cameraIndex < points2D_.size();
            ++cameraIndex)
        {
            if(points2D_[cameraIndex].empty())
            {
                continue;
            }
            const CameraPose fullPose = compose(cameraExtrinsics_[cameraIndex], pose);
            poselib::PinholeAbsolutePoseRefiner<
                    poselib::UniformWeightVector,
                    ScaledAccumulator> cameraRefiner(
                    points2D_[cameraIndex],
                    points3D_[cameraIndex]);
            ScaledAccumulator scaledAccumulator(
                    &accumulator,
                    residualScales_[cameraIndex]);
            cameraRefiner.compute_jacobian(scaledAccumulator, fullPose);
        }
    }

    CameraPose step(
            const Eigen::VectorXd & increment,
            const CameraPose & pose) const
    {
        CameraPose result;
        result.q = poselib::quat_step_post(
                pose.q,
                increment.block<3, 1>(0, 0));
        result.t = pose.t + pose.rotate(increment.block<3, 1>(3, 0));
        return result;
    }

    typedef CameraPose param_t;

private:
    static CameraPose compose(const CameraPose & lhs, const CameraPose & rhs)
    {
        return CameraPose(
                poselib::quat_multiply(lhs.q, rhs.q),
                lhs.t + lhs.rotate(rhs.t));
    }

    const std::vector<std::vector<Point2D> > & points2D_;
    const std::vector<std::vector<Point3D> > & points3D_;
    const std::vector<CameraPose> & cameraExtrinsics_;
    const std::vector<double> & residualScales_;
};

void refineGeneralized(
        const std::vector<std::vector<Point2D> > & points2D,
        const std::vector<std::vector<Point3D> > & points3D,
        const std::vector<CameraPose> & cameraExtrinsics,
        const std::vector<double> & residualScales,
        CameraPose * pose,
        const poselib::BundleOptions & options)
{
    if(options.max_iterations == 0)
    {
        return;
    }
    GeneralizedPinholeRefiner refiner(
            points2D,
            points3D,
            cameraExtrinsics,
            residualScales);
    poselib::lm_impl(refiner, pose, options);
}

class AbsoluteEstimator
{
public:
    AbsoluteEstimator(
            const std::vector<Point2D> & points2D,
            const std::vector<Point3D> & points3D,
            const PoseEstimationOptions & options) :
        sample_sz(3),
        num_data(points2D.size()),
        points2D_(points2D),
        points3D_(points3D),
        options_(options),
        sampler_(num_data, sample_sz, options.ransac),
        sample_(sample_sz),
        sampled2D_(sample_sz),
        sampled3D_(sample_sz)
    {
    }

    void generate_models(std::vector<CameraPose> * models)
    {
        sampler_.generate_sample(&sample_);
        for(std::size_t i = 0; i < sample_sz; ++i)
        {
            sampled2D_[i] = points2D_[sample_[i]].homogeneous().normalized();
            sampled3D_[i] = points3D_[sample_[i]];
        }
        poselib::p3p(sampled2D_, sampled3D_, models);
    }

    double score_model(const CameraPose & pose, std::size_t * inlierCount) const
    {
        return absoluteScore(
                pose,
                points2D_,
                points3D_,
                options_.max_error * options_.max_error,
                inlierCount);
    }

    void refine_model(CameraPose * pose) const
    {
        refineAbsolute(
                points2D_,
                points3D_,
                pose,
                localOptimizationOptions(options_, options_.max_error));
    }

    const std::size_t sample_sz;
    const std::size_t num_data;

private:
    const std::vector<Point2D> & points2D_;
    const std::vector<Point3D> & points3D_;
    const PoseEstimationOptions & options_;
    poselib::RandomSampler sampler_;
    std::vector<std::size_t> sample_;
    std::vector<Point3D> sampled2D_;
    std::vector<Point3D> sampled3D_;
};

class RelativeEstimator
{
public:
    RelativeEstimator(
            const std::vector<Point2D> & points2D1,
            const std::vector<Point2D> & points2D2,
            const PoseEstimationOptions & options) :
        sample_sz(5),
        num_data(points2D1.size()),
        points2D1_(points2D1),
        points2D2_(points2D2),
        options_(options),
        sampler_(num_data, sample_sz, options.ransac),
        sample_(sample_sz),
        sampled1_(sample_sz),
        sampled2_(sample_sz)
    {
    }

    void generate_models(std::vector<CameraPose> * models)
    {
        sampler_.generate_sample(&sample_);
        for(std::size_t i = 0; i < sample_sz; ++i)
        {
            sampled1_[i] = points2D1_[sample_[i]].homogeneous().normalized();
            sampled2_[i] = points2D2_[sample_[i]].homogeneous().normalized();
        }
        poselib::relpose_5pt(sampled1_, sampled2_, models);
    }

    double score_model(const CameraPose & pose, std::size_t * inlierCount) const
    {
        return relativeScore(
                pose,
                points2D1_,
                points2D2_,
                options_.max_error * options_.max_error,
                inlierCount);
    }

    void refine_model(CameraPose * pose) const
    {
        std::vector<char> approximateInliers;
        std::size_t approximateCount = 0;
        relativeScore(
                *pose,
                points2D1_,
                points2D2_,
                5.0 * options_.max_error * options_.max_error,
                &approximateCount,
                &approximateInliers);
        if(approximateCount < 6)
        {
            return;
        }

        std::vector<Point2D> inlierPoints1;
        std::vector<Point2D> inlierPoints2;
        inlierPoints1.reserve(approximateCount);
        inlierPoints2.reserve(approximateCount);
        for(std::size_t i = 0; i < approximateInliers.size(); ++i)
        {
            if(approximateInliers[i])
            {
                inlierPoints1.push_back(points2D1_[i]);
                inlierPoints2.push_back(points2D2_[i]);
            }
        }
        refineRelative(
                inlierPoints1,
                inlierPoints2,
                pose,
                localOptimizationOptions(options_, options_.max_error));
    }

    const std::size_t sample_sz;
    const std::size_t num_data;

private:
    const std::vector<Point2D> & points2D1_;
    const std::vector<Point2D> & points2D2_;
    const PoseEstimationOptions & options_;
    poselib::RandomSampler sampler_;
    std::vector<std::size_t> sample_;
    std::vector<Point3D> sampled1_;
    std::vector<Point3D> sampled2_;
};

class GeneralizedEstimator
{
public:
    GeneralizedEstimator(
            const std::vector<std::vector<Point2D> > & points2D,
            const std::vector<std::vector<Point3D> > & points3D,
            const std::vector<CameraPose> & cameraExtrinsics,
            const GeneralizedPoseEstimationOptions & options) :
        sample_sz(3),
        num_data(0),
        points2D_(points2D),
        points3D_(points3D),
        cameraExtrinsics_(cameraExtrinsics),
        options_(options),
        rng_(options.ransac.seed),
        sample_(sample_sz),
        centers_(sample_sz),
        bearings_(sample_sz),
        worldPoints_(sample_sz)
    {
        counts_.resize(points2D.size());
        cameraCenters_.resize(points2D.size());
        for(std::size_t cameraIndex = 0;
            cameraIndex < points2D.size();
            ++cameraIndex)
        {
            counts_[cameraIndex] = points2D[cameraIndex].size();
            num_data += counts_[cameraIndex];
            cameraCenters_[cameraIndex] = cameraExtrinsics[cameraIndex].center();
            if(counts_[cameraIndex] > 0)
            {
                activeCameras_.push_back(cameraIndex);
                for(std::size_t pointIndex = 0;
                    pointIndex < counts_[cameraIndex];
                    ++pointIndex)
                {
                    correspondences_.push_back(
                            std::make_pair(cameraIndex, pointIndex));
                }
            }
        }
    }

    void generate_models(std::vector<CameraPose> * models)
    {
        models->clear();
        if(activeCameras_.empty())
        {
            return;
        }

        bool sampled = false;
        if(options_.stratified_sampling && activeCameras_.size() > 1)
        {
            sampled = generateStratifiedSample();
        }
        else
        {
            sampled = generateUniformSample();
        }
        if(!sampled)
        {
            return;
        }

        bool singleCamera = true;
        for(std::size_t i = 1; i < sample_sz; ++i)
        {
            singleCamera = singleCamera &&
                    sample_[i].first == sample_[0].first;
        }

        for(std::size_t i = 0; i < sample_sz; ++i)
        {
            const std::size_t cameraIndex = sample_[i].first;
            const std::size_t pointIndex = sample_[i].second;
            centers_[i] = cameraCenters_[cameraIndex];
            bearings_[i] = cameraExtrinsics_[cameraIndex].derotate(
                    points2D_[cameraIndex][pointIndex].homogeneous().normalized());
            worldPoints_[i] = points3D_[cameraIndex][pointIndex];
        }

        if(singleCamera)
        {
            const std::size_t cameraIndex = sample_[0].first;
            std::vector<Point3D> cameraBearings(sample_sz);
            for(std::size_t i = 0; i < sample_sz; ++i)
            {
                cameraBearings[i] =
                        points2D_[cameraIndex][sample_[i].second].
                        homogeneous().normalized();
            }
            std::vector<CameraPose> cameraModels;
            poselib::p3p(cameraBearings, worldPoints_, &cameraModels);
            const CameraPose inverseExtrinsic =
                    cameraExtrinsics_[cameraIndex].inverse();
            models->reserve(cameraModels.size());
            for(std::size_t i = 0; i < cameraModels.size(); ++i)
            {
                models->push_back(compose(inverseExtrinsic, cameraModels[i]));
            }
        }
        else
        {
            poselib::gp3p(centers_, bearings_, worldPoints_, models);
        }
    }

    double score_model(const CameraPose & pose, std::size_t * inlierCount) const
    {
        *inlierCount = 0;
        std::size_t validCameras = 0;
        double score = 0.0;

        for(std::size_t cameraIndex = 0;
            cameraIndex < points2D_.size();
            ++cameraIndex)
        {
            const CameraPose fullPose =
                    compose(cameraExtrinsics_[cameraIndex], pose);
            std::size_t cameraInliers = 0;
            const double threshold = thresholdFor(cameraIndex);
            score += absoluteScore(
                    fullPose,
                    points2D_[cameraIndex],
                    points3D_[cameraIndex],
                    threshold * threshold,
                    &cameraInliers);
            *inlierCount += cameraInliers;
            if(!points2D_[cameraIndex].empty() &&
               cameraInliers >= options_.min_inliers_per_camera)
            {
                ++validCameras;
            }
        }

        if(validCameras < options_.min_valid_cameras)
        {
            *inlierCount = 0;
            return std::numeric_limits<double>::max();
        }
        return score;
    }

    void refine_model(CameraPose * pose) const
    {
        refineGeneralized(
                points2D_,
                points3D_,
                cameraExtrinsics_,
                residualScales(),
                pose,
                localOptimizationOptions(options_, options_.max_error));
    }

    double thresholdFor(std::size_t cameraIndex) const
    {
        return options_.max_errors.size() == points2D_.size()?
                options_.max_errors[cameraIndex] :
                options_.max_error;
    }

    static CameraPose compose(const CameraPose & lhs, const CameraPose & rhs)
    {
        return CameraPose(
                poselib::quat_multiply(lhs.q, rhs.q),
                lhs.t + lhs.rotate(rhs.t));
    }

    const std::size_t sample_sz;
    std::size_t num_data;

private:
    static std::size_t randomIndex(poselib::RNG_t & rng, std::size_t count)
    {
        return static_cast<std::uint32_t>(poselib::random_int(rng)) % count;
    }

    bool alreadySampled(
            const std::pair<std::size_t, std::size_t> & correspondence,
            std::size_t sampleCount) const
    {
        for(std::size_t i = 0; i < sampleCount; ++i)
        {
            if(sample_[i] == correspondence)
            {
                return true;
            }
        }
        return false;
    }

    bool sampleFromCamera(
            std::size_t cameraIndex,
            std::size_t sampleIndex)
    {
        const std::size_t count = counts_[cameraIndex];
        const std::size_t start = randomIndex(rng_, count);
        for(std::size_t offset = 0; offset < count; ++offset)
        {
            const std::pair<std::size_t, std::size_t> candidate(
                    cameraIndex,
                    (start + offset) % count);
            if(!alreadySampled(candidate, sampleIndex))
            {
                sample_[sampleIndex] = candidate;
                return true;
            }
        }
        return false;
    }

    bool generateStratifiedSample()
    {
        std::vector<std::size_t> cameraOrder = activeCameras_;
        const std::size_t distinctCount =
                std::min(sample_sz, cameraOrder.size());
        for(std::size_t i = 0; i < distinctCount; ++i)
        {
            const std::size_t swapIndex =
                    i + randomIndex(rng_, cameraOrder.size() - i);
            std::swap(cameraOrder[i], cameraOrder[swapIndex]);
            if(!sampleFromCamera(cameraOrder[i], i))
            {
                return false;
            }
        }

        for(std::size_t i = distinctCount; i < sample_sz; ++i)
        {
            std::vector<std::size_t> availableCameras;
            availableCameras.reserve(activeCameras_.size());
            for(std::size_t j = 0; j < activeCameras_.size(); ++j)
            {
                const std::size_t cameraIndex = activeCameras_[j];
                std::size_t used = 0;
                for(std::size_t k = 0; k < i; ++k)
                {
                    if(sample_[k].first == cameraIndex)
                    {
                        ++used;
                    }
                }
                if(used < counts_[cameraIndex])
                {
                    availableCameras.push_back(cameraIndex);
                }
            }
            if(availableCameras.empty())
            {
                return false;
            }
            const std::size_t cameraIndex = availableCameras[
                    randomIndex(rng_, availableCameras.size())];
            if(!sampleFromCamera(cameraIndex, i))
            {
                return false;
            }
        }
        return true;
    }

    bool generateUniformSample()
    {
        for(std::size_t i = 0; i < sample_sz; ++i)
        {
            const std::size_t start = randomIndex(rng_, correspondences_.size());
            bool found = false;
            for(std::size_t offset = 0;
                offset < correspondences_.size();
                ++offset)
            {
                const std::pair<std::size_t, std::size_t> & candidate =
                        correspondences_[
                                (start + offset) % correspondences_.size()];
                if(!alreadySampled(candidate, i))
                {
                    sample_[i] = candidate;
                    found = true;
                    break;
                }
            }
            if(!found)
            {
                return false;
            }
        }
        return true;
    }

    std::vector<double> residualScales() const
    {
        std::vector<double> scales(points2D_.size(), 1.0);
        for(std::size_t i = 0; i < scales.size(); ++i)
        {
            scales[i] = options_.max_error / thresholdFor(i);
        }
        return scales;
    }

    const std::vector<std::vector<Point2D> > & points2D_;
    const std::vector<std::vector<Point3D> > & points3D_;
    const std::vector<CameraPose> & cameraExtrinsics_;
    const GeneralizedPoseEstimationOptions & options_;
    poselib::RNG_t rng_;
    std::vector<std::size_t> counts_;
    std::vector<std::size_t> activeCameras_;
    std::vector<std::pair<std::size_t, std::size_t> > correspondences_;
    std::vector<std::pair<std::size_t, std::size_t> > sample_;
    std::vector<Point3D> cameraCenters_;
    std::vector<Point3D> centers_;
    std::vector<Point3D> bearings_;
    std::vector<Point3D> worldPoints_;
};

template<typename FirstPoints, typename SecondPoints>
bool validPairedInput(const FirstPoints & first, const SecondPoints & second)
{
    return first.size() == second.size();
}

void clearGeneralizedInliers(
        const std::vector<std::vector<Point2D> > & points2D,
        std::vector<std::vector<char> > * inliers)
{
    if(!inliers)
    {
        return;
    }
    inliers->resize(points2D.size());
    for(std::size_t i = 0; i < points2D.size(); ++i)
    {
        (*inliers)[i].assign(points2D[i].size(), 0);
    }
}

} // namespace

RansacStats estimateAbsolutePose(
        const std::vector<Point2D> & points2D,
        const std::vector<Point3D> & points3D,
        const PoseEstimationOptions & options,
        CameraPose * pose,
        std::vector<char> * inliers)
{
    RansacStats stats;
    if(!pose || !inliers ||
       !validPairedInput(points2D, points3D) ||
       points2D.size() < 3 ||
       !finitePoints(points2D) ||
       !finitePoints(points3D) ||
       !positiveFinite(options.max_error) ||
       !validRansacOptions(options.ransac) ||
       (options.ransac.score_initial_model && !finitePose(*pose)))
    {
        if(inliers)
        {
            inliers->assign(points2D.size(), 0);
        }
        return stats;
    }

    if(options.ransac.score_initial_model)
    {
        pose->q.normalize();
    }
    else
    {
        *pose = CameraPose();
    }

    AbsoluteEstimator estimator(points2D, points3D, options);
    stats = poselib::ransac(estimator, options.ransac, pose);

    std::size_t inlierCount = 0;
    double score = absoluteScore(
            *pose,
            points2D,
            points3D,
            options.max_error * options.max_error,
            &inlierCount,
            inliers);

    if(inlierCount < 3)
    {
        inliers->assign(points2D.size(), 0);
        stats.num_inliers = 0;
        stats.inlier_ratio = 0.0;
        stats.model_score = std::numeric_limits<double>::max();
        return stats;
    }

    if(inlierCount >= 4 && options.bundle.max_iterations > 0)
    {
        std::vector<Point2D> inlierPoints2D;
        std::vector<Point3D> inlierPoints3D;
        inlierPoints2D.reserve(inlierCount);
        inlierPoints3D.reserve(inlierCount);
        for(std::size_t i = 0; i < inliers->size(); ++i)
        {
            if((*inliers)[i])
            {
                inlierPoints2D.push_back(points2D[i]);
                inlierPoints3D.push_back(points3D[i]);
            }
        }
        CameraPose refinedPose = *pose;
        refineAbsolute(
                inlierPoints2D,
                inlierPoints3D,
                &refinedPose,
                options.bundle);
        std::vector<char> refinedInliers;
        std::size_t refinedInlierCount = 0;
        const double refinedScore = absoluteScore(
                refinedPose,
                points2D,
                points3D,
                options.max_error * options.max_error,
                &refinedInlierCount,
                &refinedInliers);
        if(refinedScore < score)
        {
            *pose = refinedPose;
            score = refinedScore;
            inlierCount = refinedInlierCount;
            inliers->swap(refinedInliers);
        }
    }

    stats.num_inliers = inlierCount;
    stats.model_score = score;
    stats.inlier_ratio = static_cast<double>(inlierCount) /
            static_cast<double>(points2D.size());
    return stats;
}

RansacStats estimateRelativePose(
        const std::vector<Point2D> & points2D1,
        const std::vector<Point2D> & points2D2,
        const PoseEstimationOptions & options,
        CameraPose * pose,
        std::vector<char> * inliers)
{
    RansacStats stats;
    if(!pose || !inliers ||
       !validPairedInput(points2D1, points2D2) ||
       points2D1.size() < 5 ||
       !finitePoints(points2D1) ||
       !finitePoints(points2D2) ||
       !positiveFinite(options.max_error) ||
       !validRansacOptions(options.ransac) ||
       (options.ransac.score_initial_model && !finitePose(*pose)))
    {
        if(inliers)
        {
            inliers->assign(points2D1.size(), 0);
        }
        return stats;
    }

    if(options.ransac.score_initial_model)
    {
        pose->q.normalize();
    }
    else
    {
        *pose = CameraPose();
    }

    RelativeEstimator estimator(points2D1, points2D2, options);
    stats = poselib::ransac(estimator, options.ransac, pose);

    std::size_t inlierCount = 0;
    double score = relativeScore(
            *pose,
            points2D1,
            points2D2,
            options.max_error * options.max_error,
            &inlierCount,
            inliers);

    if(inlierCount < 5)
    {
        inliers->assign(points2D1.size(), 0);
        stats.num_inliers = 0;
        stats.inlier_ratio = 0.0;
        stats.model_score = std::numeric_limits<double>::max();
        return stats;
    }

    if(inlierCount >= 6 && options.bundle.max_iterations > 0)
    {
        std::vector<Point2D> inlierPoints1;
        std::vector<Point2D> inlierPoints2;
        inlierPoints1.reserve(inlierCount);
        inlierPoints2.reserve(inlierCount);
        for(std::size_t i = 0; i < inliers->size(); ++i)
        {
            if((*inliers)[i])
            {
                inlierPoints1.push_back(points2D1[i]);
                inlierPoints2.push_back(points2D2[i]);
            }
        }
        CameraPose refinedPose = *pose;
        refineRelative(
                inlierPoints1,
                inlierPoints2,
                &refinedPose,
                options.bundle);
        std::vector<char> refinedInliers;
        std::size_t refinedInlierCount = 0;
        const double refinedScore = relativeScore(
                refinedPose,
                points2D1,
                points2D2,
                options.max_error * options.max_error,
                &refinedInlierCount,
                &refinedInliers);
        if(refinedScore < score)
        {
            *pose = refinedPose;
            score = refinedScore;
            inlierCount = refinedInlierCount;
            inliers->swap(refinedInliers);
        }
    }

    stats.num_inliers = inlierCount;
    stats.model_score = score;
    stats.inlier_ratio = static_cast<double>(inlierCount) /
            static_cast<double>(points2D1.size());
    return stats;
}

RansacStats estimateGeneralizedAbsolutePose(
        const std::vector<std::vector<Point2D> > & points2D,
        const std::vector<std::vector<Point3D> > & points3D,
        const std::vector<CameraPose> & cameraExtrinsics,
        const GeneralizedPoseEstimationOptions & options,
        CameraPose * pose,
        std::vector<std::vector<char> > * inliers)
{
    RansacStats stats;
    clearGeneralizedInliers(points2D, inliers);
    if(!pose || !inliers ||
       points2D.size() != points3D.size() ||
       points2D.size() != cameraExtrinsics.size() ||
       points2D.empty() ||
       !positiveFinite(options.max_error) ||
       !validRansacOptions(options.ransac) ||
       (!options.max_errors.empty() &&
        options.max_errors.size() != points2D.size()) ||
       (options.ransac.score_initial_model && !finitePose(*pose)))
    {
        return stats;
    }

    std::size_t totalPoints = 0;
    std::size_t constraintEligibleCameras = 0;
    for(std::size_t cameraIndex = 0;
        cameraIndex < points2D.size();
        ++cameraIndex)
    {
        if(points2D[cameraIndex].size() != points3D[cameraIndex].size() ||
           !finitePoints(points2D[cameraIndex]) ||
           !finitePoints(points3D[cameraIndex]) ||
           !finitePose(cameraExtrinsics[cameraIndex]) ||
           (!options.max_errors.empty() &&
            !positiveFinite(options.max_errors[cameraIndex])))
        {
            return stats;
        }
        totalPoints += points2D[cameraIndex].size();
        if(!points2D[cameraIndex].empty() &&
           points2D[cameraIndex].size() >= options.min_inliers_per_camera)
        {
            ++constraintEligibleCameras;
        }
    }
    if(totalPoints < 3 ||
       options.min_valid_cameras > constraintEligibleCameras)
    {
        return stats;
    }

    if(options.ransac.score_initial_model)
    {
        pose->q.normalize();
    }
    else
    {
        *pose = CameraPose();
    }

    GeneralizedEstimator estimator(
            points2D,
            points3D,
            cameraExtrinsics,
            options);
    stats = poselib::ransac(estimator, options.ransac, pose);
    if(stats.num_inliers < 3 ||
       stats.model_score == std::numeric_limits<double>::max())
    {
        clearGeneralizedInliers(points2D, inliers);
        stats.num_inliers = 0;
        stats.inlier_ratio = 0.0;
        stats.model_score = std::numeric_limits<double>::max();
        return stats;
    }

    std::size_t totalInliers = 0;
    std::size_t validCameras = 0;
    for(std::size_t cameraIndex = 0;
        cameraIndex < points2D.size();
        ++cameraIndex)
    {
        const CameraPose fullPose =
                GeneralizedEstimator::compose(cameraExtrinsics[cameraIndex], *pose);
        std::size_t cameraInliers = 0;
        const double threshold = estimator.thresholdFor(cameraIndex);
        absoluteScore(
                fullPose,
                points2D[cameraIndex],
                points3D[cameraIndex],
                threshold * threshold,
                &cameraInliers,
                &(*inliers)[cameraIndex]);
        totalInliers += cameraInliers;
        if(!points2D[cameraIndex].empty() &&
           cameraInliers >= options.min_inliers_per_camera)
        {
            ++validCameras;
        }
    }

    if(validCameras < options.min_valid_cameras)
    {
        for(std::size_t cameraIndex = 0;
            cameraIndex < inliers->size();
            ++cameraIndex)
        {
            std::fill(
                    (*inliers)[cameraIndex].begin(),
                    (*inliers)[cameraIndex].end(),
                    0);
        }
        totalInliers = 0;
    }

    double score = estimator.score_model(*pose, &totalInliers);
    if(totalInliers >= 4 &&
       score < std::numeric_limits<double>::max() &&
       options.bundle.max_iterations > 0)
    {
        std::vector<std::vector<Point2D> > inlierPoints2D(points2D.size());
        std::vector<std::vector<Point3D> > inlierPoints3D(points3D.size());
        for(std::size_t cameraIndex = 0;
            cameraIndex < points2D.size();
            ++cameraIndex)
        {
            inlierPoints2D[cameraIndex].reserve(
                    (*inliers)[cameraIndex].size());
            inlierPoints3D[cameraIndex].reserve(
                    (*inliers)[cameraIndex].size());
            for(std::size_t i = 0; i < (*inliers)[cameraIndex].size(); ++i)
            {
                if((*inliers)[cameraIndex][i])
                {
                    inlierPoints2D[cameraIndex].push_back(
                            points2D[cameraIndex][i]);
                    inlierPoints3D[cameraIndex].push_back(
                            points3D[cameraIndex][i]);
                }
            }
        }

        std::vector<double> residualScales(points2D.size(), 1.0);
        for(std::size_t cameraIndex = 0;
            cameraIndex < points2D.size();
            ++cameraIndex)
        {
            residualScales[cameraIndex] =
                    options.max_error / estimator.thresholdFor(cameraIndex);
        }

        CameraPose refinedPose = *pose;
        refineGeneralized(
                inlierPoints2D,
                inlierPoints3D,
                cameraExtrinsics,
                residualScales,
                &refinedPose,
                options.bundle);
        std::size_t refinedInlierCount = 0;
        const double refinedScore =
                estimator.score_model(refinedPose, &refinedInlierCount);
        if(refinedScore < score)
        {
            *pose = refinedPose;
            score = refinedScore;
            totalInliers = refinedInlierCount;
            validCameras = 0;
            for(std::size_t cameraIndex = 0;
                cameraIndex < points2D.size();
                ++cameraIndex)
            {
                const CameraPose fullPose =
                        GeneralizedEstimator::compose(
                                cameraExtrinsics[cameraIndex],
                                *pose);
                std::size_t cameraInliers = 0;
                const double threshold = estimator.thresholdFor(cameraIndex);
                absoluteScore(
                        fullPose,
                        points2D[cameraIndex],
                        points3D[cameraIndex],
                        threshold * threshold,
                        &cameraInliers,
                        &(*inliers)[cameraIndex]);
                if(!points2D[cameraIndex].empty() &&
                   cameraInliers >= options.min_inliers_per_camera)
                {
                    ++validCameras;
                }
            }
        }
    }

    if(validCameras < options.min_valid_cameras)
    {
        clearGeneralizedInliers(points2D, inliers);
        totalInliers = 0;
        score = std::numeric_limits<double>::max();
    }

    stats.num_inliers = totalInliers;
    stats.model_score = score;
    stats.inlier_ratio = totalPoints == 0?
            0.0 :
            static_cast<double>(totalInliers) /
            static_cast<double>(totalPoints);
    return stats;
}

} // namespace poselibplus
