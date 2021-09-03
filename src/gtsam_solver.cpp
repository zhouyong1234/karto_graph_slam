#include <limits>
#include <open_karto/Karto.h>
#include <ros/console.h>
#include "karto_graph_slam/gtsam_solver/gtsam_solver.h"


GTSAMSolver::GTSAMSolver()
{
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1e-6, 1e-6, 1e-8));

    graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose2> >(0, gtsam::Pose2(0,0,0), priorNoise);
}

GTSAMSolver::~GTSAMSolver()
{

}

void GTSAMSolver::Clear()
{
    corrections_.clear();
}

const karto::ScanSolver::IdPoseVector& GTSAMSolver::GetCorrections() const
{
    return corrections_;
}

void GTSAMSolver::Compute()
{
    corrections_.clear();
    graphNodes_.clear();

    gtsam::LevenbergMarquardtParams parameters;
    parameters.relativeErrorTol = 1e-5;
    parameters.maxIterations = 500;

    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initialGuess_, parameters);

    gtsam::Values result = optimizer.optimize();

    gtsam::Values::ConstFiltered<gtsam::Pose2> viewPose2 = result.filter<gtsam::Pose2>();

    for(const gtsam::Values::ConstFiltered<gtsam::Pose2>::KeyValuePair& key_value : viewPose2)
    {
        karto::Pose2 pose(key_value.value.x(), key_value.value.y(), key_value.value.theta());

        corrections_.push_back(std::make_pair(key_value.key, pose));

        graphNodes_.push_back(Eigen::Vector2d(key_value.value.x(), key_value.value.y()));
    }
}


void GTSAMSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex)
{
    karto::Pose2 odom = pVertex->GetObject()->GetCorrectedPose();

    initialGuess_.insert(pVertex->GetObject()->GetUniqueId(), gtsam::Pose2(odom.GetX(), odom.GetY(), odom.GetHeading()));

    graphNodes_.push_back(Eigen::Vector2d(odom.GetX(), odom.GetY()));

    ROS_DEBUG("[GTSAM] Adding node %d.", pVertex->GetObject()->GetUniqueId());
}

void GTSAMSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge)
{
    int sourceID = pEdge->GetSource()->GetObject()->GetUniqueId();
    int targetID = pEdge->GetTarget()->GetObject()->GetUniqueId();

    karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

    karto::Pose2 diff = pLinkInfo->GetPoseDifference();

    karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance();

    Eigen::Matrix<double, 3, 3> cov;

    cov(0,0) = precisionMatrix(0,0);
  
    cov(0,1) = cov(1,0) = precisionMatrix(0,1);
    
    cov(0,2) = cov(2,0) = precisionMatrix(0,2);
    
    cov(1,1) = precisionMatrix(1,1);
    
    cov(1,2) = cov(2,1) = precisionMatrix(1,2);
    
    cov(2,2) = precisionMatrix(2,2);

    gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Diagonal::Covariance(cov);

    // Add odometry factors
    // Create odometry factors between consecutive poses
    graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(sourceID, targetID, gtsam::Pose2(diff.GetX(), diff.GetY(), diff.GetHeading()), model);

    // Add the constraint to the optimizer
    ROS_DEBUG("[GTSAM] Adding Edge from node %d to node %d. ", sourceID, targetID);
}

void GTSAMSolver::getGraph(std::vector<Eigen::Vector2d>& nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> >& edges)
{
    nodes = graphNodes_;
}