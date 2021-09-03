#ifndef GTSAM_SOLVER_H
#define GTSAM_SOLVER_H

#include <open_karto/Mapper.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

typedef std::vector<karto::Matrix3> CovarianceVector;

class GTSAMSolver : public karto::ScanSolver
{
public:
    GTSAMSolver(/* args */);
    virtual ~GTSAMSolver();

public:
    virtual void Clear();

    virtual void Compute();

    virtual const karto::ScanSolver::IdPoseVector& GetCorrections() const;

    virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan>* pVertex);

    virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* pEdge);

    void getGraph(std::vector<Eigen::Vector2d>& nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d> >& edges);

private:
    karto::ScanSolver::IdPoseVector corrections_;

    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initialGuess_;
    std::vector<Eigen::Vector2d> graphNodes_;
};


#endif