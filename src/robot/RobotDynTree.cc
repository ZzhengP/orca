#include <orca/robot/RobotDynTree.h>
#include <orca/optim/OptimisationVector.h>
#include <exception>
#include <stdexcept>

using namespace orca::robot;
using namespace orca::optim;

RobotDynTree::RobotDynTree(const std::string& modelFile)
{
    global_gravity_vector_.setZero();
    if(!modelFile.empty())
        loadModelFromFile(modelFile);
}



bool RobotDynTree::loadModelFromFile(const std::string& modelFile)
{
    bool ok = kinDynComp_.loadRobotModelFromFile(modelFile);
    if( !ok || getNrOfDegreesOfFreedom() == 0 )
    {
        throw std::runtime_error("Could not load robot model");
        return false;
    }

    urdf_url_ = modelFile;

    const iDynTree::Model & model = kinDynComp_.model();

    robotData_.resize(model);
    eigRobotState_.resize(model.getNrOfDOFs());
    idynRobotState_.resize(model.getNrOfDOFs());
    eigRobotState_.setFixedBase();

    for(unsigned int i = 0 ; i < kinDynComp_.getNrOfDegreesOfFreedom() ; i++)
    {
        double min = 0,max = 0;
        if(kinDynComp_.model().getJoint(i)->hasPosLimits())
        {
            min = kinDynComp_.model().getJoint(i)->getMinPosLimit(0);
            max = kinDynComp_.model().getJoint(i)->getMaxPosLimit(0);
            joint_pos_limits_[i] = {min,max};
        }
    }

    OptimisationVector().buildControlVariablesMapping(model.getNrOfDOFs());

    is_initialized_ = ok;
    return ok;
}

void RobotDynTree::print()
{
    std::cout << "Robot Model " << std::endl;
    for(unsigned int i=0; i < kinDynComp_.model().getNrOfJoints() ; i++)
    {
        std::cout << "      Joint " << i << " " << kinDynComp_.model().getJointName(i) << std::endl;
    }

    for(unsigned int i=0; i < kinDynComp_.model().getNrOfFrames() ; i++)
    {
        std::cout << "      Frame " << i << " " << kinDynComp_.model().getFrameName(i) << std::endl;
    }
    for(unsigned int i=0; i < kinDynComp_.model().getNrOfLinks() ; i++)
    {
        std::cout << "      Link " << i << " " << kinDynComp_.model().getLinkName(i) << std::endl;
    }
}

void RobotDynTree::setGravity(const Eigen::Vector3d& g)
{
    global_gravity_vector_ = g;
}

void RobotDynTree::setBaseFrame(const std::string& base_frame)
{
    if(!frameExists(base_frame))
        throw std::runtime_error("Frame is not part of the robot");
    base_frame_ = base_frame;
    kinDynComp_.setFloatingBase( base_frame_ );
}

void RobotDynTree::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel)
{
    setRobotState(jointPos,jointVel,global_gravity_vector_);
}
void RobotDynTree::setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    setRobotState(Eigen::Matrix4d::Identity(),jointPos,Eigen::Matrix<double,6,1>::Zero(),jointVel,gravity);
}

void RobotDynTree::setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity)
{
    if( base_frame_.empty())
        throw std::runtime_error("Base/FreeFloating frame is empty. Please use setBaseFrame before setting the robot state");

    if(global_gravity_vector_.mean() == 0)
        throw std::runtime_error("Global gravity vector is not set. Please use setGravity before setting the robot state");

    eigRobotState_.world_H_base = world_H_base;
    eigRobotState_.jointPos = jointPos;
    eigRobotState_.baseVel = baseVel;
    eigRobotState_.jointVel = jointVel;
    eigRobotState_.gravity = gravity;

    iDynTree::fromEigen(idynRobotState_.world_H_base,eigRobotState_.world_H_base);
    iDynTree::toEigen(idynRobotState_.jointPos) = eigRobotState_.jointPos;
    iDynTree::fromEigen(idynRobotState_.baseVel,eigRobotState_.baseVel);
    iDynTree::toEigen(idynRobotState_.jointVel) = eigRobotState_.jointVel;
    iDynTree::toEigen(idynRobotState_.gravity)  = eigRobotState_.gravity;

    kinDynComp_.setRobotState(idynRobotState_.world_H_base
                            ,idynRobotState_.jointPos
                            ,idynRobotState_.baseVel
                            ,idynRobotState_.jointVel
                            ,idynRobotState_.gravity);
}

const std::string& RobotDynTree::getBaseFrame() const
{
    return base_frame_;
}

const std::string& RobotDynTree::getFileURL() const
{
    return urdf_url_;
}

int RobotDynTree::getNrOfDegreesOfFreedom() const
{
    return kinDynComp_.getNrOfDegreesOfFreedom();
}

const std::map<unsigned int, std::pair<double,double> >& RobotDynTree::getJointPositionLimits()
{
    return joint_pos_limits_;
}

bool RobotDynTree::frameExists(const std::string& frame_name)
{
    if(kinDynComp_.getFrameIndex(frame_name) < 0)
    {
        return false;
    }
    return true;
}

const Eigen::Matrix4d& RobotDynTree::getRelativeTransform(const std::string& refFrameName, const std::string& frameName)
{
    return iDynTree::toEigen(kinDynComp_.getRelativeTransform(refFrameName,frameName).asHomogeneousTransform());
}

const Eigen::Matrix<double,6,1>& RobotDynTree::getFrameVel(const std::string& frameName)
{
    return iDynTree::toEigen(kinDynComp_.getFrameVel(frameName));
}

const Eigen::Matrix<double,6,1>& RobotDynTree::getFrameBiasAcc(const std::string& frameName)
{
    return iDynTree::toEigen(kinDynComp_.getFrameBiasAcc(frameName));
}

const Eigen::MatrixXd& RobotDynTree::getFreeFloatingMassMatrix()
{
    kinDynComp_.getFreeFloatingMassMatrix(robotData_.idynMassMatrix);
    return iDynTree::toEigen(robotData_.idynMassMatrix);
}

const Eigen::MatrixXd& RobotDynTree::getFrameFreeFloatingJacobian(const std::string& frameName)
{
    kinDynComp_.getFrameFreeFloatingJacobian(frameName,robotData_.idynJacobianFb);
    return iDynTree::toEigen(robotData_.idynJacobianFb);
}

const Eigen::MatrixXd& RobotDynTree::getRelativeJacobian(const std::string& refFrameName, const std::string& frameName)
{
    kinDynComp_.getRelativeJacobian(kinDynComp_.getFrameIndex(refFrameName)
                                ,kinDynComp_.getFrameIndex(frameName)
                                ,robotData_.idynJacobianFb);
    return iDynTree::toEigen(robotData_.idynJacobianFb);
}

const Eigen::VectorXd& RobotDynTree::getJointPos()
{
    return eigRobotState_.jointPos;
}

const Eigen::VectorXd& RobotDynTree::getJointVel()
{
    return eigRobotState_.jointVel;
}

const Eigen::VectorXd& RobotDynTree::generalizedBiasForces()
{
    kinDynComp_.generalizedBiasForces(robotData_.generalizedBiasForces);
    return iDynTree::toEigen(robotData_.generalizedBiasForces);
}