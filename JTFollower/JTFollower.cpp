/**
 * @file JT_Follower.cpp
 * @brief Read the .h heading for details :)
 * @author A.H.Q.
 * @date March 07th, 2012
 */

#include <boost/algorithm/string.hpp>
#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "JTFollower.h"
#include <Tools/Constants.h>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>


/**
 * @function JTFollower
 * @brief Constructor
 */
JTFollower::JTFollower() {
  mCopyWorld = false;
  mWorld = NULL;
}

/**
 * @function JTFollower
 * @brief Constructor
 */
JTFollower::JTFollower( robotics::World &_world,
			bool _copyWorld,
			double _configStep ) {

  mCopyWorld = _copyWorld;

  if( mCopyWorld ) {
    printf( "Not implemented yet. Sorry -- achq \n" );
  } else {
    mWorld = &_world;
  }

  mConfigStep = _configStep;
}

/**
 * @function ~JTFollower
 * @brief Destructor
 */
JTFollower::~JTFollower() {

  if( mCopyWorld ) {
    delete mWorld;
  }
}

/**
 * @function init
 */
void JTFollower::init( int _robotId,
		       const Eigen::VectorXi &_links,
		       std::string _EEName,
		       int _EEId,
		       double _res ) {

  mRobotId = _robotId;
  mLinks = _links;

  mMaxIter = 5000;
  mWorkspaceThresh = _res; // An error of half the resolution
  mEENode = (dynamics::BodyNodeDynamics*) mWorld->getRobot(mRobotId)->getNode( _EEName.c_str() );
  mEEId = _EEId;
}

/**
 * @function planPath
 * @brief Main function
 */
std::vector< Eigen::VectorXd > JTFollower::PlanPath( const Eigen::VectorXd &_start,
						     const std::vector<Eigen::VectorXd> &_workspacePath ) {

  //-- Follow the path
  std::vector< Eigen::VectorXd > configPath;
  Eigen::VectorXd q;

  size_t numPoints = _workspacePath.size();

  //-- Initialize
  q = _start;

  for( size_t i = 1; i < numPoints; ++i ) { // start from 1 since 0 is the current start position
    if( GoToXYZ( q, GetXYZ(_workspacePath[i]), configPath ) == false ) {
      printf(" --(x) An error here, stop following path \n"); break;
    }
  }

  printf("End of Plan Path \n");
  return configPath;

}

/**
 * @function GetJac
 */
Eigen::MatrixXd JTFollower::GetJac( Eigen::VectorXd _q ) {

  Eigen::MatrixXd Jaclin = mEENode->getJacobianLinear();
  Eigen::MatrixXd Jacang = mEENode->getJacobianAngular();

  Eigen::MatrixXd Jac(Jaclin.rows()*2,Jaclin.cols()); Jac << Jaclin, Jacang;
  Jac = Jac.topRightCorner( 6, mEENode->getNumDependentDofs() -6 );

  return Jac;
}

/**
 * @function GetPseudoInvJac
 */
Eigen::MatrixXd JTFollower::GetPseudoInvJac( Eigen::VectorXd _q ) {

  Eigen::MatrixXd Jaclin = mEENode->getJacobianLinear();
  Eigen::MatrixXd Jacang = mEENode->getJacobianAngular();

  Eigen::MatrixXd Jac(Jaclin.rows()*2,Jaclin.cols()); Jac << Jaclin, Jacang;
  Jac = Jac.topRightCorner( 6, mEENode->getNumDependentDofs() -6 );

  Eigen::MatrixXd JacT = Jac.transpose();
  Eigen::MatrixXd Jt;
  Eigen::MatrixXd JJt = (Jac*JacT);
  Eigen::FullPivLU<Eigen::MatrixXd> lu(JJt);
  Jt = JacT*( lu.inverse() );


  return Jt;
}


/**
 * @function GoToXYZ
 */
bool JTFollower::GoToXYZR( Eigen::VectorXd &_q,
			   Eigen::VectorXd _targetXYZ,
			   Eigen::VectorXd _targetRPY,
			   std::vector<Eigen::VectorXd> &_workspacePath ) {

  Eigen::VectorXd dXYZ,dRPY,dMov(6);
  Eigen::VectorXd dConfig;

  int iter;
  mWorld->getRobot(mRobotId)->setDofs( _q, mLinks );
  mWorld->getRobot(mRobotId)->update();

  //-- Initialize
  dXYZ = ( _targetXYZ - GetXYZ(_q) ); // GetXYZ also updates the config to _q, so Jac use an updated value
  dRPY = ( _targetRPY - GetRPY(_q) ); // GetRPY also updates the config to _q, so Jac use an updated value
  dMov << dXYZ,dRPY;
  iter = 0;
  while( dMov.norm() > mWorkspaceThresh && iter < mMaxIter ) {
    mWorld->getRobot(mRobotId)->setDofs( _q, mLinks );
    mWorld->getRobot(mRobotId)->update();

    Eigen::MatrixXd Jt = GetPseudoInvJac(_q);
    Eigen::MatrixXd Jac = GetJac(_q);


    // We have the jacobian, now to implement joint limit avoidance.
    Eigen::VectorXd _h(_q.size());
    Eigen::VectorXd theta_bar(_q.size());
    Eigen::MatrixXd W(_q.size(),_q.size()); W << Eigen::MatrixXd::Identity(_q.size(),_q.size());

    // Get joint limits for each link
    for( int idx = 6; idx < mEENode->getNumDependentDofs(); idx++){
      kinematics::Dof *eval = mEENode->getSkel()->getDof(mEENode->getDependentDof(idx));
      double min = eval->getMin();
      double max = eval->getMax();
      theta_bar(idx-6) = max + min;
    }
    // Calculate gradient vector _h, used to define slope away from joints
    _h = Eigen::VectorXd::Zero(_q.size());

    for( int idx = 0; idx < _q.size(); idx++){
      double epsilon = .001;
      Eigen::VectorXd _qh(_q);
      _qh(idx) += epsilon;
      _h(idx) = ((
		  ((_qh - theta_bar).transpose() * W * (_qh - theta_bar)) -
		  ((_q - theta_bar).transpose() * W * (_q - theta_bar))
		  )(0)
		 );
    }

    // Avoid joint limits
    Eigen::MatrixXd avoid_limit = ( Eigen::MatrixXd::Ones(Jt.rows(),Jac.cols()) - (Jt*Jac));
    // (resize avoid_limit to include all the non-adjusted DoFs)
    avoid_limit.conservativeResizeLike(Eigen::MatrixXd::Zero(_q.size(),_q.size()));
    avoid_limit = avoid_limit * _h;

    if (Jt.cols() < _q.size()){
      dConfig = (Eigen::VectorXd(_q.size()) << Jt*dMov,
		 Eigen::VectorXd::Zero(_q.size() - Jt.rows())).finished(); // Top row is base, bottom is end effector.
    }else {
      dConfig = (Eigen::VectorXd(_q.size()) << Jt*dMov).finished(); // Top row is base, bottom is end effector.
    }
    // Magic -- add the additional avoidance information to dconfig. We
    // will now try to stay near the zero points of joints.
    dConfig += avoid_limit;

    if( dConfig.norm() > mConfigStep ) {
      double n = dConfig.norm();
      dConfig = dConfig *(mConfigStep/n);
    }

    _q = _q + dConfig;
    _workspacePath.push_back( _q );

    dXYZ = (_targetXYZ - GetXYZ(_q) );
    dRPY = (_targetRPY - GetRPY(_q) );

    dMov = (Eigen::VectorXd(6) << dXYZ, dRPY).finished();
    iter++;
  }
  if( iter >= mMaxIter ) { return false; }
  else { return true; }
}

/**
 * @function GoToXYZ
 */
bool JTFollower::GoToXYZ(  Eigen::VectorXd &_q,
			   Eigen::VectorXd _targetXYZ,
			   std::vector<Eigen::VectorXd> &_workspacePath ) {
  return GoToXYZR( _q, _targetXYZ, GetRPY(_q), _workspacePath);

}

/**
 * @function GetRPY
 */
Eigen::VectorXd JTFollower::GetRPY( Eigen::VectorXd _q ) {
  // Get current RPY position
  mWorld->getRobot(mRobotId)->setDofs( _q, mLinks );
  mWorld->getRobot(mRobotId)->update();

  Eigen::MatrixXd qTransform = mEENode->getWorldTransform();

  double r,p,y;
  r = atan2( qTransform(2,1), qTransform(2,2) );
  p = -asin( qTransform(2,0) );
  y = atan2( qTransform(1,0), qTransform(0,0) );

  Eigen::VectorXd qRPY(3); qRPY << r,p,y;
  return qRPY;
}


/**
 * @function GetXYZ
 */
Eigen::VectorXd JTFollower::GetXYZ( Eigen::VectorXd _q ) {
  // Get current XYZ position
  mWorld->getRobot(mRobotId)->setDofs( _q, mLinks );
  mWorld->getRobot(mRobotId)->update();

  Eigen::MatrixXd qTransform = mEENode->getWorldTransform();
  Eigen::VectorXd qXYZ(3); qXYZ << qTransform(0,3), qTransform(1,3), qTransform(2,3);

  return qXYZ;
}
