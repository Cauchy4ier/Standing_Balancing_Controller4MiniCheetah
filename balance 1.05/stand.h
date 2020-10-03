#ifndef JPOS_CONTROLLER
#define JPOS_CONTROLLER

#include "RobotController.h"
#include "stand_parameters.h"
#include "ControlParameters/RobotParameters.h"
#include "ControlFSMData.h"
#include "TransitionData.h"
#include "Controllers/GaitScheduler.h"
#include "BalanceController.hpp"

class Leg_InvDyn_Controller:public RobotController{
  public:
    Leg_InvDyn_Controller():RobotController(){
    }
    virtual ~Leg_InvDyn_Controller(){}

    virtual void initializeController(){};
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
    }

void cartesianImpedanceControl(int leg, Vec3<float> pDes, Vec3<float> vDes,
                                 Vec3<double> kp_cartesian,
                                 Vec3<double> kd_cartesian);

void runBalanceController();

 //**Defined class and parameters
  RobotControlParameters* controlParameters;
  Mat34<float> jointFeedForwardTorques;  // feed forward joint torques
  Mat34<float> jointPositions;           // joint angle positions
  Mat34<float> jointVelocities;          // joint angular velocities
  Mat34<float> footFeedForwardForces;    // feedforward forces at the feet
  Mat34<float> footPositions;            // cartesian foot positions
  Mat34<float> footVelocities;           // cartesian foot velocities
  Mat34<float> footstepLocations;           // next step location
  BalanceController balanceController;
  protected:
    Leg_InvDyn_UserParameters userParameters;
  private:
    int iter=0;
    Mat3<float>kpMat;
    Mat3<float>kdMat;
};

#endif
