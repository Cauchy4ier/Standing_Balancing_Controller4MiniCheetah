#include "stand.h"
#include <iostream>

void Leg_InvDyn_Controller::runController(){

  iter ++;
   
  FBModelState<float> state;
  
  state.bodyOrientation = _stateEstimate->orientation;
  state.bodyPosition    = _stateEstimate->position;
  state.bodyVelocity.head(3) = _stateEstimate->omegaBody;
  state.bodyVelocity.tail(3) = _stateEstimate->vBody;
  state.q.setZero(12);
  state.qd.setZero(12);
  state.q.setZero(12);
  state.qd.setZero(12);
  for (int i = 0; i < 4; ++i) {
    state.q(3*i+0) = _legController->datas[i].q[0];
    state.q(3*i+1) = _legController->datas[i].q[1];
    state.q(3*i+2) = _legController->datas[i].q[2];
    state.qd(3*i+0)= _legController->datas[i].qd[0];
    state.qd(3*i+1)= _legController->datas[i].qd[1];
    state.qd(3*i+2)= _legController->datas[i].qd[2];
  }
  float hMax = 0.25; 
  float progress = 2 * iter *_controlParameters->controller_dt;
  _model->setState(state);
  if (progress > 1.)
     { progress = 1.; }

  for(int i = 0; i < 4; i++) {
    _legController->commands[i].kpCartesian = Vec3<float>(500, 500, 500).asDiagonal();
     _legController->commands[i].kdCartesian = Vec3<float>(8, 8, 8).asDiagonal();
    _legController->commands[i].pDes = _legController->datas[i].p;
    _legController->commands[i].pDes[2] = progress*(-hMax) + (1. - progress) * _legController->datas[i].p[2]; 
    }

// **after iter>500,the robot stands up and keep it balance,otherwise, it would fall down.
  if(iter>500){

    runBalanceController();

 for (int leg = 0; leg < 4; leg++) {
   footstepLocations.col(leg) << 0.0, 0.0,_quadruped->_maxLegLength / 2;
   cartesianImpedanceControl(leg, footstepLocations.col(leg), Vec3<float>::Zero(),
        controlParameters->stand_kp_cartesian,
       controlParameters->stand_kd_cartesian); 
    _legController->commands[leg].forceFeedForward =footFeedForwardForces.col(leg);
  }

 }
}

void  Leg_InvDyn_Controller::cartesianImpedanceControl(int leg, Vec3<float> pDes,
                                             Vec3<float> vDes,
                                             Vec3<double> kp_cartesian,
                                             Vec3<double> kd_cartesian) {
  _legController->commands[leg].pDes = pDes;
  // Create the cartesian P gain matrix
  kpMat << kp_cartesian[0], 0, 0, 0,
      controlParameters->stand_kp_cartesian[1], 0, 0, 0,
      controlParameters->stand_kp_cartesian[2];
  _legController->commands[leg].kpCartesian = kpMat;

  _legController->commands[leg].vDes = vDes;
  // Create the cartesian D gain matrix
  kdMat << kd_cartesian[0], 0, 0, 0, kd_cartesian[1], 0, 0, 0, kd_cartesian[2];
  _legController->commands[leg].kdCartesian = kdMat;
}


 void Leg_InvDyn_Controller::runBalanceController() {
  double minForce = 25;
  double maxForce = 500;
  double contactStateScheduled[4];  // = {1, 1, 1, 1};
  for (int i = 0; i < 4; i++) {
    contactStateScheduled[i] =1;
  }

  double minForces[4];  // = {minForce, minForce, minForce, minForce};
  double maxForces[4];  // = {maxForce, maxForce, maxForce, maxForce};
  for (int leg = 0; leg < 4; leg++) {
    minForces[leg] = contactStateScheduled[leg] * minForce;
    maxForces[leg] = contactStateScheduled[leg] * maxForce;
  }

  double COM_weights_stance[3] = {1, 1, 10};
  double Base_weights_stance[3] = {20, 10, 10};
  double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy[3],
      omegaDes[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];

  for (int i = 0; i < 4; i++) {
    se_xfb[i] = (double)_stateEstimator->getResult().orientation(i);
  }
  // se_xfb[3] = 1.0;
  for (int i = 0; i < 3; i++) {
    rpy[i] = 0;  
    p_des[i] = (double)_stateEstimator->getResult().position(i);
    p_act[i] = (double)_stateEstimator->getResult().position(i);
    omegaDes[i] =0;  
    v_act[i] = (double)_stateEstimator->getResult().vBody(i);
    v_des[i] = (double)_stateEstimator->getResult().vBody(i);

    se_xfb[4 + i] = (double)_stateEstimator->getResult().position(i);
    se_xfb[7 + i] = (double)_stateEstimator->getResult().omegaBody(i);
    se_xfb[10 + i] = (double)_stateEstimator->getResult().vBody(i);

    // Set the translational and orientation gains
    kpCOM[i] = (double)controlParameters->kpCOM(i);
    kdCOM[i] = (double)controlParameters->kdCOM(i);
    kpBase[i] = (double)controlParameters->kpBase(i);
    kdBase[i] = (double)controlParameters->kdBase(i);
  }

  Vec3<float> pFeetVec;
  Vec3<float> pFeetVecCOM;
  // Get the foot locations relative to COM
  for (int leg = 0; leg < 4; leg++) {
    computeLegJacobianAndPosition(**&_quadruped,_legController->datas[leg].q,(Mat3<float>*)nullptr, &pFeetVec, 1);

    pFeetVecCOM = _stateEstimator->getResult().rBody.transpose() *
                 (_quadruped->getHipLocation(leg)+_legController->datas[leg].p);


    pFeet[leg * 3] = pFeetVecCOM[0];
    pFeet[leg * 3 + 1] = pFeetVecCOM[1];
    pFeet[leg * 3 + 2] = pFeetVecCOM[2];
  }
  
  balanceController.set_alpha_control(0.01);
  balanceController.set_friction(0.5);
  balanceController.set_mass(46.0);
  balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
  balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
  balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
  balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
  balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, 0.0);

  double fOpt[12];
  balanceController.solveQP_nonThreaded(fOpt);

  // Publish the results over LCM
  balanceController.publish_data_lcm();

  // Copy the results to the feed forward forces
  for (int leg = 0; leg < 4; leg++) {
    footFeedForwardForces.col(leg) <<fOpt[leg * 3],fOpt[leg * 3 + 1],fOpt[leg * 3 + 2];
  }

}












