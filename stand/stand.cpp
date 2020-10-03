#include "stand.h"
#include <iostream>
#include<time.h>
#include<unistd.h>
void Leg_InvDyn_Controller::runController(){

  static int iter = 0;
  iter ++;
  //Vec3<float>_ini_foot_pos;
  //for(size_t leg(0); leg<4; ++leg){_ini_foot_pos[leg] = _legController->datas[leg].p;}
   
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
  if (progress > 1.){ progress = 1.; }

  for(int i = 0; i < 4; i++) {
    _legController->commands[i].kpCartesian = Vec3<float>(500, 500, 500).asDiagonal();
     _legController->commands[i].kdCartesian = Vec3<float>(8, 8, 8).asDiagonal();
    _legController->commands[i].pDes = _legController->datas[i].p;
    _legController->commands[i].pDes[2] = progress*(-hMax) + (1. - progress) * _legController->datas[i].p[2]; 
    }
  
 
  }
