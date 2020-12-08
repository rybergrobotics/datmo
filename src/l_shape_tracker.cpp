/*
 * Copyright (c) 2020, Robobrain.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Konstantinos Konstantinidis */

#include "l_shape_tracker.hpp"

LshapeTracker::LshapeTracker(){}//Creates a blank estimator

LshapeTracker::LshapeTracker(const double x_corner, const double y_corner, const double dt){

  // Initialization of Dynamic Kalman Filter
  int n = 4; // Number of states
  int m = 2; // Number of measurements
  MatrixXd A(n, n); // System dynamics matrix
  MatrixXd C(m, n); // Output matrix
  MatrixXd Q(n, n); // Process noise covariance
  MatrixXd R(m, m); // Measurement noise covariance
  MatrixXd P(n, n); // Estimate error covariance
      
  A << 1, 0,dt, 0, 
       0, 1, 0,dt, 
       0, 0, 1, 0, 
       0, 0, 0, 1;
       
  C << 1, 0, 0, 0,
       0, 1, 0, 0;

  Q << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0,10, 0,
       0, 0, 0,10;

  R.setIdentity();
  R *=10;
  R *= 0.1;
  P.setIdentity() * 0.1;

  KalmanFilter dynamic_kalman_filter(dt, A, C, Q, R, P); 
  this->dynamic_kf = dynamic_kalman_filter;

  VectorXd x0_dynamic(n);
  x0_dynamic << x_corner, y_corner, 0, 0;
  dynamic_kf.init(0,x0_dynamic);

  x_old = x_corner;
  y_old = y_corner;

}

void LshapeTracker::update(const double x_corner, const double y_corner, const double dt) {

  // Update Dynamic Kalman Filter
  Vector2d y;
  y << x_corner, y_corner;
  dynamic_kf.update(y, dt);

  x_old = x_corner;
  y_old = y_corner;

}


  void LshapeTracker::BoxModel(double& x, double& y,double& vx, double& vy){
  x = dynamic_kf.state()(0);
  y = dynamic_kf.state()(1);

  //Equations 31 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  //TODO test the complete equation also
  vx = dynamic_kf.state()(2);
  vy = dynamic_kf.state()(3);

}
