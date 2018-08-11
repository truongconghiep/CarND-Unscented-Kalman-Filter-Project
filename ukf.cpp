#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 10;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 15;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 1.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 1.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //* State dimension
  n_x_ = 5;

  n_z_ = 3;

  L_n_z_ = 2;

  //* Augmented state dimension
  n_aug_ = 7;

  //* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* Lidar measurement noise covariance matrix
  // R_Li_ = MatrixXd(L_n_z_, L_n_z_);;
  // R_Li_ <<  std_laspx_ * std_laspx_,   0,
  //           0,                         std_laspy_ * std_laspy_;

  // ///* Radar measurement noise covariance matrix
  // R_Ra_ = MatrixXd(n_z_, n_z_);
  // R_Ra_ <<  std_radr_ * std_radr_, 0,                         0,
  //           0,                     std_radphi_ * std_radphi_, 0,
  //           0,                     0,                         std_radrd_ * std_radrd_;

  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  float delta_t;

  if(!is_initialized_)
  {
    last_timestamp = meas_package.timestamp_;
    is_initialized_ = true;

    // initialize mean
    x_ << 0.31224,
          0.58033,
          4.89281,
          0.55433,
          0.353577;

    // initialize covariance matrix
    P_ << 0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
          -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
          0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
          -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
          -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;
  }
  else
  {
    delta_t = (meas_package.timestamp_ - last_timestamp)/ 1000000.0;
    last_timestamp = meas_package.timestamp_;

    MatrixXd AugSigPts = MatrixXd(7, 15);
    //create matrix with predicted sigma points as columns
    MatrixXd PredSigPts = MatrixXd(5, 15);

    VectorXd x_pred_mean = VectorXd(n_x_);
    MatrixXd P_pred = MatrixXd(5, 5);

    VectorXd z_pred_meas_mean_ra = VectorXd(3);

    VectorXd z_pred_meas_mean_li = VectorXd(2);

    MatrixXd Zsig_pts_radar = MatrixXd(3, 15);

    MatrixXd Zsig_pts_lidar = MatrixXd(2, 15);

    MatrixXd S = MatrixXd(3,3);

    MatrixXd S_Li = MatrixXd(2,2);

    AugmentedSigmaPoints(&AugSigPts);

    SigmaPointPrediction(&AugSigPts, &PredSigPts, delta_t);

    PredictMeanAndCovariance(&PredSigPts, &x_pred_mean, &P_pred);

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // PredictRadarMeasurement(&PredSigPts, &z_pred_meas_mean_ra, &S, &Zsig_pts_radar, &R_Ra_);
      // PredictRadarMeasurement(&PredSigPts, &z_pred_meas_mean_ra, &S, &Zsig_pts_radar);
      // UpdateRadar(meas_package, &PredSigPts, &x_pred_mean, &P_pred, &Zsig_pts_radar, &z_pred_meas_mean_ra, &S);
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      // PredictLidarMeasurement(&PredSigPts, 
      //                         &z_pred_meas_mean_li, 
      //                         &S_Li, 
      //                         &Zsig_pts_lidar);
      // UpdateLidar(meas_package,
      //             &PredSigPts,               
      //             &x_pred_mean,                 
      //             &P_pred,                  
      //             &Zsig_pts_lidar,                    
      //             &z_pred_meas_mean_li,                  
      //             &S_Li);
    }
    else
    {

    }
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) 
{
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /**
   *  1. Generate sigma points
   *  2. predict sigma points
   *  3. predict mean and covariance
   */



}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar( MeasurementPackage meas_package,
                    MatrixXd *Xsig_pred,               
                    VectorXd *x_pred,                 
                    MatrixXd *P_pred,                  
                    MatrixXd *Zsig,                    
                    VectorXd *z_pred,                  
                    MatrixXd *S)  {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

    //set vector for weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  GenerateWeight(&weights);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, L_n_z_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); i++)  //2n+1 simga points
  {  

    //residual
    VectorXd z_diff = Zsig->col(i) - *z_pred;
    //angle normalization
    // while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    // while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred->col(i) - *x_pred;
    //angle normalization
    // while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    // while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S->inverse();

  // //residual
  VectorXd z_diff = meas_package.raw_measurements_ - *z_pred;

  // //angle normalization
  // // while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  // // while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // //update state mean and covariance matrix
  x_ = *x_pred + K * z_diff;
  // x_ = K * z_diff;
  P_ = *P_pred - K*(*S)*K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar( MeasurementPackage meas_package,
                    MatrixXd *Xsig_pred,               
                    VectorXd *x_pred,                 
                    MatrixXd *P_pred,                  
                    MatrixXd *Zsig,                    
                    VectorXd *z_pred,                  
                    MatrixXd *S) 
{
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //set vector for weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  GenerateWeight(&weights);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)  //2n+1 simga points
  {  

    //residual
    VectorXd z_diff = Zsig->col(i) - *z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred->col(i) - *x_pred;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S->inverse();

  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - *z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = *x_pred + K * z_diff;
  // x_ = K * z_diff;
  P_ = *P_pred - K*(*S)*K.transpose();
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) 
{
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  //write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_in, MatrixXd* Xsig_out, float delta_t)
{
  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = (*Xsig_in)(0,i);
    double p_y = (*Xsig_in)(1,i);
    double v = (*Xsig_in)(2,i);
    double yaw = (*Xsig_in)(3,i);
    double yawd = (*Xsig_in)(4,i);
    double nu_a = (*Xsig_in)(5,i);
    double nu_yawdd = (*Xsig_in)(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  //write result
  *Xsig_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance( MatrixXd* Xsig_in, 
                                    VectorXd *x_pred_mean, 
                                    MatrixXd *P_pred)
{
  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  // set weights
  //GenerateWeight(&weights);
  
  //create vector for predicted state mean
  VectorXd x = VectorXd(n_x_);
  // CalculateMean( &x, &weights, Xsig_in);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  // set weights
  GenerateWeight(&weights);

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) 
  {  //iterate over sigma points
    x = x + weights(i) * Xsig_in->col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) 
  {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_in->col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }

  //write result
  *x_pred_mean = x;
  *P_pred = P;
}

void UKF::PredictLidarMeasurement(MatrixXd* Xsig_in, 
                                  VectorXd* z_out, 
                                  MatrixXd* S_out, 
                                  MatrixXd *Zsig_pts) 
{
  // Generates weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  GenerateWeight(&weights);

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(L_n_z_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) //2n+1 simga points
  {  
    // extract values for better readibility
    double p_x = (*Xsig_in)(0,i);
    double p_y = (*Xsig_in)(1,i);
    // measurement model
    Zsig(0,i) = p_x;                        // px
    Zsig(1,i) = p_y;                        // px
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(L_n_z_);
  CalculateMean( &z_pred, &weights, &Zsig);

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(L_n_z_, L_n_z_);
  S.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(L_n_z_, L_n_z_);
  R <<    std_laspx_ * std_laspx_, 0,
          0, std_laspy_ * std_laspy_;

  S = S + R;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_pts = Zsig;
}

// void UKF::PredictRadarMeasurement(MatrixXd* Xsig_in, 
//                                   VectorXd* z_out, 
//                                   MatrixXd* S_out, 
//                                   MatrixXd *Zsig_pts,
//                                   MatrixXd *R)
void UKF::PredictRadarMeasurement(MatrixXd* Xsig_in, 
                                  VectorXd* z_out, 
                                  MatrixXd* S_out, 
                                  MatrixXd *Zsig_pts) 
{
  // Generates weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  GenerateWeight(&weights);

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);


  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = (*Xsig_in)(0,i);
    double p_y = (*Xsig_in)(1,i);
    double v  = (*Xsig_in)(2,i);
    double yaw = (*Xsig_in)(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  CalculateMean( &z_pred, &weights, &Zsig);

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  S.fill(0.0);
  for (int i = 0; i < (2 * n_aug_ + 1); i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R1 = MatrixXd(n_z_, n_z_);
  R1 <<    std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0,std_radrd_ * std_radrd_;
  S = S + R1;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_pts = Zsig;
}

void UKF::GenerateWeight(VectorXd *weights)
{
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  (*weights)(0) = weight_0;
  for (int i=1; i < (2 * n_aug_ + 1); i++) 
  {  
    double weight = 0.5 / (n_aug_ + lambda_);
    (*weights)(i) = weight;
  }
}

void UKF::CalculateMean( VectorXd *Mean_out, 
                         VectorXd *weights, 
                         MatrixXd *Zsig)
{
  Mean_out->fill(0.0);
  for (int i=0; i < (2 * n_aug_ + 1); i++) 
  {
      *Mean_out = *Mean_out + (*weights)(i) * Zsig->col(i);
  }
}