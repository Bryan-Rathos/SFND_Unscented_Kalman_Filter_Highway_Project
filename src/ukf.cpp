#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
  Initializes Unscented Kalman filter
*/
UKF::UKF() {
  use_laser_ = true;    // if this is false, laser measurements will be ignored (except during init)
  use_radar_ = true;    // if this is false, radar measurements will be ignored (except during init)
  x_ = VectorXd(5);     // initial state vector
  P_ = MatrixXd(5, 5);  // initial covariance matrix
  std_a_ = 70.0;          // Process noise standard deviation longitudinal acceleration in m/s^2
  std_yawdd_ = 15.0;      // Process noise standard deviation yaw acceleration in rad/s^2
  // 1.2, 0.6

  /*
    * DO NOT MODIFY measurement noise values below.
    * These are provided by the sensor manufacturer.
  */

  std_laspx_ = 0.15;    // Laser measurement noise standard deviation position1 in m
  std_laspy_ = 0.15;    // Laser measurement noise standard deviation position2 in m
  std_radr_ = 0.3;      // Radar measurement noise standard deviation radius in m
  std_radphi_ = 0.03;   // Radar measurement noise standard deviation angle in rad
  std_radrd_ = 0.3;     // Radar measurement noise standard deviation radius change in m/s
  
  /*
    End DO NOT MODIFY section for measurement noise values 
  */
  
  /*
    * TODO: Complete the initialization. See ukf.h for other member properties.
    * Hint: one or more values initialized above might be wildly off...
  */

  is_initialized_ = false;                      // Set to false initailly. Set to true in the first call to ProcessMeasurement()
  time_us_ = 0.0;                               // Initialize timestamp, in microseconds
  n_x_ = 5;                                     // State dimension
  n_aug_ = 7;                                   // Augmented state dimension
  x_ = VectorXd::Zero(n_x_);                    // initial state vector
  P_ = MatrixXd::Identity(n_x_, n_x_);          // initial covariance matrix
  lambda_ = 3 - n_aug_;                           // sigma points spreading parameter
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);  // predicted sigma point parameter 
  weights_ = VectorXd(2 * n_aug_ + 1);          // weights vector
  NIS_radar_ = 0.0;                             // the current NIS for radar
  NIS_laser_ = 0.0;                             // the current NIS for laser 
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*
    TODO: Complete this function! Make sure you switch between lidar and radar
    measurements.
  */

  /************************
  * Initialization 
  ************************/
  if(!is_initialized_)
  {
    // In initializing the State covariance matrix I first started with an identity
    // However, since it wasn't within RMSE error bounds I started fine tuning with values
    // with variances that I exepected the respective state variables to have.
    P_ << 0.2, 0, 0, 0, 0,
          0, 0.2, 0, 0, 0,
          0, 0, 4, 0, 0,
          0, 0, 0, 0.03, 0,
          0, 0, 0, 0, 0.3;


    if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho = meas_package.raw_measurements_(0);
      double phi  = meas_package.raw_measurements_(1);
      double rho_dot = meas_package.raw_measurements_(2);

      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);
    } 
    else
    {
      std::cout << "No sensor measurement" << std::endl;
    }

    time_us_ = meas_package.timestamp_;

    is_initialized_ = true;
       
    return;
  }

  /**********************
  * Prediction
  **********************/
  long double dt = (meas_package.timestamp_ - time_us_) / static_cast<double>(1e6);
  time_us_ = meas_package.timestamp_;
  
  Prediction(dt);

  /**********************
  * Update
  **********************/
  if(meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }
  else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  else
  {
    std::cout << "No sensor measurement" << std::endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  /***************************************
  * Genrate Sigma points and augmentation
  ****************************************/
  //create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  /****************************
  * Sigma points prediction
  ****************************/
  // predict sigma points
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v   = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    //avoid division by zero
    if(fabs(yawd) > 0.001)
    {
      px_p = p_x + v/yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd * delta_t));    
    }
    else
    {
      px_p = p_x + v * cos(yaw) * delta_t;
      py_p = p_y + v * sin(yaw) * delta_t;
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p += nu_a * delta_t;
    yaw_p += 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p += nu_yawdd * delta_t;

    // save calculated results to sigma point prediction matrix
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /***************************************************************
  * Predict Mean and Covariance from the predeicted sigma points
  ***************************************************************/
  //set weights
  double weight_0 = lambda_/(lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i < 2 * n_aug_ + 1; i++)
  { 
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  //predict state mean
  x_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);   //update state 
  }

  // predict state covariance matrix
  P_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();   //update state 
  }
}

/*
  Updates the state and the state covariance matrix using a laser measurement.
  @param {MeasurementPackage} meas_package
*/
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // set LIDAR measurement dimaension
  int n_z = 2;

  // Retrieve measurement and stor in vector z
  VectorXd z = meas_package.raw_measurements_;

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  //Mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);


  // Build measurement model
  Zsig = Xsig_pred_.block(0, 0, n_z, 2 * n_aug_ + 1);
  
  // Mean predicted measurement
  for(int i = 0; i < 2 * n_aug_+1; i++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // Predicted measurement covariance matrix S
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << pow(std_laspx_,2),0,
       0,pow(std_laspy_,2);
  S += R;

  /***** UKF LIDAR Update ******/

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // Calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // Residual (predicted sigma pts. in measurement space and predicted measurement mean pts)
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual (Actual measurement and predicted measurement)
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  // std::cout << NIS_laser_ << ", ";

  std::ofstream nis_laser_file;
  nis_laser_file.open("../NIS_values/nis_laser.csv", std::ios::app);
  if(!nis_laser_file)					// if the open failed
  {
    std::cerr << "Attempt to create lasert file failed\n";
    exit(1);
  }
  nis_laser_file << NIS_laser_<< ",";
  nis_laser_file.close();
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // Set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // Retrieve measurement and store in vector z
  VectorXd z = meas_package.raw_measurements_;

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z);
  
  // Measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);

  // Build measurement model
  for(int i=0; i < 2 * n_aug_ + 1; i++)
  {
    // Extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
  
    double v_x = v * cos(yaw);
    double v_y = v * sin(yaw);
    
    // Transform sigma points into measurement space
    // Measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);             //rho
    Zsig(1,i) = atan2(p_y,p_x);                      //phi
    Zsig(2,i) = (p_x*v_x + p_y*v_y)/Zsig(0,i);       //rho_dot
  }

  // Calculate mean predicted measurement z_pred
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // Predicted measurement covariance matrix S
  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    S +=  weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << pow(std_radr_,2),0,0,
       0,pow(std_radphi_,2),0,
       0,0,pow(std_radrd_,2);

  S += R;

  /***** UKF Radar Update ******/

  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // Calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // Residual (predicted sigma pts. in measurement space and predicted measurement mean pts)
    VectorXd z_diff = Zsig.col(i) - z_pred;
   
    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    // angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // Difference between actual measurement and predicted measurement
  VectorXd z_diff = z - z_pred;

  while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS for RADAR
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  // std::cout << NIS_radar_ << ", ";

  std::ofstream nis_radar_file;
  nis_radar_file.open("../NIS_values/nis_radar.csv", std::ios::app);
  if(!nis_radar_file)					// if the open failed
  {
    std::cerr << "Attempt to create radar file failed\n";
    exit(1);
  }
  nis_radar_file << NIS_radar_<< ",";
  nis_radar_file.close();
}