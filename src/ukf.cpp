#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // std::cout << "In UKF constructor() ..." << std::endl;

  // Set to true after first call
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  // Not quite sure what to initialize this to ...
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 0.098, 0,
        0, 0, 0, 0, 0.0123;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 30;
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 30;
  std_yawdd_ = 0.9;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // Dimension of the state vector
  n_x_ = 5;

  // Dimension of augmented state vector
  n_aug_ = n_x_ + 2;   // Added longitudial and yaw acceleration noise

  // Initialize sigma point spreading design parameter
  lambda_ = 3 - n_aug_;

  // Initialize Covariance Matrix
  

  // Instantiate and Initialize weights for the sigma points
  weights_ = VectorXd(2*n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2*n_aug_ + 1; i++) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

}

UKF::~UKF()
{
    // std::cout << "In UKF destructor() ..." << std::endl;
}


void UKF::InitializeFilter(MeasurementPackage meas_package)
{
  // std::cout << "In InitializeFilter(): "; 

  double p_x = 0.0, p_y = 0.0, v = 0.0;
  double yaw = 0.0, yaw_d = 0.0;

  // Initialize state variables with first measurement data
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) 
  {
    // std::cout << "LASER data = \n" << meas_package.raw_measurements_ << std::endl;
    p_x = meas_package.raw_measurements_[0];
    p_y = meas_package.raw_measurements_[1];
  } 
  else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR) 
  {
    // std::cout << "RADAR data = \n" << meas_package.raw_measurements_ << std::endl;
    p_x = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
    p_y = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
    double v_x = meas_package.raw_measurements_[2] * cos(meas_package.raw_measurements_[1]);
    double v_y = meas_package.raw_measurements_[2] * sin(meas_package.raw_measurements_[1]);
    v = sqrt(v_x*v_x + v_y*v_y);
  } 
  else 
  {
    std::cout << " *** ERROR: Invalid Sensor Type '" << meas_package.sensor_type_ << "'..." << std::endl;
  }

  // Initialize state vector
  x_ << p_x, p_y, v, yaw, yaw_d;

  // Initialize time
  time_us_ = meas_package.timestamp_;

  // std::cout << "Initial State Vector x_:" << std::endl;
  // std::cout << x_ << std::endl;

}

static int dbgCount = 0;   // for debugging

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  // std::cout << "In ProcessMeasurement(): time = " << meas_package.timestamp_ << std::endl;

  if (!is_initialized_) {
    InitializeFilter(meas_package);
    is_initialized_ = true;
    return;
  } 

  //if (dbgCount++ > 4) return;
  dbgCount++;

  // if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
  //     std::cout << " Process LIDAR Data:" << std::endl;
  // } else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR) {
  //     std::cout << " Process RADAR Data:" << std::endl;
  // }
  // std::cout << meas_package.raw_measurements_ << std::endl;

  // Figure out delta time in seconds
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // Perform Prediction Step
  Prediction(delta_t);

  // Perform Measurement Update Step
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
      UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR) {
      UpdateRadar(meas_package);
  } else {
     std::cout << " *** ERROR: Invalid Sensor Type '" << meas_package.sensor_type_ << "'..." << std::endl;   
  }

}


void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  // std::cout << "In Prediction(): delta_t =  " << delta_t << std::endl;

  //-----------------------------------------------------------
  // GENERATE AUGMENTED SIGMA POINTS
  //-----------------------------------------------------------
  // Augmended state vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  //std::cout << "x_aug = \n" << x_aug << std::endl;


  // Augmented state covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_; 
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  //std::cout << "P_aug = \n" << P_aug << std::endl;

  // Augmented Sigma point Matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1); 
  Xsig_aug.fill(0.0);

  // Calculate square root of covariance matrix
  MatrixXd L = P_aug.llt().matrixL();
  //std::cout << "L = \n" << L << std::endl;

  // Generate augmented Sigma Points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) 
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  //std::cout << "Xsig_aug = \n" << Xsig_aug << std::endl;

  //-----------------------------------------------------------
  // PREDICT SIGMA POINTS
  //-----------------------------------------------------------
  
  // Temporary storage
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2*n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    // Sigma point values
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yaw_d = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yaw_dd = Xsig_aug(6, i);
    
    // Predicted values
    double pp_x, pp_y, pv, pyaw, pyaw_d;

    // if driving in straight line, avoid division by zero
    if (fabs(yaw_d) > 0.001) 
    {
        pp_x = p_x + v / yaw_d * (sin(yaw + yaw_d * delta_t) - sin(yaw) );
        pp_y = p_y + v / yaw_d * (cos(yaw) - cos(yaw + yaw_d * delta_t) );
    }
    else 
    {  // simple process model for driving in straight line
        pp_x = p_x + v * delta_t * cos(yaw);
        pp_y = p_y + v * delta_t * sin(yaw);
    }
    pv = v;
    pyaw = yaw + yaw_d * delta_t;
    pyaw_d = yaw_d;

    // add process noise
    pp_x = pp_x + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    pp_y = pp_y + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    pv = pv + nu_a * delta_t;

    pyaw = pyaw + 0.5 * nu_yaw_dd * delta_t * delta_t;
    pyaw_d = pyaw_d + nu_yaw_dd * delta_t;

    // Save predicted sigma points into the rightmost column
    Xsig_pred(0, i) = pp_x;
    Xsig_pred(1, i) = pp_y;
    Xsig_pred(2, i) = pv;
    Xsig_pred(3, i) = pyaw;
    Xsig_pred(4, i) = pyaw_d;

  }  // for (sigma points)

  Xsig_pred_ = Xsig_pred;

  //-----------------------------------------------------------
  // PREDICT MEAN AND COVARIANCE
  //-----------------------------------------------------------

  // Calculate predicted mean
  x_.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // Calculate Predicted covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; i++)
  {
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      // Normalize angles to correct range
      while (x_diff(3) > M_PI) {
        x_diff(3) -= 2.0 * M_PI;
      }
      while (x_diff(3) < -M_PI) {
        x_diff(3) += 2.0 * M_PI;
      }

      P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

  if (dbgCount % 60 == 0) {
    // std::cout << "Predicted state: \n" << x_ << std::endl;
    // std::cout << "Predicted covariance matrix: \n" << P_ << std::endl;
  }
}


void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // std::cout << "In UpdateLiDar() " << std::endl;

  // Laser measurement dimension: p_x, p_y
  int n_z = 2;

  //-----------------------------------------------------------
  // PREDICT MEASUREMENT
  //-----------------------------------------------------------
  // Matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);   // 2 x 15

  // Vector for predicted measurement
  VectorXd z_pred = VectorXd(n_z);   // 2

  // Matrix for measurement covariance
  MatrixXd S = MatrixXd(n_z, n_z);  // 2 x 2

  // Map sigma points into measurement space
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    Zsig(0, i) = Xsig_pred_(0,i);   // for px
    Zsig(1, i) = Xsig_pred_(1,i);   // for py
  }

  // Predicted measurement mean
  z_pred.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; ++i) { 
      VectorXd z_diff = Zsig.col(i) - z_pred;
      
      S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise
  MatrixXd R = MatrixXd(n_z, n_z);  // 2 x 2
  R << std_laspx_*std_laspx_, 0,
      0, std_laspy_*std_laspy_;
  S = S + R;

  //-----------------------------------------------------------
  // UPDATE UKF MEAN AND COVARIANCE
  //-----------------------------------------------------------
  // Cross correlation matrix for sigma points between state space and
  // measurement space
  MatrixXd Tc = MatrixXd(n_x_, n_z);   // 5 x 2

  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; ++i) { 

    // Measurement difference
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman Gain
  MatrixXd K = Tc * S.inverse();

  // Measurement difference
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  // Finally update mean and covariance 
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  if (dbgCount % 60 == 0) {
    // std::cout << "After Laser Update:" << std::endl;
    // std::cout << "New state x_ = \n" << x_ << std::endl;
    // std::cout << "New covariance P_ = \n" << P_ << std::endl;
  }  

}


void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  // std::cout << "In UpdateRadar() " << std::endl;

  // Radar measurement dimension: rho, phi, and "rho dot"
  int n_z = 3;

  //-----------------------------------------------------------
  // PREDICT MEASUREMENT
  //-----------------------------------------------------------

  // Matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_ + 1);   // 3 x 15

  // Vector for predicted measurement
  VectorXd z_pred = VectorXd(n_z);   // 3

  // Matrix for measurement covariance
  MatrixXd S = MatrixXd(n_z, n_z);  // 3 x 3

  // Map sigma points into measurement space
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v_x = cos(yaw) * v;
    double v_y = sin(yaw) * v;

    // Convert polar coordinates to cartesian coordinates
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);  // rho
    Zsig(1, i) = atan2(p_y, p_x);   // phi
    Zsig(2, i) = (p_x * v_x + p_y * v_y) / sqrt(p_x * p_x + p_y * p_y);  // rho dot
  }

  // Predicted measurement mean
  z_pred.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // Measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; ++i) { 
      VectorXd z_diff = Zsig.col(i) - z_pred;

      // Normalize angles to correct range
      while (z_diff(1) > M_PI) {
        z_diff(1) -= 2.0 * M_PI;
      }
      while (z_diff(1) < -M_PI) {
        z_diff(1) += 2.0 * M_PI;
      }

      S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_, 0, 0,
      0, std_radphi_*std_radphi_, 0,
      0, 0, std_radrd_*std_radrd_;
  S = S + R;


  //-----------------------------------------------------------
  // UPDATE UKF MEAN AND COVARIANCE
  //-----------------------------------------------------------

  // Cross correlation matrix for sigma points between state space and
  // measurement space
  MatrixXd Tc = MatrixXd(n_x_, n_z);   // 5 x 3

  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_ + 1; ++i) { 

    // Measurement difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // Normalize angles to correct range
    while (z_diff(1) > M_PI) {
      z_diff(1) -= 2.0 * M_PI;
    }
    while (z_diff(1) < -M_PI) {
      z_diff(1) += 2.0 * M_PI;
    }

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Normalize angles to correct range
    while (x_diff(3) > M_PI) {
      x_diff(3) -= 2.0 * M_PI;
    }
    while (x_diff(3) < -M_PI) {
      x_diff(3) += 2.0 * M_PI;
    }

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman Gain
  MatrixXd K = Tc * S.inverse();

  // Measurement difference
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  // Normalize angles to correct range
  while (z_diff(1) > M_PI) {
    z_diff(1) -= 2.0 * M_PI;
  }
  while (z_diff(1) < -M_PI) {
    z_diff(1) += 2. * M_PI;
  }

  // Finally update mean and covariance 
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  if (dbgCount % 60 == 0) {
    // std::cout << "After Radar Update:" << std::endl;
    // std::cout << "New state x_ = \n" << x_ << std::endl;
    // std::cout << "New covariance P_ = \n" << P_ << std::endl;
  }
}