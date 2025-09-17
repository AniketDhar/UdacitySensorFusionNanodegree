#include "ukf.h"
#include "Eigen/Dense"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.setZero();

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.setZero();

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.setZero();

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.95;

  // time when the state is true, in us
  time_us_ = 0;
  
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
  
  // finalize initialization of covariance and weights
  P_.setIdentity();
  P_(2, 2) = 4.0;
  P_(3, 3) = 0.7;
  P_(4, 4) = 0.7;
  const double denom = lambda_ + n_aug_;
  weights_(0) = lambda_ / denom;
  const double weight = 0.5 / denom;
  for (int i = 1; i < weights_.size(); ++i) {
    weights_(i) = weight;
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    x_.setZero();
    P_.setZero();

    constexpr double kMinPos = 1e-6;  // prevent near-zero positions
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      const double rho = meas_package.raw_measurements_(0);
      const double phi = meas_package.raw_measurements_(1);
      double px = rho * std::cos(phi);
      double py = rho * std::sin(phi);

      if (std::fabs(px) < kMinPos) {
        px = px >= 0.0 ? kMinPos : -kMinPos;
      }
      if (std::fabs(py) < kMinPos) {
        py = py >= 0.0 ? kMinPos : -kMinPos;
      }

      x_ << px, py, 0.0, 0.0, 0.0;
      P_(0, 0) = std_radr_ * std_radr_ ;
      P_(1, 1) = std_radr_ * std_radr_ ;
      P_(2, 2) = std_radrd_ * std_radrd_;
      P_(3, 3) = std_radphi_ * std_radphi_;
      P_(4, 4) = 1.1;
    } else {
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);

      if (std::fabs(px) < kMinPos) {
        px = px >= 0.0 ? kMinPos : -kMinPos;
      }
      if (std::fabs(py) < kMinPos) {
        py = py >= 0.0 ? kMinPos : -kMinPos;
      }

      x_ << px, py, 0.0, 0.0, 0.0;
      P_(0, 0) = std_laspx_ * std_laspx_;
      P_(1, 1) = std_laspy_ * std_laspy_;
      P_(2, 2) = 0.9;
      P_(3, 3) = 0.9;
      P_(4, 4) = 1.1;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  const double delta_t = (meas_package.timestamp_ - time_us_) * 1e-6;  // convert µs to s
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    if (use_radar_) {
      UpdateRadar(meas_package);
    }
  } else if (use_laser_) {
    UpdateLidar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  const double delta_t2 = delta_t * delta_t;

  // Augment the state with process noise components
  Eigen::VectorXd x_aug(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0.0;
  x_aug(n_x_ + 1) = 0.0;

  // Build the augmented covariance and factor it for sigma point generation
  Eigen::MatrixXd P_aug(n_aug_, n_aug_);
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  const double scale = std::sqrt(lambda_ + n_aug_);
  Eigen::MatrixXd L = P_aug.llt().matrixL();

  // Generate sigma points around the augmented mean
  Eigen::MatrixXd Xsig_aug(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    const Eigen::VectorXd offset = scale * L.col(i);
    Xsig_aug.col(i + 1) = x_aug + offset;
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - offset;
  }

  // Predict sigma points using CTRV model with process noise
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    const double px = Xsig_aug(0, i);
    const double py = Xsig_aug(1, i);
    const double v = Xsig_aug(2, i);
    const double yaw = Xsig_aug(3, i);
    const double yawd = Xsig_aug(4, i);
    const double nu_a = Xsig_aug(5, i);
    const double nu_yawdd = Xsig_aug(6, i);

    double px_p;
    double py_p;

    if (std::fabs(yawd) > 1e-6) {  // handle small yaw rates without division spikes
      const double yaw_dt = yaw + yawd * delta_t;
      const double ratio = v / yawd;
      px_p = px + ratio * (std::sin(yaw_dt) - std::sin(yaw));
      py_p = py + ratio * (-std::cos(yaw_dt) + std::cos(yaw));
    } else {
      const double v_dt = v * delta_t;
      px_p = px + v_dt * std::cos(yaw);
      py_p = py + v_dt * std::sin(yaw);
    }

    px_p += 0.5 * delta_t2 * std::cos(yaw) * nu_a;
    py_p += 0.5 * delta_t2 * std::sin(yaw) * nu_a;
    const double v_p = v + delta_t * nu_a;
    const double yaw_p = yaw + yawd * delta_t + 0.5 * delta_t2 * nu_yawdd;
    const double yawd_p = yawd + delta_t * nu_yawdd;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // Recover the predicted state mean from the sigma points
  x_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  x_(3) = std::atan2(std::sin(x_(3)), std::cos(x_(3)));  // normalize yaw

  // Recover the predicted covariance, normalizing yaw deviations
  P_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = std::atan2(std::sin(x_diff(3)), std::cos(x_diff(3)));
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  constexpr int n_z = 2;
  static const Eigen::Matrix<double, n_z, 5> H =
      (Eigen::Matrix<double, n_z, 5>() << 1., 0., 0., 0., 0., 0., 1., 0., 0., 0.)
          .finished();

  const Eigen::Vector2d z = meas_package.raw_measurements_.head<n_z>();
  const Eigen::Vector2d z_pred = x_.head<n_z>();
  const Eigen::Vector2d z_diff = z - z_pred;

  Eigen::Matrix2d S = P_.topLeftCorner<n_z, n_z>();
  S(0, 0) += std_laspx_ * std_laspx_;
  S(1, 1) += std_laspy_ * std_laspy_;

  // Analytically invert the 2x2 innovation covariance for performance
  const double det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
  const double inv_det = 1.0 / det;
  Eigen::Matrix2d S_inv;
  S_inv(0, 0) = S(1, 1) * inv_det;
  S_inv(0, 1) = -S(0, 1) * inv_det;
  S_inv(1, 0) = -S(1, 0) * inv_det;
  S_inv(1, 1) = S(0, 0) * inv_det;

  const Eigen::Matrix<double, 5, n_z> PHt = P_.block<5, n_z>(0, 0);
  const Eigen::Matrix<double, n_z, 5> HP = P_.topRows<n_z>();
  const Eigen::Matrix<double, 5, n_z> K = PHt * S_inv;

  x_ += K * z_diff;
  x_(3) = std::atan2(std::sin(x_(3)), std::cos(x_(3)));
  P_ -= K * HP;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  constexpr int n_z = 3;
  const int n_sig = 2 * n_aug_ + 1;

  Eigen::Matrix<double, n_z, Eigen::Dynamic> Zsig(n_z, n_sig);
  // Transform predicted sigma points into radar measurement space
  for (int i = 0; i < n_sig; ++i) {
    const double px = Xsig_pred_(0, i);
    const double py = Xsig_pred_(1, i);
    const double v = Xsig_pred_(2, i);
    const double yaw = Xsig_pred_(3, i);

    const double rho = std::sqrt(px * px + py * py);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    double rhod = 0.0;
    if (rho > 1e-6) {  // skip radial velocity calculation when range is tiny
      rhod = (px * cos_yaw * v + py * sin_yaw * v) / rho;
    }

    Zsig(0, i) = rho;
    Zsig(1, i) = std::atan2(py, px);
    Zsig(2, i) = rhod;
  }

  Eigen::Matrix<double, n_z, 1> z_pred;
  z_pred.setZero();
  for (int i = 0; i < n_sig; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }
  z_pred(1) = std::atan2(std::sin(z_pred(1)), std::cos(z_pred(1)));

  Eigen::Matrix<double, n_z, n_z> S;
  S.setZero();
  for (int i = 0; i < n_sig; ++i) {
    Eigen::Matrix<double, n_z, 1> z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = std::atan2(std::sin(z_diff(1)), std::cos(z_diff(1)));
    S.noalias() += weights_(i) * z_diff * z_diff.transpose();
  }

  S(0, 0) += std_radr_ * std_radr_;
  S(1, 1) += std_radphi_ * std_radphi_;
  S(2, 2) += std_radrd_ * std_radrd_;

  Eigen::Matrix<double, 5, n_z> Tc;
  Tc.setZero();
  // Compute the state-measurement cross covariance
  for (int i = 0; i < n_sig; ++i) {
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = std::atan2(std::sin(x_diff(3)), std::cos(x_diff(3)));

    Eigen::Matrix<double, n_z, 1> z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = std::atan2(std::sin(z_diff(1)), std::cos(z_diff(1)));

    Tc.noalias() += weights_(i) * x_diff * z_diff.transpose();
  }

  const Eigen::Matrix<double, n_z, n_z> S_inv = S.inverse();
  const Eigen::Matrix<double, 5, n_z> K = Tc * S_inv;

  Eigen::Matrix<double, n_z, 1> z_diff =
      meas_package.raw_measurements_.head<n_z>() - z_pred;
  z_diff(1) = std::atan2(std::sin(z_diff(1)), std::cos(z_diff(1)));

  x_ += K * z_diff;
  x_(3) = std::atan2(std::sin(x_(3)), std::cos(x_(3)));
  P_ -= K * S * K.transpose();
}
