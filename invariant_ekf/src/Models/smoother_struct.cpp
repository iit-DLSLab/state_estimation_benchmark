#include "Models/smoother_struct.hpp"
#include <stdio.h>
#include <iostream>

EstimatorCommonStruct::EstimatorCommonStruct() {
  
  // IMU2BD.setZero();
  IMU2BD << -0.13014, 0.065, 0.05025; // Hound 2
  // IMU2BD << -0.13014, 0.065, 0.05025; // Hound 
  // IMU2BD <<  0.0, 0.0, 0.001*34.2; // Hound: 0.0, 0.0, 0.001*34.2;
  // IMU2BD << 0.038, -0.062, 0.184; // ANYmal
  // IMU2BD << 0.0, -0.062, 0.184; // ANYmal

  estimator_covariances_.cov_gyro_diagonal      << 1e-5, 1e-5, 1e-5;
  estimator_covariances_.cov_acc_diagonal       << 1e-2, 1e-2, 1e-2;
  estimator_covariances_.cov_slip_diagonal      << 1e-3, 1e-3, 1e-3;
  estimator_covariances_.cov_contact_diagonal   << 1e-2, 1e-2, 1e-2;
  estimator_covariances_.cov_enc_diagonal       << 1e-3, 1e-3, 1e-3;
  estimator_covariances_.cov_bias_gyro_diagonal << 1e-3, 1e-3, 1e-3;
  estimator_covariances_.cov_bias_acc_diagonal  << 1e-3, 1e-3, 1e-3;

  estimator_covariances_.cov_prior_orientation_diagonal << 1e-3, 1e-3, 1e-3;
  estimator_covariances_.cov_prior_velocity_diagonal    << 1e-3, 1e-3, 1e-3;
  estimator_covariances_.cov_prior_position_diagonal    << 1e-3, 1e-3, 1e-3;
  estimator_covariances_.cov_prior_bias_gyro_diagonal   << 1e-3, 1e-3, 1e-3;
  estimator_covariances_.cov_prior_bias_acc_diagonal    << 1e-3, 1e-3, 1e-3;

  estimator_covariances_.Covariance_Gyro.setIdentity();
  estimator_covariances_.Covariance_Acc.setIdentity();
  estimator_covariances_.Covariance_Contact.setIdentity();
  estimator_covariances_.Covariance_Slip.setIdentity();
  estimator_covariances_.Covariance_Encoder.setIdentity();
  estimator_covariances_.Covariance_Bias_Gyro.setIdentity();
  estimator_covariances_.Covariance_Bias_Acc.setIdentity();
  estimator_covariances_.Covariance_Prior_Orientation.setIdentity();
  estimator_covariances_.Covariance_Prior_Velocity.setIdentity();
  estimator_covariances_.Covariance_Prior_Position.setIdentity();
  estimator_covariances_.Covariance_Prior_Bias_Gyro.setIdentity();
  estimator_covariances_.Covariance_Prior_Bias_Acc.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Gyro.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Acc .setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Contact.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Slip.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Encoder.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Bias_Gyro.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Bias_Acc.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Orientation.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Velocity.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Position.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Gyro.setIdentity();
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Acc.setIdentity();


}

EstimatorCommonStruct::~EstimatorCommonStruct() {

}

void EstimatorCommonStruct::Covariance_Reset() {

  estimator_covariances_.Covariance_Acc.diagonal() = cov_amplifier * estimator_covariances_.cov_acc_diagonal;
  estimator_covariances_.Covariance_Gyro.diagonal() = cov_amplifier * estimator_covariances_.cov_gyro_diagonal;
  estimator_covariances_.Covariance_Slip.diagonal() = cov_amplifier * estimator_covariances_.cov_slip_diagonal;
  estimator_covariances_.Covariance_Contact.diagonal() = cov_amplifier * estimator_covariances_.cov_contact_diagonal;
  estimator_covariances_.Covariance_Encoder.diagonal() = cov_amplifier * estimator_covariances_.cov_enc_diagonal;

  estimator_covariances_.Covariance_Bias_Gyro.diagonal() = cov_amplifier * estimator_covariances_.cov_bias_gyro_diagonal;
  estimator_covariances_.Covariance_Bias_Acc.diagonal() = cov_amplifier * estimator_covariances_.cov_bias_acc_diagonal;
  estimator_covariances_.Covariance_Prior_Orientation.diagonal() = cov_amplifier * estimator_covariances_.cov_prior_orientation_diagonal;
  estimator_covariances_.Covariance_Prior_Velocity.diagonal() = cov_amplifier * estimator_covariances_.cov_prior_velocity_diagonal;
  estimator_covariances_.Covariance_Prior_Position.diagonal() = cov_amplifier * estimator_covariances_.cov_prior_position_diagonal;
  estimator_covariances_.Covariance_Prior_Bias_Gyro.diagonal() = cov_amplifier * estimator_covariances_.cov_prior_bias_gyro_diagonal;
  estimator_covariances_.Covariance_Prior_Bias_Acc.diagonal() = cov_amplifier * estimator_covariances_.cov_prior_bias_acc_diagonal;

  estimator_covariances_.SQRT_INFO_Covariance_Acc.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_acc_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_acc_diagonal(1)), 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_acc_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Gyro.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_gyro_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_gyro_diagonal(1)), 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_gyro_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Slip.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_slip_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_slip_diagonal(1)), 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_slip_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Contact.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_contact_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_contact_diagonal(1)), 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_contact_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Encoder.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_enc_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_enc_diagonal(1)), 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_enc_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Bias_Gyro.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_bias_gyro_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_bias_gyro_diagonal(1)), 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_bias_gyro_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Bias_Acc.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_bias_acc_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_bias_acc_diagonal(1)), 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_bias_acc_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Orientation.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_prior_orientation_diagonal(0)),
	  1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_prior_orientation_diagonal(1)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_prior_orientation_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Velocity.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_prior_velocity_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_prior_velocity_diagonal(1)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_prior_velocity_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Position.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_prior_position_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_prior_position_diagonal(1)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_prior_position_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Gyro.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_prior_bias_gyro_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_prior_bias_gyro_diagonal(1)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_prior_bias_gyro_diagonal(2));
  estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Acc.diagonal() << 1.0 / sqrt(cov_amplifier * estimator_covariances_.cov_prior_bias_acc_diagonal(0)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_prior_bias_acc_diagonal(1)), 1.0
	  / sqrt(cov_amplifier * estimator_covariances_.cov_prior_bias_acc_diagonal(2));


//  std::cout << "SQRT_INFO_Covariance_Acc is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Acc << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Gyro is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Gyro << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Slip is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Slip << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Contact is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Contact << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Encoder is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Encoder << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Bias_Gyro is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Bias_Gyro << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Bias_Acc is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Bias_Acc << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Prior_Orientation is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Prior_Orientation << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Prior_Velocity is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Prior_Velocity << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Prior_Position is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Prior_Position << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Prior_Bias_Gyro is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Gyro << std::endl;
//  std::cout << "SQRT_INFO_Covariance_Prior_Bias_Acc is " << std::endl << estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Acc << std::endl;



}

std::vector<Eigen::Vector3d> EstimatorCommonStruct::Variable_Contact_Cov(Eigen::Matrix<bool, -1, 1> Contact,
																Eigen::Matrix<double, -1, 1> dv) {

  std::vector<Eigen::Vector3d> contact_cov_array;
  contact_cov_array.clear();

  for (int k = 0; k < leg_no; k++) {

	Eigen::Vector3d contact_cov = cov_amplifier*estimator_covariances_.cov_contact_diagonal;
	double dv_abs;

//	std::cout << "slip_rejection_mode: " << slip_rejection_mode << std::endl;

	if (Contact(k)) {
	  if (variable_contact_cov_mode) {

		for(int iter_joint=0; iter_joint < 3; iter_joint++){
		  dv_abs = abs(dv(3 * k + iter_joint));
		  contact_cov(iter_joint) = (1+dv_abs)*cov_amplifier*estimator_covariances_.cov_contact_diagonal(iter_joint);
		  if (contact_cov(iter_joint) > cov_amplifier*estimator_covariances_.cov_slip_diagonal(iter_joint)) {
			contact_cov(iter_joint) = cov_amplifier*estimator_covariances_.cov_slip_diagonal(iter_joint);
		  }

		}

	  } else if (slip_rejection_mode) {
		if ((dv.block(3 * k, 0, 3, 1).norm() > slip_threshold)) {
		  contact_cov = cov_amplifier*estimator_covariances_.cov_slip_diagonal;
		} else {
		  contact_cov = cov_amplifier*estimator_covariances_.cov_contact_diagonal;
		}
	  } else {
		contact_cov = cov_amplifier*estimator_covariances_.cov_contact_diagonal;
	  }
	}
	contact_cov_array.push_back(contact_cov);
  }

  //cout<<"contact_cov is "<<endl<<contact_cov_array[0]<<", "<<contact_cov_array[1]<<", "<<contact_cov_array[2]<<", "<<contact_cov_array[3]<<", "<<endl;

  return contact_cov_array;
}



