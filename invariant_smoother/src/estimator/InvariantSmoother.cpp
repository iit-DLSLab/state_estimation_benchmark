// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.



#include "invariant_smoother/estimator/InvariantSmoother.hpp"
#include <string>
#include <utility>

#include <malloc.h>

//---------------------------------




void Inv_Factors::Batch_Initialize(ROBOT_STATES *RS, const ROBOT_STATES &RS_Prior,
								   const Eigen::Matrix<double, Eigen::Dynamic, 1> &Estimation_Z,
								   factor_info *_fac_info,
								   const EstimatorCommonStruct &estimator_common_struct) {

  gravity << 0, 0, -9.81;
  frame_count = Estimation_Z.rows() / num_z - 1;
  estimator_common_struct_ = estimator_common_struct;
  dt = estimator_common_struct_.dt;

  fac_info = _fac_info;

  RS_temp = RS;
//    RS_temp = new ROBOT_STATES[frame_count+1];
//    for(int i=0; i<=frame_count; i++){
//        RS_temp[i]  =_RS[i];
//    }


  //prior_RVP--------------------------------------------------------------------------------------
  X_Prior.resize(5, 5);
  X_Prior.setIdentity();

  X_Prior.block<3, 3>(0, 0) = RS_Prior.Rotation;
  X_Prior.block<3, 1>(0, 3) = RS_Prior.Velocity;
  X_Prior.block<3, 1>(0, 4) = RS_Prior.Position;

  SQRT_INFO_Prior.setZero();
  SQRT_INFO_Prior.block<3, 3>(0, 0) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Orientation;
  SQRT_INFO_Prior.block<3, 3>(3, 3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Velocity;
  SQRT_INFO_Prior.block<3, 3>(6, 6) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Position;

  //prior_bias--------------------------------------------------------------------------------------
  Bias_Prior << RS_Prior.Bias_Gyro, RS_Prior.Bias_Acc;

  SQRT_INFO_Prior_BIAS.setZero();
  SQRT_INFO_Prior_BIAS.block<3, 3>(0, 0) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Gyro;
  SQRT_INFO_Prior_BIAS.block<3, 3>(3, 3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Acc;

}

void Inv_Factors::Batch_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization, ROBOT_STATES *RS,
														Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
														Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient,
														double &cost) {

  if (!is_for_marginalization) {

	RS_temp = RS;

	if (cost != 0) {

	  for (int p = 0; p < frame_count; p++) {
		std::vector<Eigen::Vector3d> contact_cov_array_temp =
			estimator_common_struct_.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
		int count = 0;
		for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		  if (RS_temp[p].Hard_Contact(k) && RS_temp[p + 1].Hard_Contact(k)) {

			Eigen::Matrix3d temp_sqrt_contact_info;
			temp_sqrt_contact_info.setZero();
			temp_sqrt_contact_info.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)),
				1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
			fac_info[p].prop_primitive_sqrt_info.block<3, 3>(15 + 3 * count, 15 + 3 * count) =temp_sqrt_contact_info;
			count++;

		  }
		}
	  }

	}

  }

  cost = 0;

  if (!is_for_marginalization) {

	Hessian.setZero();
	gradient.setZero();

	Invariant_Prior_RVP_Factor(Hessian, gradient, cost);

	Invariant_Prior_Bias_Factor(Hessian, gradient, cost);

	for (int i = 0; i < frame_count; i++) {
	  Invariant_Propagation_Factor(i, Hessian, gradient, cost);
	}

	for (int i = 0; i <= frame_count; i++) {
	  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		if (RS_temp[i].Hard_Contact(k)) {

		  Invariant_Measurement_Factor(i, k, Hessian, gradient, cost);

		}

	  }
	  if (RS_temp[i].GPS_In) {
		Invariant_GPS_Measurement_Factor(i, Hessian, gradient, cost);
	  }
	}
  } else {

	Hessian = -Hessian;
	gradient = -gradient;

	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS_temp[1].Hard_Contact(k)) {

		Invariant_Measurement_Factor(1, k, Hessian, gradient, cost);

	  }

	}
	if (RS_temp[1].GPS_In) {
	  Invariant_GPS_Measurement_Factor(1, Hessian, gradient, cost);
	}

	if (frame_count > 1) {

	  Invariant_Propagation_Factor(1, Hessian, gradient, cost);

	}

	Hessian = -Hessian;
	gradient = -gradient;

  }

}

void Inv_Factors::Marg_Initialize(ROBOT_STATES *RS,
								  const Eigen::Matrix<double, Eigen::Dynamic, 1> &Estimation_Z,
								  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _H,
								  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Marg_Hessian,
								  const Eigen::Matrix<double, Eigen::Dynamic, 1> _b,
								  const Eigen::Matrix<double, Eigen::Dynamic, 1> Marg_Gradient,
								  factor_info *_fac_info,
								  const EstimatorCommonStruct &estimator_common_struct) {

  gravity << 0, 0, -9.81;
  frame_count = Estimation_Z.rows() / num_z - 1;
  estimator_common_struct_ = estimator_common_struct;
  dt = estimator_common_struct_.dt;

  marginalization_flag = true;

  fac_info = _fac_info;

  RS_temp = RS;



  //prior_RVP--------------------------------------------------------------------------------------

  X_Prior.resize(5 + RS_temp[0].contact_leg_num, 5 + RS_temp[0].contact_leg_num);
  X_Prior.setIdentity();

  X_Prior.block<3, 3>(0, 0) = RS_temp[0].Rotation;
  X_Prior.block<3, 1>(0, 3) = RS_temp[0].Velocity;
  X_Prior.block<3, 1>(0, 4) = RS_temp[0].Position;

  int count = 0;
  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	if (RS_temp[0].Hard_Contact(k)) {
	  X_Prior.block<3, 1>(0, 5 + count) = RS_temp[0].d.block(3 * k, 0, 3, 1);
	  count++;
	}
  }

  //prior_bias--------------------------------------------------------------------------------------
  Bias_Prior << RS_temp[0].Bias_Gyro, RS_temp[0].Bias_Acc;


  //Marginalization factor--------------------------------------------------------------------------

  Hessian_Marg.resizeLike(Marg_Hessian);
  Hessian_Marg.setZero();
//  Hessian_Marg = _H.transpose()*_H;
  Hessian_Marg = Marg_Hessian;
  H = _H;

  gradient_Marg.resizeLike(Marg_Gradient);
  gradient_Marg.setZero();
//  gradient_Marg = _H.transpose() * _b;
  gradient_Marg = Marg_Gradient;
  b = _b;

}

void Inv_Factors::Marg_Update_n_Get_Gradient_Hess_Cost_Debug(bool is_for_marginalization,
													   ROBOT_STATES *_RS,
													   Eigen::Matrix<double, -1, 1> Zeta_Xi,
													   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
													   Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient,
													   double &cost) {

  hasnan=true;

  if (!is_for_marginalization) {

	RS_temp = _RS;

	if (cost != 0) {

	  for (int p = 0; p < frame_count; p++) {
		std::vector<Eigen::Vector3d> contact_cov_array_temp =
			estimator_common_struct_.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
		int count = 0;
		for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		  if (RS_temp[p].Hard_Contact(k) && RS_temp[p + 1].Hard_Contact(k)) {

			Eigen::Matrix3d temp_sqrt_contact_info;
			temp_sqrt_contact_info.setZero();
			temp_sqrt_contact_info.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)),
				1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
			fac_info[p].prop_primitive_sqrt_info.block<3, 3>(15 + 3 * count, 15 + 3 * count) =temp_sqrt_contact_info;
			count++;

		  }
		}
	  }

	}
  }

  cost = 0;

  if (is_for_marginalization == false) {

	Hessian.setZero();
	gradient.setZero();

	//Hessian.diagonal().setConstant(1e-10);

	Bias_0_bar.block(0, 0, 3, 1) = RS_temp[0].Bias_Gyro + Zeta_Xi.block(0, 0, 3, 1);
	Bias_0_bar.block(3, 0, 3, 1) = RS_temp[0].Bias_Acc + Zeta_Xi.block(3, 0, 3, 1);

	Eigen::Matrix<double, -1, 1> Xi_0;
	Xi_0.resize(9 + 3 * RS_temp[0].contact_leg_num, 1);
	Xi_0 = Zeta_Xi.block(6, 0, 9 + 3 * RS_temp[0].contact_leg_num, 1);

	Eigen::MatrixXd X_prior_inverse;
	X_prior_inverse.resize(5 + RS_temp[0].contact_leg_num, 5 + RS_temp[0].contact_leg_num);
	X_prior_inverse.setIdentity();

	X_prior_inverse.block<3, 3>(0, 0) = X_Prior.block<3,3>(0,0).transpose();
	X_prior_inverse.block<3, 1>(0, 3) = -X_Prior.block<3,3>(0,0).transpose()*X_Prior.block<3,1>(0,3);
	X_prior_inverse.block<3, 1>(0, 4) = -X_Prior.block<3,3>(0,0).transpose()*X_Prior.block<3,1>(0,4);

	X_0_bar.resize(5 + RS_temp[0].contact_leg_num, 5 + RS_temp[0].contact_leg_num);
	X_0_bar.setIdentity();
	X_0_bar.block<3, 3>(0, 0) = RS_temp[0].Rotation;
	X_0_bar.block<3, 1>(0, 3) = RS_temp[0].Velocity;
	X_0_bar.block<3, 1>(0, 4) = RS_temp[0].Position;
	int count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS_temp[0].Hard_Contact(k)) {
		X_0_bar.block<3, 1>(0, 5 + count) = RS_temp[0].d.block(3 * k, 0, 3, 1);
		X_prior_inverse.block<3,1>(0,5+count) = -X_Prior.block<3,3>(0,0).transpose()*X_Prior.block<3,1>(0,5+count);
		count++;
	  }
	}
	X_0_bar = Expm_seK_Vec(Xi_0, 2 + RS_temp[0].contact_leg_num) * X_0_bar;




	Eigen::Matrix<double, -1, 1> Bias_X_bar_X_vee_inv_retracted,residual;
	Bias_X_bar_X_vee_inv_retracted.resize(6 + 9 + 3 * RS_temp[0].contact_leg_num, 1);
	Bias_X_bar_X_vee_inv_retracted.setZero();
	Bias_X_bar_X_vee_inv_retracted.block(0, 0, 6, 1) = Bias_0_bar - Bias_Prior;
	Bias_X_bar_X_vee_inv_retracted.block(6, 0, 9 + 3 * RS_temp[0].contact_leg_num, 1) =
		Logm_seK_Vec(X_0_bar * X_prior_inverse, 2 + RS_temp[0].contact_leg_num);

	Hessian.block(0, 0, RS_temp[0].para_size, RS_temp[0].para_size) = Hessian_Marg;
	gradient.block(0, 0, RS_temp[0].para_size, 1) = Hessian_Marg * (Bias_X_bar_X_vee_inv_retracted) + gradient_Marg;

	residual = Hessian_Marg*gradient_Marg + Bias_X_bar_X_vee_inv_retracted;

	double temp_cost = (b + H * Bias_X_bar_X_vee_inv_retracted).norm();

	cost += temp_cost * temp_cost;



	std::cout << "Bias_X_bar_X_vee_inv_retracted:\n" << Bias_X_bar_X_vee_inv_retracted << std::endl;

	std::cout << "Hessian_Marg:\n" << Hessian_Marg << std::endl;
	std::cout << "gradient_Marg:\n" << gradient_Marg << std::endl;
	std::cout << "H:\n" << H << std::endl;
	std::cout << "b:\n" << b << std::endl;

	{
	  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hessian);
	  double cond = svd.singularValues()(0)
		  / svd.singularValues()(svd.singularValues().size()-1);

	  std::cout << "cond:\n" << cond << std::endl;
	}




	//cout<<"Now marg res is "<<cost<<endl;


	for (int i = 0; i < frame_count; i++) {
	  Invariant_Propagation_Factor(i, Hessian, gradient, cost);

	  {
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hessian);
		double cond = svd.singularValues()(0)
			/ svd.singularValues()(svd.singularValues().size()-1);

		std::cout << "cond:\n" << cond << std::endl;
	  }

	}

	for (int i = 0; i <= frame_count; i++) {
	  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		if (RS_temp[i].Hard_Contact(k)) {

		  Invariant_Measurement_Factor(i, k, Hessian, gradient, cost);

		  {
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hessian);
			double cond = svd.singularValues()(0)
				/ svd.singularValues()(svd.singularValues().size()-1);

			std::cout << "cond:\n" << cond << std::endl;
		  }

		}

	  }
	  if (RS_temp[i].GPS_In) {
		Invariant_GPS_Measurement_Factor(i, Hessian, gradient, cost);
	  }
	}

	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {

	  for (int xy_z = 1; xy_z < 3; xy_z++) {

		double v_threshold = 0;
		double a_threshold = 0;

		if (xy_z == 1) {
		  v_threshold = estimator_common_struct_.long_term_v_threshold;
		  a_threshold = estimator_common_struct_.long_term_a_threshold;
		} else if (xy_z == 2) {
		  v_threshold = estimator_common_struct_.long_term_v_threshold;
		  a_threshold = estimator_common_struct_.long_term_a_threshold;
		}

		int start = 0, end = -1;
		int contact_count = 0;

		for (int i = frame_count; i > 1; i--) {

		  double dv_mag_i = 0;
		  double da_mag_i = 0;
		  if (xy_z == 1) {
			dv_mag_i = fabs(RS_temp[i].d_v.block(3 * k, 0, 2, 1).norm());
			da_mag_i =
				fabs(((RS_temp[i].d_v.block(3 * k, 0, 2, 1) - RS_temp[i - 1].d_v.block(3 * k, 0, 2, 1)) / dt).norm());
		  } else if (xy_z == 2) {
			dv_mag_i = fabs(RS_temp[i].d_v(3 * k + xy_z));
			da_mag_i = fabs((RS_temp[i].d_v(3 * k + xy_z) - RS_temp[i - 1].d_v(3 * k + xy_z)) / dt);
		  }

		  //cout<<i<<endl;
		  if (contact_count == 0 && RS_temp[i].Hard_Contact(k)
			  && (dv_mag_i < v_threshold)
			  && (da_mag_i < a_threshold)
			  ) {
			end = i;
			contact_count++;

		  } else if (contact_count > 0 && RS_temp[i].Hard_Contact(k)
			  && (dv_mag_i < v_threshold)
			  && (da_mag_i < a_threshold)
			  ) {

			contact_count++;

		  } else if (contact_count > 0 && (
			  !RS_temp[i].Hard_Contact(k)
				  || (dv_mag_i > v_threshold)
				  || (da_mag_i > a_threshold)
		  )) {
			contact_count = 0;

			start = i + 1;
		  }

		  if ((start > 0) && (end - start > 1)) {

			Long_Term_Stationary_Foot_Factor(start, end, xy_z, k, Hessian, gradient, cost);
			start = 0;
			end = -1;

		  }

		}
	  }

	}

  } else {

	Hessian = -Hessian;
	gradient = -gradient;

	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS_temp[1].Hard_Contact(k)) {

		Invariant_Measurement_Factor(1, k, Hessian, gradient, cost);

	  }
	}
	if (RS_temp[1].GPS_In) {
	  Invariant_GPS_Measurement_Factor(1, Hessian, gradient, cost);
	}

	if (frame_count > 1) {

	  Invariant_Propagation_Factor(1, Hessian, gradient, cost);

	}

	Hessian = -Hessian;
	gradient = -gradient;

  }

  hasnan=false;

}


void Inv_Factors::Marg_Update_n_Get_Gradient_Hess_Cost(bool is_for_marginalization,
													   ROBOT_STATES *_RS,
													   Eigen::Matrix<double, -1, 1> Zeta_Xi,
													   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
													   Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient,
													   double &cost) {

  if (is_for_marginalization == false) {

	RS_temp = _RS;

//	if (cost != 0) {

	  for (int p = 0; p < frame_count; p++) {
		std::vector<Eigen::Vector3d> contact_cov_array_temp =
			estimator_common_struct_.Variable_Contact_Cov(RS_temp[p].Hard_Contact, RS_temp[p].d_v);
		int count = 0;
		for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		  if (RS_temp[p].Hard_Contact(k) && RS_temp[p + 1].Hard_Contact(k)) {

			Eigen::Matrix3d temp_sqrt_contact_info;
			temp_sqrt_contact_info.setZero();
			temp_sqrt_contact_info.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)),
				1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
			fac_info[p].prop_primitive_sqrt_info.block<3, 3>(15 + 3 * count, 15 + 3 * count) =temp_sqrt_contact_info;
			count++;

		  }
		}
	  }

//	}
  }

  cost = 0;

  if (is_for_marginalization == false) {

	Hessian.setZero();
	gradient.setZero();

	Bias_0_bar.block(0, 0, 3, 1) = RS_temp[0].Bias_Gyro + Zeta_Xi.block(0, 0, 3, 1);
	Bias_0_bar.block(3, 0, 3, 1) = RS_temp[0].Bias_Acc + Zeta_Xi.block(3, 0, 3, 1);

	Eigen::Matrix<double, -1, 1> Xi_0;
	Xi_0.resize(9 + 3 * RS_temp[0].contact_leg_num, 1);
	Xi_0 = Zeta_Xi.block(6, 0, 9 + 3 * RS_temp[0].contact_leg_num, 1);
	Eigen::MatrixXd X_prior_inverse;
	X_prior_inverse.resize(5 + RS_temp[0].contact_leg_num, 5 + RS_temp[0].contact_leg_num);
	X_prior_inverse.setIdentity();

	X_prior_inverse.block<3, 3>(0, 0) = X_Prior.block<3,3>(0,0).transpose();
	X_prior_inverse.block<3, 1>(0, 3) = -X_Prior.block<3,3>(0,0).transpose()*X_Prior.block<3,1>(0,3);
	X_prior_inverse.block<3, 1>(0, 4) = -X_Prior.block<3,3>(0,0).transpose()*X_Prior.block<3,1>(0,4);
	X_0_bar.resize(5 + RS_temp[0].contact_leg_num, 5 + RS_temp[0].contact_leg_num);
	X_0_bar.setIdentity();
	X_0_bar.block<3, 3>(0, 0) = RS_temp[0].Rotation;
	X_0_bar.block<3, 1>(0, 3) = RS_temp[0].Velocity;
	X_0_bar.block<3, 1>(0, 4) = RS_temp[0].Position;
	int count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS_temp[0].Hard_Contact(k)) {
		X_0_bar.block<3, 1>(0, 5 + count) = RS_temp[0].d.block(3 * k, 0, 3, 1);
		X_prior_inverse.block<3,1>(0,5+count) = -X_Prior.block<3,3>(0,0).transpose()*X_Prior.block<3,1>(0,5+count);
		count++;
	  }
	}
	X_0_bar = Expm_seK_Vec(Xi_0, 2 + RS_temp[0].contact_leg_num) * X_0_bar;

	Eigen::Matrix<double, -1, 1> Bias_X_bar_X_vee_inv_retracted,residual;
	Bias_X_bar_X_vee_inv_retracted.resize(6 + 9 + 3 * RS_temp[0].contact_leg_num, 1);
	Bias_X_bar_X_vee_inv_retracted.setZero();
	Bias_X_bar_X_vee_inv_retracted.block(0, 0, 6, 1) = Bias_0_bar - Bias_Prior;
	Bias_X_bar_X_vee_inv_retracted.block(6, 0, 9 + 3 * RS_temp[0].contact_leg_num, 1) =
		Logm_seK_Vec(X_0_bar * X_prior_inverse, 2 + RS_temp[0].contact_leg_num);

	Hessian.block(0, 0, RS_temp[0].para_size, RS_temp[0].para_size) = Hessian_Marg;
	gradient.block(0, 0, RS_temp[0].para_size, 1) = Hessian_Marg * (Bias_X_bar_X_vee_inv_retracted) + gradient_Marg;


	residual = Hessian_Marg*Bias_X_bar_X_vee_inv_retracted + gradient_Marg;

	double temp_cost = (b + H * Bias_X_bar_X_vee_inv_retracted).norm();

	cost += temp_cost * temp_cost;

//	std::cout << residual.transpose()*residual << ", " << temp_cost * temp_cost << std::endl;

	//cout<<"Now marg res is "<<cost<<endl;


	for (int i = 0; i < frame_count; i++) {
	  Invariant_Propagation_Factor(i, Hessian, gradient, cost);
	}

	for (int i = 0; i <= frame_count; i++) {
	  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		if (RS_temp[i].Hard_Contact(k)) {

		  Invariant_Measurement_Factor(i, k, Hessian, gradient, cost);

		}

	  }
	  if (RS_temp[i].GPS_In) {
		Invariant_GPS_Measurement_Factor(i, Hessian, gradient, cost);
	  }
	}

	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {

	  for (int xy_z = 1; xy_z < 3; xy_z++) {

		double v_threshold = 0;
		double a_threshold = 0;

		if (xy_z == 1) {
		  v_threshold = estimator_common_struct_.long_term_v_threshold;
		  a_threshold = estimator_common_struct_.long_term_a_threshold;
		} else if (xy_z == 2) {
		  v_threshold = estimator_common_struct_.long_term_v_threshold;
		  a_threshold = estimator_common_struct_.long_term_a_threshold;
		}

		int start = 0, end = -1;
		int contact_count = 0;

		for (int i = frame_count; i > 1; i--) {

		  double dv_mag_i = 0;
		  double da_mag_i = 0;
		  if (xy_z == 1) {
			dv_mag_i = fabs(RS_temp[i].d_v.block(3 * k, 0, 2, 1).norm());
			da_mag_i =
				fabs(((RS_temp[i].d_v.block(3 * k, 0, 2, 1) - RS_temp[i - 1].d_v.block(3 * k, 0, 2, 1)) / dt).norm());
		  } else if (xy_z == 2) {
			dv_mag_i = fabs(RS_temp[i].d_v(3 * k + xy_z));
			da_mag_i = fabs((RS_temp[i].d_v(3 * k + xy_z) - RS_temp[i - 1].d_v(3 * k + xy_z)) / dt);
		  }

		  //cout<<i<<endl;
		  if (contact_count == 0 && RS_temp[i].Hard_Contact(k)
			  && (dv_mag_i < v_threshold)
			  && (da_mag_i < a_threshold)
			  ) {
			end = i;
			contact_count++;

		  } else if (contact_count > 0 && RS_temp[i].Hard_Contact(k)
			  && (dv_mag_i < v_threshold)
			  && (da_mag_i < a_threshold)
			  ) {

			contact_count++;

		  } else if (contact_count > 0 && (
			  !RS_temp[i].Hard_Contact(k)
				  || (dv_mag_i > v_threshold)
				  || (da_mag_i > a_threshold)
		  )) {
			contact_count = 0;

			start = i + 1;
		  }

		  if ((start > 0) && (end - start > 1)) {

			Long_Term_Stationary_Foot_Factor(start, end, xy_z, k, Hessian, gradient, cost);
			start = 0;
			end = -1;

		  }

		}
	  }

	}

  } else {

	Hessian = -Hessian;
	gradient = -gradient;

	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS_temp[1].Hard_Contact(k)) {

		Invariant_Measurement_Factor(1, k, Hessian, gradient, cost);

	  }
	}
	if (RS_temp[1].GPS_In) {
	  Invariant_GPS_Measurement_Factor(1, Hessian, gradient, cost);
	}

	if (frame_count > 1) {

	  Invariant_Propagation_Factor(1, Hessian, gradient, cost);

	}

	Hessian = -Hessian;
	gradient = -gradient;

  }

}

void Inv_Factors::Invariant_Propagation_Factor(int i,
											   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
											   Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost) {

  int j = i + 1;

  Eigen::MatrixXd A;
  A.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
  A.setZero();

  A.block(9, 6, 3, 3) = Hat_so3(gravity);
  A.block(12, 6, 3, 3) = 0.5 * Hat_so3(gravity) * dt;
  A.block(12, 9, 3, 3) = Eigen::Matrix3d::Identity();

  A.block(6, 0, 3, 3) = -RS_temp[i].Rotation;

  A.block(9, 0, 3, 3) = -Hat_so3(RS_temp[i].Velocity) * RS_temp[i].Rotation;
  A.block(9, 3, 3, 3) = -RS_temp[i].Rotation;

  A.block(12, 0, 3, 3) = -Hat_so3(RS_temp[i].Position) * RS_temp[i].Rotation;

  int count = 0;
  int count_i = 0;
  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	if (RS_temp[i].Hard_Contact(k)) {
	  if (RS_temp[j].Hard_Contact(k)) {

		Eigen::Vector3d d;
		d = RS_temp[i].d.block(3 * k, 0, 3, 1);
		A.block(15 + 3 * count, 0, 3, 3) = -Hat_so3(d) * RS_temp[i].Rotation;
		count++;
	  }

	  count_i++;
	}
  }

  //Covariance Calculation-------------------------------------------------------------------------------
  //Adjoint of X_bar---------------------------------------------------------------------

  Eigen::MatrixXd Adx_inv_augmented;

  Adx_inv_augmented.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
  Adx_inv_augmented.setZero();

  Adx_inv_augmented.block(0, 0, 6, 6).setIdentity();

  for (int k = 0; k < 3 + fac_info[i].shared_contact; k++) {
	Adx_inv_augmented.block<3, 3>(6 + 3 * k, 6 + 3 * k) = RS_temp[i].Rotation.transpose();
  }

  Adx_inv_augmented.block<3, 3>(9, 6) = -RS_temp[i].Rotation.transpose() * Hat_so3(RS_temp[i].Velocity);
  Adx_inv_augmented.block<3, 3>(12, 6) = -RS_temp[i].Rotation.transpose() * Hat_so3(RS_temp[i].Position);

  count = 0;
  count_i = 0;
  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	if (RS_temp[i].Hard_Contact(k)) {
	  if (RS_temp[j].Hard_Contact(k)) {

		Eigen::Vector3d d;
		d = RS_temp[i].d.block(3 * k, 0, 3, 1);
		Adx_inv_augmented.block<3, 3>(15 + 3 * count, 6) = -RS_temp[i].Rotation.transpose() * Hat_so3(d);
		count++;
	  }

	  count_i++;
	}
  }

  Eigen::MatrixXd sqrt_info;
  sqrt_info.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
  sqrt_info = fac_info[i].prop_primitive_sqrt_info;

  if (fac_info[i].prop_primitive_sqrt_info.cols() > 15) {
//std::cout << "prop sqrt_info "<< sqrt_info(15,15) <<" at "<<i<< std::endl;
  }

  if (hasnan) {
	std::cout << "prop_primitive_sqrt_info:\n" << fac_info[i].prop_primitive_sqrt_info << std::endl;
  }

//  if(hasnan)
//  {
//	std::cout << "sqrt_info_init:\n" << sqrt_info <<std::endl;
//  }


  sqrt_info = sqrt_info * Adx_inv_augmented / dt;


//  if(hasnan)
//  {
//	std::cout << "sqrt_info_after:\n" << sqrt_info <<std::endl;
//  }
  //residual---------------------------------------------------------------------------------------

  Eigen::VectorXd residual;
  residual.resize(fac_info[i].prop_res_size, 1);

  Eigen::MatrixXd f_X_i;
  f_X_i.resize(5 + fac_info[i].shared_contact, 5 + fac_info[i].shared_contact);
  f_X_i.setIdentity();

  Eigen::Vector3d IMU_Gyro_i, IMU_Acc_i;
  IMU_Gyro_i = fac_info[i].Z.block(0, 0, 3, 1);
  IMU_Acc_i = fac_info[i].Z.block(3, 0, 3, 1);

  Eigen::Vector3d integrated_acc, integrated_gyro, integrated_acc2, integrated_vel;
  integrated_acc.setZero();
  integrated_acc2.setZero();
  integrated_gyro.setZero();
  integrated_vel.setZero();

  Eigen::Vector3d integrated_compare_acc, integrated_compare_gyro, integrated_compare_acc2, integrated_compare_vel;
  integrated_compare_acc.setZero();
  integrated_compare_gyro.setZero();
  integrated_compare_gyro.setZero();
  integrated_compare_vel.setZero();
  int buffer_size = 0;

  buffer_size = fac_info[i].z_buffer_.size();

//  std::cout << "buffer_size: " << buffer_size << std::endl;
//  std::cout << "preintegration_dt: " <<dt/fac_info[i].z_buffer_.size() << std::endl;

  if (i == 0 && marginalization_flag) {

	Eigen::Matrix3d R0;
	Eigen::Vector3d v0, p0, bg0, ba0;
	R0 = X_0_bar.block(0, 0, 3, 3);
	v0 = X_0_bar.block(0, 3, 3, 1);
	p0 = X_0_bar.block(0, 4, 3, 1);
	bg0 = Bias_0_bar.block(0, 0, 3, 1);
	ba0 = Bias_0_bar.block(3, 0, 3, 1);

	if (preintegration_mode_) {

	  if (buffer_size != 0) {
		for (int preintegration_iter = 0; preintegration_iter < fac_info[j].z_buffer_.size(); preintegration_iter++) {
		  integrated_vel += (v0 + integrated_acc) * (dt / fac_info[j].z_buffer_.size());
		  integrated_acc += R0 * Expm_Vec(integrated_gyro) *
			  (fac_info[j].z_buffer_[preintegration_iter].block(3, 0, 3, 1) - ba0)
			  * (dt / fac_info[j].z_buffer_.size());
		  integrated_acc2 += 0.5 * R0 * Expm_Vec(integrated_gyro)
			  * (fac_info[j].z_buffer_[preintegration_iter].block(3, 0, 3, 1) - ba0)
			  * (dt / fac_info[j].z_buffer_.size()) * (dt / fac_info[j].z_buffer_.size());
		  integrated_gyro +=
			  (fac_info[j].z_buffer_[preintegration_iter].block(0, 0, 3, 1) - bg0)
				  * (dt / fac_info[j].z_buffer_.size());
		}
	  }

	}

	integrated_compare_acc = (IMU_Acc_i - ba0) * dt;
	integrated_compare_gyro = (IMU_Gyro_i - bg0) * dt;
	integrated_compare_acc2 = 0.5 * R0 * (IMU_Acc_i - ba0) * dt * dt;
	integrated_compare_vel = v0 * dt;

	if (preintegration_mode_) {
	  if (buffer_size != 0) {
		f_X_i.block<3, 3>(0, 0) = R0 * Expm_Vec(integrated_gyro);
	  } else {
		f_X_i.block<3, 3>(0, 0) = R0 * Expm_Vec((IMU_Gyro_i - bg0) * dt);
	  }
	} else {
	  f_X_i.block<3, 3>(0, 0) = R0 * Expm_Vec((IMU_Gyro_i - bg0) * dt);
	}

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(f_X_i.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
	f_X_i.block<3, 3>(0, 0) = svd.matrixU() * svd.matrixV().transpose();

	if (preintegration_mode_) {
	  if (buffer_size != 0) {
		f_X_i.block<3, 1>(0, 3) = v0
			+ integrated_acc
			+ gravity * dt;

		f_X_i.block<3, 1>(0, 4) = p0 + integrated_vel
			+ integrated_acc2 + 0.5 * gravity * dt * dt;
	  } else {
		f_X_i.block<3, 1>(0, 3) = v0
			+ R0 //* Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
				* (IMU_Acc_i - ba0) * dt
			+ gravity * dt;

		f_X_i.block<3, 1>(0, 4) = p0 + v0 * dt
			+ 0.5 * R0 * (IMU_Acc_i - ba0) * dt * dt + 0.5 * gravity * dt * dt;
	  }

	} else {
	  f_X_i.block<3, 1>(0, 3) = v0
		  + R0 //* Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
			  * (IMU_Acc_i - ba0) * dt
		  + gravity * dt;

	  f_X_i.block<3, 1>(0, 4) = p0 + v0 * dt
		  + 0.5 * R0 * (IMU_Acc_i - ba0) * dt * dt + 0.5 * gravity * dt * dt;
	}

	count_i = 0;
	count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS_temp[i].Hard_Contact(k)) {
		if (RS_temp[j].Hard_Contact(k)) {
		  f_X_i.block<3, 1>(0, 5 + count) = X_0_bar.block(0, 5 + count_i, 3, 1);
		  count++;
		}
		count_i++;
	  }
	}

	Eigen::MatrixXd X_j_bar, X_j_bar_inverse;
	X_j_bar.resize(5 + fac_info[i].shared_contact, 5 + fac_info[i].shared_contact);
	X_j_bar_inverse.resize(5 + fac_info[i].shared_contact, 5 + fac_info[i].shared_contact);
	X_j_bar.setIdentity();
	X_j_bar_inverse.setIdentity();

	X_j_bar.block<3, 3>(0, 0) = RS_temp[j].Rotation;
	X_j_bar.block<3, 1>(0, 3) = RS_temp[j].Velocity;
	X_j_bar.block<3, 1>(0, 4) = RS_temp[j].Position;

	X_j_bar_inverse.block<3, 3>(0, 0) = RS_temp[j].Rotation.transpose();
	X_j_bar_inverse.block<3, 1>(0, 3) = -RS_temp[j].Rotation.transpose() * RS_temp[j].Velocity;
	X_j_bar_inverse.block<3, 1>(0, 4) = -RS_temp[j].Rotation.transpose() * RS_temp[j].Position;
	count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS_temp[i].Hard_Contact(k) && RS_temp[j].Hard_Contact(k)) {
		X_j_bar.block<3, 1>(0, 5 + count) = RS_temp[j].d.block(3 * k, 0, 3, 1);
		X_j_bar_inverse.block<3, 1>(0, 5 + count) =
			-RS_temp[j].Rotation.transpose() * RS_temp[j].d.block(3 * k, 0, 3, 1);
		count++;
	  }
	}

	Eigen::Matrix<double, -1, 1> Delta;
	Delta.resize(9 + 3 * fac_info[i].shared_contact, 1);
	Delta.setZero();
	Delta = Logm_seK_Vec(f_X_i * X_j_bar_inverse, 2 + fac_info[i].shared_contact);

	residual.block(0, 0, 3, 1) = bg0 - RS_temp[j].Bias_Gyro;
	residual.block(3, 0, 3, 1) = ba0 - RS_temp[j].Bias_Acc;
	residual.block(6, 0, fac_info[i].prop_res_size - 6, 1) = Delta;

  } else {

	if (preintegration_mode_) {
	  if (buffer_size != 0) {
		for (int preintegration_iter = 0; preintegration_iter < fac_info[j].z_buffer_.size(); preintegration_iter++) {
		  integrated_vel += (RS_temp[i].Velocity + integrated_acc) * (dt / fac_info[j].z_buffer_.size());
		  integrated_acc += RS_temp[i].Rotation * Expm_Vec(integrated_gyro) *
			  (fac_info[j].z_buffer_[preintegration_iter].block(3, 0, 3, 1) - RS_temp[i].Bias_Acc)
			  * (dt / fac_info[j].z_buffer_.size());
		  integrated_acc2 += 0.5 * RS_temp[i].Rotation * Expm_Vec(integrated_gyro)
			  * (fac_info[j].z_buffer_[preintegration_iter].block(3, 0, 3, 1) - RS_temp[i].Bias_Acc)
			  * (dt / fac_info[j].z_buffer_.size()) * (dt / fac_info[j].z_buffer_.size());
		  integrated_gyro +=
			  (fac_info[j].z_buffer_[preintegration_iter].block(0, 0, 3, 1) - RS_temp[i].Bias_Gyro)
				  * (dt / fac_info[j].z_buffer_.size());
		}
	  }

	}

	if (preintegration_mode_) {
	  if (buffer_size != 0) {
		f_X_i.block<3, 3>(0, 0) = RS_temp[i].Rotation * Expm_Vec(integrated_gyro);
	  } else {
		f_X_i.block<3, 3>(0, 0) = RS_temp[i].Rotation * Expm_Vec((IMU_Gyro_i - RS_temp[i].Bias_Gyro) * dt);
	  }

	} else {
	  f_X_i.block<3, 3>(0, 0) = RS_temp[i].Rotation * Expm_Vec((IMU_Gyro_i - RS_temp[i].Bias_Gyro) * dt);
	}

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(f_X_i.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
	f_X_i.block<3, 3>(0, 0) = svd.matrixU() * svd.matrixV().transpose();

	if (preintegration_mode_) {
	  if (buffer_size != 0) {
		f_X_i.block<3, 1>(0, 3) = RS_temp[i].Velocity
			+ integrated_acc
			+ gravity * dt;

		f_X_i.block<3, 1>(0, 4) = RS_temp[i].Position + integrated_vel
			+ integrated_acc2 + 0.5 * gravity * dt * dt;
	  } else {
		f_X_i.block<3, 1>(0, 3) = RS_temp[i].Velocity
			+ RS_temp[i].Rotation //* Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
				* (fac_info[i].Z.block(3, 0, 3, 1) - RS_temp[i].Bias_Acc) * dt
			+ gravity * dt;

		f_X_i.block<3, 1>(0, 4) = RS_temp[i].Position + RS_temp[i].Velocity * dt
			+ 0.5 * RS_temp[i].Rotation * (fac_info[i].Z.block(3, 0, 3, 1) - RS_temp[i].Bias_Acc) * dt * dt
			+ 0.5 * gravity * dt * dt;
	  }
	} else {
	  f_X_i.block<3, 1>(0, 3) = RS_temp[i].Velocity
		  + RS_temp[i].Rotation //* Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
			  * (fac_info[i].Z.block(3, 0, 3, 1) - RS_temp[i].Bias_Acc) * dt
		  + gravity * dt;

	  f_X_i.block<3, 1>(0, 4) = RS_temp[i].Position + RS_temp[i].Velocity * dt
		  + 0.5 * RS_temp[i].Rotation * (fac_info[i].Z.block(3, 0, 3, 1) - RS_temp[i].Bias_Acc) * dt * dt
		  + 0.5 * gravity * dt * dt;
	}

	count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS_temp[i].Hard_Contact(k) && RS_temp[j].Hard_Contact(k)) {
		f_X_i.block<3, 1>(0, 5 + count) = RS_temp[i].d.block(3 * k, 0, 3, 1);
		count++;
	  }
	}

	Eigen::MatrixXd X_j_bar, X_j_bar_inverse;
	X_j_bar.resize(5 + fac_info[i].shared_contact, 5 + fac_info[i].shared_contact);
	X_j_bar_inverse.resize(5 + fac_info[i].shared_contact, 5 + fac_info[i].shared_contact);
	X_j_bar.setIdentity();
	X_j_bar_inverse.setIdentity();

	X_j_bar.block<3, 3>(0, 0) = RS_temp[j].Rotation;
	X_j_bar.block<3, 1>(0, 3) = RS_temp[j].Velocity;
	X_j_bar.block<3, 1>(0, 4) = RS_temp[j].Position;

	X_j_bar_inverse.block<3, 3>(0, 0) = RS_temp[j].Rotation.transpose();
	X_j_bar_inverse.block<3, 1>(0, 3) = -RS_temp[j].Rotation.transpose() * RS_temp[j].Velocity;
	X_j_bar_inverse.block<3, 1>(0, 4) = -RS_temp[j].Rotation.transpose() * RS_temp[j].Position;
	count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS_temp[i].Hard_Contact(k) && RS_temp[j].Hard_Contact(k)) {
		X_j_bar.block<3, 1>(0, 5 + count) = RS_temp[j].d.block(3 * k, 0, 3, 1);
		X_j_bar_inverse.block<3, 1>(0, 5 + count) =
			-RS_temp[j].Rotation.transpose() * RS_temp[j].d.block(3 * k, 0, 3, 1);
		count++;
	  }
	}

	Eigen::Matrix<double, -1, 1> Delta;
	Delta.resize(9 + 3 * fac_info[i].shared_contact, 1);
	Delta.setZero();
	Delta = Logm_seK_Vec(f_X_i * X_j_bar_inverse, 2 + fac_info[i].shared_contact);

	residual.block(0, 0, 3, 1) = RS_temp[i].Bias_Gyro - RS_temp[j].Bias_Gyro;
	residual.block(3, 0, 3, 1) = RS_temp[i].Bias_Acc - RS_temp[j].Bias_Acc;
	residual.block(6, 0, fac_info[i].prop_res_size - 6, 1) = Delta;

  }

  if (residual.rows() > 15) {
//std::cout << "prop res "<< residual.block(15,0,3,1).transpose() <<" at "<<i<< std::endl;
  }

if(hasnan)
{
    std::cout << "prop res :\n"<< residual <<"\n at "<<i<< std::endl;
}


  residual = sqrt_info * residual;
  cost += residual.transpose() * residual;








  //Jacobian-------------------------------------------------------------------------

  Eigen::MatrixXd I;
  I.setIdentity(fac_info[i].prop_res_size, fac_info[i].prop_res_size);

  Eigen::MatrixXd IAdt;
  IAdt.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
  IAdt = (I + A * dt);

  Eigen::MatrixXd partial_i;
  partial_i.resize(fac_info[i].prop_res_size, fac_info[i].prop_para0_size + 6);
  partial_i.block(0, 0, fac_info[i].prop_res_size, 6) = (IAdt).block(0, 0, fac_info[i].prop_res_size, 6);
  partial_i.block(0, 6, fac_info[i].prop_res_size, fac_info[i].prop_para0_size) =
	  (IAdt).block(0, 6, fac_info[i].prop_res_size, fac_info[i].prop_res_size - 6) * fac_info[i].Mi;
  if (hasnan){
	std::cout << "prop partial_i :\n" << partial_i << "\n at " << i << std::endl;
}
  partial_i = sqrt_info * partial_i;

  Eigen::MatrixXd partial_j;
  partial_j.resize(fac_info[i].prop_res_size, fac_info[i].prop_para1_size + 6);
  partial_j.block(0, 0, fac_info[i].prop_res_size, 6) = (-I).block(0, 0, fac_info[i].prop_res_size, 6);
  partial_j.block(0, 6, fac_info[i].prop_res_size, fac_info[i].prop_para1_size) =
	  (-I).block(0, 6, fac_info[i].prop_res_size, fac_info[i].prop_res_size - 6) * fac_info[i].Mj;
  if (hasnan){
  std::cout << "prop partial_j :\n"<< partial_j <<"\n at "<<i<< std::endl;
  }
  partial_j = sqrt_info * partial_j;


  //Hessian-------------------------------------------------------------------------------------------

  Hessian.block(RS_temp[i].para_idx, RS_temp[i].para_idx, RS_temp[i].para_size, RS_temp[i].para_size) +=
	  partial_i.transpose() * partial_i;
  Hessian.block(RS_temp[j].para_idx, RS_temp[j].para_idx, RS_temp[j].para_size, RS_temp[j].para_size) +=
	  partial_j.transpose() * partial_j;
  Hessian.block(RS_temp[i].para_idx, RS_temp[j].para_idx, RS_temp[i].para_size, RS_temp[j].para_size) +=
	  partial_i.transpose() * partial_j;
  Hessian.block(RS_temp[j].para_idx, RS_temp[i].para_idx, RS_temp[j].para_size, RS_temp[i].para_size) +=
	  partial_j.transpose() * partial_i;

  //gradient-------------------------------------------------------------------------------------------
  gradient.block(RS_temp[i].para_idx, 0, RS_temp[i].para_size, 1) += partial_i.transpose() * residual;
  gradient.block(RS_temp[j].para_idx, 0, RS_temp[j].para_size, 1) += partial_j.transpose() * residual;


//  std::cout << "prop res :\n"<< residual <<"\n at "<<i<< std::endl;
//
//  std::cout << "prop partial_i :\n"<< partial_i <<"\n at "<<i<< std::endl;
//
//  std::cout << "prop partial_j :\n"<< partial_j <<"\n at "<<i<< std::endl;

  if(hasnan)
  {
	std::cout <<	"temp_matrix1 :\n" << (IAdt).block(0, 0, fac_info[i].prop_res_size, 6) << "\n at " << i << std::endl;
	std::cout <<	"temp_matrix2 :\n" << (IAdt).block(0, 6, fac_info[i].prop_res_size, fac_info[i].prop_res_size - 6) * fac_info[i].Mi << "\n at " << i << std::endl;
	std::cout <<	"temp_matrix3 :\n" << (-I).block(0, 0, fac_info[i].prop_res_size, 6) << "\n at " << i << std::endl;
	std::cout <<	"temp_matrix4 :\n" << (-I).block(0, 6, fac_info[i].prop_res_size, fac_info[i].prop_res_size - 6) * fac_info[i].Mj << "\n at " << i << std::endl;

	std::cout << "sqrt_info :\n" << sqrt_info << "\n at " << i << std::endl;
	std::cout << "Adx_inv_augmented :\n" << Adx_inv_augmented << "\n at " << i << std::endl;

	std::cout << "I:\n" << I << "\n at " << i << std::endl;
	std::cout << "IAdt:\n" << IAdt << "\n at " << i << std::endl;
	std::cout << "fac_info[i].prop_res_size:\n" << fac_info[i].prop_res_size << "\n at " << i << std::endl;
	std::cout << "fac_info[i].prop_para0_size:\n" << fac_info[i].prop_para0_size << "\n at " << i << std::endl;
	std::cout << "fac_info[i].prop_para1_size:\n" << fac_info[i].prop_para1_size << "\n at " << i << std::endl;

	std::cout << "fac_info[i].Mi:\n"<< fac_info[i].Mi <<"\n at "<<i<< std::endl;
	std::cout << "fac_info[i].Mj:\n"<< fac_info[i].Mj <<"\n at "<<i<< std::endl;


	std::cout << "prop res :\n"<< residual <<"\n at "<<i<< std::endl;

	std::cout << "prop partial_i :\n"<< partial_i <<"\n at "<<i<< std::endl;

	std::cout << "prop partial_j :\n"<< partial_j <<"\n at "<<i<< std::endl;
  }


}




//***************************Faster Meas factor********************************

void Inv_Factors::Invariant_Measurement_Factor(int j, int leg_num,
											   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
											   Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost) {

  kinematics_info kin_k = *fac_info[j].leg_info.find(kinematics_info(leg_num));
  // Calculating Covariance--------------------------------------------------------------
  Eigen::Matrix3d sqrt_info;
  sqrt_info = kin_k.meas_primitive_sqrt_info * RS_temp[j].Rotation.transpose();
  //residual------------------------------------------------------------------------------
  Eigen::Vector3d residual;

  if (j == 0 && marginalization_flag) {
	residual = X_0_bar.block(0, 0, 3, 3) * kin_k.fk_kin + X_0_bar.block(0, 4, 3, 1)
		- X_0_bar.block(0, 5 + kin_k.leg_num_in_state, 3, 1);
  } else {
	residual = (RS_temp[j].Rotation * kin_k.fk_kin + RS_temp[j].Position
		- RS_temp[j].d.block(3 * leg_num, 0, 3, 1));// = R*hp +p -d
  }
  Eigen::Vector3d residual_t = residual;
  residual = sqrt_info * residual;



  cost += residual.transpose() * residual;
  //cout<<"now meas residual is "<<residual.transpose()*residual<<endl;

  //Jacobians--------------------------------------------------------------------------------------

  Eigen::MatrixXd partial_xi_i; // only about phi, vel, pos, d
  partial_xi_i.resize(3, fac_info[j].meas_para_size);
  partial_xi_i.setZero();

  //partial_xi_i.block(0,0, 3,3) = -sqrt_info * Hat_so3(residual_t);
  partial_xi_i.block(0, 6, 3, 3) = sqrt_info;
  partial_xi_i.block(0, 9 + 3 * (kin_k.leg_num_in_state), 3, 3) = -sqrt_info;

  //Hessian and Gradient
  gradient.block(RS_temp[j].para_idx + 6, 0, RS_temp[j].para_size - 6, 1) += partial_xi_i.transpose() * residual;
  Hessian.block(RS_temp[j].para_idx + 6, RS_temp[j].para_idx + 6, RS_temp[j].para_size - 6, RS_temp[j].para_size - 6) +=
	  partial_xi_i.transpose() * partial_xi_i;


  if(hasnan)
  {
	std::cout << "meas res :\n"<< residual <<"\n at "<<j<< ", at " << leg_num <<  std::endl;

	std::cout << "meas partial_xi_i:\n"<< partial_xi_i <<"\n at "<<j<< ", at " << leg_num <<  std::endl;
  }

}

void Inv_Factors::Invariant_GPS_Measurement_Factor(int j,
												   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
												   Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost) {
  //residual------------------------------------------------------------------------------
  Eigen::Vector3d residual;

  Eigen::Matrix3d sqrt_information_matrix_GPS;

  sqrt_information_matrix_GPS.setIdentity();

  sqrt_information_matrix_GPS(0, 0) = 1.0 / (fac_info[j].sqrt_GPS(0) + 1e-10);
  sqrt_information_matrix_GPS(1, 1) = 1.0 / (fac_info[j].sqrt_GPS(1) + 1e-10);
  sqrt_information_matrix_GPS(2, 2) = 1.0 / (fac_info[j].sqrt_GPS(2) + 1e-10);

  if (j == 0 && marginalization_flag) {
	residual = X_0_bar.block(0, 4, 3, 1) - fac_info[j].X_GPS;
  } else {
	residual = RS_temp[j].Position - fac_info[j].X_GPS;
  }
  Eigen::Vector3d residual_t = residual;
  residual = sqrt_information_matrix_GPS * residual;

  cost += residual.transpose() * residual;
//  std::cout<<"now meas residual is "<<residual.transpose()*residual<<std::endl;

  //Jacobians--------------------------------------------------------------------------------------

  Eigen::MatrixXd partial_xi_i;
  partial_xi_i.resize(3, fac_info[j].meas_para_size);
  partial_xi_i.setZero();

  partial_xi_i.block(0, 0, 3, 3) = -sqrt_information_matrix_GPS * Hat_so3(fac_info[j].X_GPS);
  partial_xi_i.block(0, 6, 3, 3) = sqrt_information_matrix_GPS;

  //Hessian and Gradient
  gradient.block(RS_temp[j].para_idx + 6, 0, RS_temp[j].para_size - 6, 1) += partial_xi_i.transpose() * residual;
  Hessian.block(RS_temp[j].para_idx + 6, RS_temp[j].para_idx + 6, RS_temp[j].para_size - 6, RS_temp[j].para_size - 6) +=
	  partial_xi_i.transpose() * partial_xi_i;
}

//***************************Faster Meas factor********************************

void Inv_Factors::Long_Term_Stationary_Foot_Factor(int start, int end, int xyz, int leg_num,
												   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
												   Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost) {

  kinematics_info kin_k_start = *fac_info[start].leg_info.find(kinematics_info(leg_num));
  kinematics_info kin_k_end = *fac_info[end].leg_info.find(kinematics_info(leg_num));




  // Calculating Covariance--------------------------------------------------------------
  Eigen::Matrix3d sqrt_info;
  sqrt_info.setIdentity();
  sqrt_info = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Contact/(estimator_common_struct_.dt);



  //residual------------------------------------------------------------------------------
  Eigen::Vector3d residual;
  residual.setZero();

  residual = RS_temp[start].d.block(3 * leg_num, 0, 3, 1) - RS_temp[end].d.block(3 * leg_num, 0, 3, 1);
  residual = sqrt_info * residual;

  if(hasnan)
  {
//	std::cout << "long term foot meas res "<< residual.transpose() <<" at "<<leg_num<< std::endl;
  }

  cost += residual.transpose() * residual;

  //Jacobians--------------------------------------------------------------------------------------

  Eigen::MatrixXd partial_xi_i;
  partial_xi_i.resize(3, fac_info[start].meas_para_size);
  partial_xi_i.setZero();
  partial_xi_i.block(0, 9 + 3 * (kin_k_start.leg_num_in_state), 3, 3) = sqrt_info;

  Eigen::MatrixXd partial_xi_j;
  partial_xi_j.resize(3, fac_info[end].meas_para_size);
  partial_xi_j.setZero();
  partial_xi_j.block(0, 9 + 3 * (kin_k_end.leg_num_in_state), 3, 3) = -sqrt_info;

  //Hessian and Gradient
  gradient.block(RS_temp[start].para_idx + 6, 0, RS_temp[start].para_size - 6, 1) +=
	  partial_xi_i.transpose() * residual;
  Hessian.block(RS_temp[start].para_idx + 6,
				RS_temp[start].para_idx + 6,
				RS_temp[start].para_size - 6,
				RS_temp[start].para_size - 6) += partial_xi_i.transpose() * partial_xi_i;

  gradient.block(RS_temp[end].para_idx + 6, 0, RS_temp[end].para_size - 6, 1) += partial_xi_j.transpose() * residual;
  Hessian.block(RS_temp[end].para_idx + 6,
				RS_temp[end].para_idx + 6,
				RS_temp[end].para_size - 6,
				RS_temp[end].para_size - 6) += partial_xi_j.transpose() * partial_xi_j;

}

void Inv_Factors::Invariant_Prior_RVP_Factor(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
											 Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost) {

  Eigen::Matrix<double, 5, 5> X_hat_current_est;
  X_hat_current_est.setIdentity();

  X_hat_current_est.block<3, 3>(0, 0) = RS_temp[0].Rotation;
  X_hat_current_est.block<3, 1>(0, 3) = RS_temp[0].Velocity;
  X_hat_current_est.block<3, 1>(0, 4) = RS_temp[0].Position;


  Eigen::MatrixXd X_prior_inverse;
  X_prior_inverse.resize(5, 5);
  X_prior_inverse.setIdentity();

  X_prior_inverse.block<3, 3>(0, 0) = X_Prior.block<3,3>(0,0).transpose();
  X_prior_inverse.block<3, 1>(0, 3) = -X_Prior.block<3,3>(0,0).transpose()*X_Prior.block<3,1>(0,3);
  X_prior_inverse.block<3, 1>(0, 4) = -X_Prior.block<3,3>(0,0).transpose()*X_Prior.block<3,1>(0,4);


  //----------------------------------------------------------------

  Eigen::Matrix<double, 9, 1> X_pri_X_hat_inv_retracted;
  X_pri_X_hat_inv_retracted.setZero();
  X_pri_X_hat_inv_retracted = Logm_seK_Vec(X_hat_current_est * X_prior_inverse, 2);

  Eigen::Matrix<double, 9, 9> jacobian_left_inv;
  jacobian_left_inv.setZero();

  jacobian_left_inv = Inv_Left_Jacobian_SEk(X_pri_X_hat_inv_retracted, 2);

  //residual---------------------------------------------------------------------------------------
  Eigen::Matrix<double, 9, 1> residual;
  residual.setZero();
  residual = SQRT_INFO_Prior * (X_pri_X_hat_inv_retracted);

  cost += residual.transpose() * residual;
  //std::cout<<"now prior residual is "<<residual.norm()<<std::endl;

  //Jacobians---------------------------------------------------------------------------------
  Eigen::Matrix<double, 9, 9> partial_xi_0;
  partial_xi_0.setZero();
  partial_xi_0 = SQRT_INFO_Prior;// * jacobian_left_inv;



  //Hessian, gradient---------------------------------------------------------------------------
  Hessian.block(6, 6, 9, 9) += partial_xi_0.transpose() * partial_xi_0;
  gradient.block(6, 0, 9, 1) += partial_xi_0.transpose() * residual;

}

void Inv_Factors::Invariant_Prior_Bias_Factor(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Hessian,
											  Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost) {


  //residual---------------------------------------------------------------------------
  Eigen::Matrix<double, 6, 1> residual;
  residual.setZero();

  residual << RS_temp[0].Bias_Gyro, RS_temp[0].Bias_Acc;
  residual = residual - Bias_Prior;

  residual = SQRT_INFO_Prior_BIAS * (residual);

  cost += residual.transpose() * residual;
  //cout<<"now prior bias residual is "<<esidual.norm()<<endl;

  Eigen::Matrix<double, 6, 6> partial_bias_0;
  partial_bias_0.setIdentity();
  partial_bias_0 = SQRT_INFO_Prior_BIAS * partial_bias_0;

  //Hessian, gradient-----------------------------------------------------------------------------
  Hessian.block(0, 0, 6, 6) += partial_bias_0.transpose() * partial_bias_0;
  gradient.block(0, 0, 6, 1) += partial_bias_0.transpose() * residual;

}

void InvariantSmoother::Call_File(double _dt, const std::string &file_path) {

  frame_count = 0;
  //literally, counting frame number(fulfilment) in the window
  time_count = 0;
  sliding_window_flag = false;


  std::ifstream myfile;
std::cout << "[IS] file... " << file_path << "/sensordata.txt" << std::endl;
  myfile.open(file_path+"/sensordata.txt");

  //myfile.open("/home/rainbow/Desktop/Minicheetah_ws/Z1_JH_Estimator_Minicheetah_Test/third-party/Z1_JH_Estimator/src/Z1_estimation/test_data/" + file_name + "_sensordata.txt");
  //sensordata is sensor measurement 0~5 : IMU gyro/bias, 6~17 : 3 encoder values for 4 legs

  std::string line;
  std::string temp;

  std::string trash;
  for (int k = 0; k < NUM_OF_TRASH_DATA; k++) {
	std::getline(myfile, trash);
  }
  row_index = 0;
  while (std::getline(myfile, line)) { //while there is a line
	column_index = 0;
	for (char i : line) { // for each character in rowstring
	  if (!isblank(i)) { // if it is not blank, do this
		std::string d(1, i); // convert character to string
		temp.append(d); // append the two strings
	  } else {
		SensorData[row_index][column_index] = stod(temp);  // convert string to double
		temp = ""; // reset the capture
		column_index++; // increment b cause we have a new number
	  }
	}

	SensorData[row_index][column_index] = stod(temp);
	if (row_index >= max_time) {
	  break;
	}
	temp = "";
	row_index++; // onto next row

  }
  myfile.close();

  myfile.open(file_path+"/groundtruth.txt");
  //groundtruth is true state value (motion capture)
  //0~2 is body position, 3~6 is quaternion

  row_index = 0; // row index

  //IS_03
  gt_sd =0;//IS_04
  //IS_05
  //IEKF_01
  //IEKF

  std::string line2;

  for (int k = 0; k < NUM_OF_TRASH_DATA; k++) {
	std::getline(myfile, trash);
  }

  while (std::getline(myfile, line2)) { //while there is a line
	column_index = 0;

	for (char i : line2) { // for each character in rowstring
	  if (!isblank(i)) { // if it is not blank, do this
		std::string d(1, i); // convert character to string
		temp.append(d); // append the two strings

	  } else {
		GroundTruth[row_index][column_index] = stod(temp);  // convert string to double


		temp = ""; // reset the capture
		column_index++; // increment b cause we have a new number
	  }
	}

	GroundTruth[row_index][column_index] = stod(temp);
	if (row_index >= max_time) {

	  break;
	}
	temp = "";
	row_index++; // onto next row


  }
  std::cout<<"groundtruth row, col "<<row_index<<","<<column_index<<std::endl;
  myfile.close();

  textfile_flag = true;
}

Eigen::MatrixXd InvariantSmoother::readMatrix(const char *filename) {
  int cols = 0, rows = 0;
  double buff[((WINDOW_SIZE + 1) * 100) * ((WINDOW_SIZE + 1) * 100)];

  // Read numbers from file into buffer.
  std::ifstream infile;
  infile.open(filename);
  while (!infile.eof()) {
	std::string line;
	getline(infile, line);

	int temp_cols = 0;
	std::stringstream stream(line);
	while (!stream.eof())
	  stream >> buff[cols * rows + temp_cols++];

	if (temp_cols == 0)
	  continue;

	if (cols == 0)
	  cols = temp_cols;

	rows++;
  }

  infile.close();

  rows--;

  // Populate matrix with numbers.
  Eigen::MatrixXd result(rows, cols);
  for (int i = 0; i < rows; i++)
	for (int j = 0; j < cols; j++)
	  result(i, j) = buff[cols * i + j];

  return result;
};

void InvariantSmoother::Initialize(double _dt,
								   EstimatorCovariances estimator_covariances,
								   Eigen::Matrix<double, 16, 1> &initial_condition) {

  sliding_window_flag = false;
  marginalization_flag = false;
  frame_count = 0;
  time_count = 0;


//	std::cout << cov_val_setting << std::endl;

  estimator_common_struct_.variable_contact_cov_mode = variable_contact_cov_mode;
  estimator_common_struct_.cov_amplifier = cov_amplifier;
  estimator_common_struct_.gps_covariance_amplifier = gps_covariance_amplifier;
  estimator_common_struct_.slip_rejection_mode = slip_rejection_mode;
  estimator_common_struct_.slip_threshold = slip_threshold;
  estimator_common_struct_.long_term_v_threshold = long_term_v_threshold;
  estimator_common_struct_.long_term_a_threshold = long_term_a_threshold;
	estimator_common_struct_.estimator_covariances_ = estimator_covariances;
  estimator_common_struct_.dt = _dt;
  estimator_common_struct_.Covariance_Reset();


  dt = _dt;
  gravity << 0, 0, -9.81;


  if (textfile_flag) {

	RS[0].Position << GroundTruth[1][12] + initial_condition(0), GroundTruth[1 ][13]
		+ initial_condition(1), GroundTruth[1 + gt_sd][14] + initial_condition(2);
	RS[0].Velocity << GroundTruth[1][9] + initial_condition(7), GroundTruth[1][10]
		+ initial_condition(8), GroundTruth[1][11] + initial_condition(9);

	RS[0].Bias_Gyro << initial_condition(10), initial_condition(11), initial_condition(12);
	RS[0].Bias_Acc << initial_condition(13), initial_condition(14), initial_condition(15);

	RS[0].Rotation << GroundTruth[1][0], GroundTruth[1][3], GroundTruth[1][6]
		, GroundTruth[1][1], GroundTruth[1][4], GroundTruth[1][7]
		, GroundTruth[1][2], GroundTruth[1][5], GroundTruth[1][8];

	// Eigen::Vector3d Euler = Rotation_to_EulerZYX(RS[0].Rotation);
	// Euler(0) = Euler(0) + initial_condition[3];
	// Euler(1) = Euler(1) + initial_condition[4];
	// Euler(2) = Euler(2) + initial_condition[5];
	// RS[0].Rotation = EulerZYX_to_R_bw(Euler);

  } else {
	RS[0].Position = initial_condition.block(0, 0, 3, 1);
	RS[0].Velocity.setZero();
	Eigen::Vector4d initial_quaternion = initial_condition.block(3, 0, 4, 1);

	RS[0].Rotation = Quaternion_to_Rotation_Matrix(initial_quaternion);
	RS[0].Bias_Acc.setZero();
	RS[0].Bias_Gyro.setZero();

	RS[1].Position = initial_condition.block(0, 0, 3, 1);
	RS[1].Velocity.setZero();

	RS[1].Rotation = Quaternion_to_Rotation_Matrix(initial_quaternion);
	RS[1].Bias_Acc.setZero();
	RS[1].Bias_Gyro.setZero();

  }

}

void InvariantSmoother::new_measurement(Eigen::Matrix<double, num_z, 1> Sensor_i, Eigen::Matrix<bool, 4, 1> Contact_i,const MEAS_FORWARD_KINEMATICS & forkin_set) {



  //sensor measurement reading
  if (textfile_flag) {
	int j = time_count + 1;

	//sensor measurement storage
	Estimation_Z.block(num_z * frame_count, 0, num_z, 1)
		<< SensorData[j][0], SensorData[j][1], SensorData[j][2], SensorData[j][3], SensorData[j][4], SensorData[j][5],
		SensorData[j][6], SensorData[j][7], SensorData[j][8], SensorData[j][9], SensorData[j][10], SensorData[j][11],
		SensorData[j][12], SensorData[j][13], SensorData[j][14], SensorData[j][15], SensorData[j][16], SensorData[j][17],

		SensorData[j][18], SensorData[j][19], SensorData[j][20], SensorData[j][21], SensorData[j][22], SensorData[j][23],
		SensorData[j][24], SensorData[j][25], SensorData[j][26], SensorData[j][27], SensorData[j][28], SensorData[j][29];

	RS[frame_count].Contact << GroundTruth[j][19], GroundTruth[j][20], GroundTruth[j][21], GroundTruth[j][22];

  	// std::cout <<"[IS] sensor... \n" << Estimation_Z.block(num_z * frame_count, 0, num_z, 1).transpose() << std::endl;
  	// std::cout <<"[IS] contact... \n" << RS[frame_count].Contact.transpose() << std::endl;
  	// std::cout <<"[IS] cov GYRO... \n" <<  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Gyro.diagonal().transpose() << std::endl;
  	// std::cout <<"[IS] cov ACC... \n" <<  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Acc.diagonal().transpose() << std::endl;
  	// std::cout <<"[IS] cov CONTACT... \n" <<  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Contact.diagonal().transpose() << std::endl;
  	// std::cout <<"[IS] cov ENCODER... \n" <<  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Encoder.diagonal().transpose() << std::endl;
  	// std::cout <<"[IS] cov B G... \n" <<  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Gyro.diagonal().transpose() << std::endl;
  	// std::cout <<"[IS] cov B A... \n" <<  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Acc.diagonal().transpose() << std::endl;
  	// std::cout <<"[IS] cov P O... \n" <<  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Orientation.diagonal().transpose() << std::endl;
  	// std::cout <<"[IS] cov P P... \n" <<  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Position.diagonal().transpose() << std::endl;
  	// std::cout <<"[IS] cov P V... \n" <<  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Velocity.diagonal().transpose() << std::endl;
  } else {

	//sensor measurement storage
	Estimation_Z.block(num_z * frame_count, 0, num_z, 1) << Sensor_i.block(0, 0, 30, 1);
	RS[frame_count].Contact = Contact_i;

  }

  //making state
  if (frame_count == 0) {//Initializing states which require sensor measurement necessarily

	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {

	  RS[0].Slip(k) = false;
	  RS[0].Hard_Contact(k) = RS[0].Contact(k) - RS[0].Slip(k);
	}

	//estimating foot position
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  RS[0].d.block(3 * k, 0, 3, 1) = RS[0].Position + RS[0].Rotation * forkin_set.forkin_position.at(k);
	}

  } else {//At inner timestep larger than 0

	Eigen::Vector3d IMU_Gyro_Previous, IMU_Acc_Previous, IMU_Gyro;
	Eigen::Matrix<double, 12, 1> ENCODER, ENCODERDOT;
	IMU_Gyro_Previous = Estimation_Z.block(num_z * (frame_count - 1), 0, 3, 1);
	IMU_Acc_Previous = Estimation_Z.block(num_z * (frame_count - 1) + 3, 0, 3, 1);
	IMU_Gyro = Estimation_Z.block(num_z * (frame_count), 0, 3, 1);
	ENCODER = Estimation_Z.block(num_z * (frame_count) + 6, 0, 12, 1);
	ENCODERDOT = Estimation_Z.block(num_z * (frame_count) + num_z_imu + num_z_encoder, 0, num_z_encoderdot, 1);

	//Propagating state using IMU
	RS[frame_count].Bias_Gyro << RS[frame_count - 1].Bias_Gyro;
	RS[frame_count].Bias_Acc << RS[frame_count - 1].Bias_Acc;

	RS[frame_count].Rotation =
		RS[frame_count - 1].Rotation * Expm_Vec((IMU_Gyro_Previous - RS[frame_count - 1].Bias_Gyro) * dt);
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(RS[frame_count].Rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
	RS[frame_count].Rotation = svd.matrixU() * svd.matrixV().transpose();

	RS[frame_count].Velocity = RS[frame_count - 1].Velocity
		+ RS[frame_count - 1].Rotation //* Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
			* (IMU_Acc_Previous - RS[frame_count - 1].Bias_Acc) * dt
		+ gravity * dt;
	//
	RS[frame_count].Position = RS[frame_count - 1].Position + RS[frame_count - 1].Velocity * dt
		+ 0.5 * RS[frame_count - 1].Rotation * (IMU_Acc_Previous - RS[frame_count - 1].Bias_Acc) * dt * dt
		+ 0.5 * gravity * dt * dt;

	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {

	  //Estimating foot velocity
	  RS[frame_count].d_v.block(3 * k, 0, 3, 1) = RS[frame_count].Velocity
		  + RS[frame_count].Rotation * forkin_set.forkin_jacobian.at(k)
			  * ENCODERDOT.block<3, 1>(3 * k, 0)
		  + RS[frame_count].Rotation * Hat_so3(IMU_Gyro - RS[frame_count].Bias_Gyro)
			  * forkin_set.forkin_position.at(k);

	  RS[frame_count].Slip(k) = false;

	  //Slip rejection
	  if (slip_rejection_mode && RS[frame_count].Contact(k) &&
		  RS[frame_count].d_v.block(3 * k, 0, 3, 1).norm() > slip_threshold) {
		RS[frame_count].Slip(k) = true;
	  }
	  RS[frame_count].Hard_Contact(k) = RS[frame_count].Contact(k);

//            RS[frame_count].Hard_Contact(k)  = RS[frame_count].Contact(k)-RS[frame_count].Slip(k);

	}


	//estimating foot position
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[frame_count].Hard_Contact(k) && RS[frame_count - 1].Hard_Contact(k)) {
		RS[frame_count].d.block(3 * k, 0, 3, 1) = RS[frame_count - 1].d.block(3 * k, 0, 3, 1);

	  } else {
		RS[frame_count].d.block(3 * k, 0, 3, 1) = RS[frame_count].Position + RS[frame_count].Rotation
			*forkin_set.forkin_position.at(k);
	  }
	}

  }





  //contact number storage
  RS[frame_count].contact_leg_num = 0;
  for (int k = 0; k < 4; k++) {
	if (RS[frame_count].Hard_Contact(k)) {
	  RS[frame_count].contact_leg_num++;
	}
  }


  //state storage
  RS[frame_count].state_size = 6 + 15 + 3 * RS[frame_count].contact_leg_num;
  if (frame_count == 0) {
	RS[frame_count].state_idx = 0;
  } else {
	RS[frame_count].state_idx = RS[frame_count - 1].state_idx + RS[frame_count - 1].state_size;
  }







  //parameter storage
  //para_size might be smaller than it corresponding state_size by 6, since rotation parameter lives in 3d vectorspace, while state info stores rotation using 3D matrix.
  RS[frame_count].para_size = 6 + 9 + 3 * RS[frame_count].contact_leg_num;
  if (frame_count == 0) {
	RS[frame_count].para_idx = 0;
  } else {
	RS[frame_count].para_idx = RS[frame_count - 1].para_idx + RS[frame_count - 1].para_size;
  }

  //prior info storage
  if (frame_count == 0) {
	RS_Pri = RS[0];
  }

  int i = frame_count - 1;
  int j = frame_count;

  if (frame_count > 0) {

	int shared_contact = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[i].Hard_Contact(k) && RS[j].Hard_Contact(k)) {
		shared_contact = shared_contact + 1;
	  }
	}

	fac_info[i].shared_contact = shared_contact;

	int prop_para0_size = 9 + 3 * RS[i].contact_leg_num;
	int prop_para1_size = 9 + 3 * RS[j].contact_leg_num;
	int prop_res_size = 9 + 3 * shared_contact + 6;

	fac_info[i].prop_para0_size = prop_para0_size;
	fac_info[i].prop_para1_size = prop_para1_size;
	fac_info[i].prop_res_size = prop_res_size;

	//Converting original Xi_i(parameter 0) to common Xi size-----------------------------------------

	Eigen::MatrixXd Mi;

	if (RS[i].contact_leg_num == fac_info[i].shared_contact) {
	  Mi.resize(fac_info[i].prop_para0_size, fac_info[i].prop_para0_size);
	  Mi.setIdentity();
	} else {

	  Mi.resize(9 + 3 * fac_info[i].shared_contact, fac_info[i].prop_para0_size);
	  Mi.setZero();
	  Mi.block(0, 0, 9, 9).setIdentity();

	  int omitted_leg_count = 0;
	  int count = 0;
	  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		if (RS[i].Hard_Contact(k)) {
		  count++;
		  if (RS[i].Hard_Contact(k) == RS[j].Hard_Contact(k)) {
			Mi.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Identity();
		  } else {
			omitted_leg_count++;
			Mi.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Zero();
		  }
		}
	  }
	}

	fac_info[i].Mi.resize(fac_info[i].prop_para0_size, fac_info[i].prop_para0_size);
	fac_info[i].Mi = Mi;

	//Converting original Xi_j(parameter 1) to common Xi size-----------------------------------------
	Eigen::MatrixXd Mj;

	if (RS[j].contact_leg_num == fac_info[i].shared_contact) {
	  Mj.resize(fac_info[i].prop_para1_size, fac_info[i].prop_para1_size);
	  Mj.setIdentity();
	} else {

	  Mj.resize(9 + 3 * fac_info[i].shared_contact, fac_info[i].prop_para1_size);
	  Mj.setZero();
	  Mj.block(0, 0, 9, 9).setIdentity();

	  int omitted_leg_count = 0;
	  int count = 0;
	  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		if (RS[j].Hard_Contact(k)) {
		  count++;
		  if (RS[j].Hard_Contact(k) == RS[i].Hard_Contact(k)) {
			Mj.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Identity();

		  } else {
			omitted_leg_count++;
			Mj.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Zero();
		  }
		}
	  }
	}

	fac_info[i].Mj.resizeLike(Mj);
	fac_info[i].Mj = Mj;
	//propagation covariance-------------------------------------------------------

	Eigen::MatrixXd prop_primitive_sqrt_info;
	prop_primitive_sqrt_info.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
	prop_primitive_sqrt_info.setZero();
	prop_primitive_sqrt_info.block<3, 3>(6, 6) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Gyro;
	prop_primitive_sqrt_info.block<3, 3>(9, 9) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Acc;
	prop_primitive_sqrt_info.block<3, 3>(12, 12) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Acc / (dt / sqrt(2));


	//See below for contact covariance

	std::vector<Eigen::Vector3d> contact_cov_array_temp =
		estimator_common_struct_.Variable_Contact_Cov(RS[i].Hard_Contact, RS[i].d_v);
	int count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[i].Hard_Contact(k) && RS[j].Hard_Contact(k)) {

		Eigen::Matrix3d temp_sqrt_contact_info;
		temp_sqrt_contact_info.setZero();
		temp_sqrt_contact_info.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)),
			1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
		prop_primitive_sqrt_info.block<3, 3>(15 + 3 * count, 15 + 3 * count) =temp_sqrt_contact_info;
		count++;

	  }
	}
	prop_primitive_sqrt_info.block<3, 3>(0, 0) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Gyro;
	prop_primitive_sqrt_info.block<3, 3>(3, 3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Acc;
	fac_info[i].prop_primitive_sqrt_info = prop_primitive_sqrt_info;
  }
  //measurement---------------------------------------------------------------
  fac_info[j].meas_para_size = 9 + 3 * RS[j].contact_leg_num;
  fac_info[j].Z = Estimation_Z.block(num_z * j, 0, num_z, 1);
  fac_info[j].leg_info.clear();

  if (preintegration_mode_) {
//	  Eigen::Vector3d integrated_acc, integrated_gyro;
//	  integrated_acc.setZero();
//	  integrated_gyro.setZero();
//
//
//	  for(int preintegration_iter=0; preintegration_iter < z_buffer_.size(); preintegration_iter++)
//	  {
//		integrated_acc += z_buffer_[preintegration_iter].block(3,0,3,1)*(dt/z_buffer_.size());
//		integrated_gyro += z_buffer_[preintegration_iter].block(0,0,3,1)*(dt/z_buffer_.size());
//	  }
//	  fac_info[j].Integrated_Acc = integrated_acc;
//	  fac_info[j].Integrated_Gyro = integrated_gyro;
	fac_info[j].z_buffer_.resize(z_buffer_.size());
	fac_info[j].z_buffer_ = z_buffer_;
  }
  z_buffer_.clear();

  int count = 0;
  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	if (RS[j].Hard_Contact(k)) {
	  kinematics_info kin_k(k);
	  kin_k.leg_num_in_state = count;
	  Eigen::Vector3d ENC = fac_info[j].Z.block(6 + 3 * k, 0, 3, 1);
	  kin_k.fk_kin = forkin_set.forkin_position.at(k);
	  kin_k.meas_primitive_sqrt_info =
		  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Encoder * forkin_set.forkin_jacobian.at(k).inverse();
//	  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Encoder;
	  //std::cout << "Test meas sqrt_info: "<< (kin_k.meas_primitive_sqrt_info.transpose()*kin_k.meas_primitive_sqrt_info* LeggedRobotKinematics::GetJacobian(k,ENC) * estimator_common_struct_.Covariance_Encoder*LeggedRobotKinematics::GetJacobian(k,ENC).transpose() - Eigen::MatrixXd::Identity(3,3)).norm() << std::endl;
	  fac_info[j].leg_info.insert(kin_k);
	  count++;
	}
  }
  for (int p = 0; p <= frame_count; p++) {
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  RS[p].d_v.block(3 * k, 0, 3, 1) = RS[p].Velocity
		  + RS[p].Rotation * forkin_set.forkin_jacobian.at(k)
			  * Estimation_Z.block<3, 1>(num_z * p + num_z_imu + num_z_encoder + 3 * k, 0)
		  + RS[p].Rotation * Hat_so3(Estimation_Z.block<3, 1>(num_z * p, 0) - RS[p].Bias_Gyro)
			  * forkin_set.forkin_position.at(k);
	}
  }
  for (int p = 0; p < frame_count; p++) {
	std::vector<Eigen::Vector3d> contact_cov_array_temp =
		estimator_common_struct_.Variable_Contact_Cov(RS[p].Hard_Contact, RS[p].d_v);
	int count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[p].Hard_Contact(k) && RS[p + 1].Hard_Contact(k)) {

		Eigen::Matrix3d temp_sqrt_contact_info;
		temp_sqrt_contact_info.setZero();
		temp_sqrt_contact_info.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)),
			1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
		fac_info[p].prop_primitive_sqrt_info.block<3, 3>(15 + 3 * count, 15 + 3 * count) =temp_sqrt_contact_info;
		count++;

	  }
	}
  }

}

void InvariantSmoother::new_measurement(Eigen::Matrix<double, num_z, 1> Sensor_i,
										Eigen::Matrix<bool, 4, 1> Contact_i,
										const MEAS_FORWARD_KINEMATICS & forkin_set,
										Eigen::Matrix<double, 3, 1> X_GPS_i,
										Eigen::Matrix<double, 3, 1> sqrt_GPS_i,
										bool GPS_In_i) {



  //sensor measurement reading
  if (textfile_flag) {
	int j = time_count + 1;

	//sensor measurement storage

	for (int k = 0; k < num_z; k++) {
	  Estimation_Z(num_z * frame_count + k) = SensorData[j][k];
	}

	RS[frame_count].Contact << GroundTruth[j][19], GroundTruth[j][20], GroundTruth[j][21], GroundTruth[j][22];

	RS[frame_count].GPS_In = GPS_In_i;


  } else {

	//sensor measurement storage
	Estimation_Z.block(num_z * frame_count, 0, num_z, 1) << Sensor_i.block(0, 0, num_z, 1);
	RS[frame_count].Contact = Contact_i;
	RS[frame_count].GPS_In = GPS_In_i;

  }

//  for(int k=0; k<frame_count;k++)
//  {
//	if(RS[k].GPS_In)
//	  std::cout << "GPS in at frame " << k << std::endl;
//  }

  //making state
  if (frame_count == 0) {//Initializing states which require sensor measurement necessarily

	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {

	  RS[0].Slip(k) = false;
	  RS[0].Hard_Contact(k) = RS[0].Contact(k) - RS[0].Slip(k);
	}


	//estimating foot position
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  RS[0].d.block(3 * k, 0, 3, 1) = RS[0].Position + RS[0].Rotation * forkin_set.forkin_position.at(k);
	}

  } else {//At inner timestep larger than 0

	Eigen::Vector3d IMU_Gyro_Previous, IMU_Acc_Previous, IMU_Gyro;
	Eigen::Matrix<double, 12, 1> ENCODER, ENCODERDOT;
	IMU_Gyro_Previous = Estimation_Z.block(num_z * (frame_count - 1), 0, 3, 1);
	IMU_Acc_Previous = Estimation_Z.block(num_z * (frame_count - 1) + 3, 0, 3, 1);
	IMU_Gyro = Estimation_Z.block(num_z * (frame_count), 0, 3, 1);
	ENCODER = Estimation_Z.block(num_z * (frame_count) + 6, 0, 12, 1);
	ENCODERDOT = Estimation_Z.block(num_z * (frame_count) + num_z_imu + num_z_encoder, 0, num_z_encoderdot, 1);

	//Propagating state using IMU
	RS[frame_count].Bias_Gyro << RS[frame_count - 1].Bias_Gyro;
	RS[frame_count].Bias_Acc << RS[frame_count - 1].Bias_Acc;

	RS[frame_count].Rotation =
		RS[frame_count - 1].Rotation * Expm_Vec((IMU_Gyro_Previous - RS[frame_count - 1].Bias_Gyro) * dt);
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(RS[frame_count].Rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
	RS[frame_count].Rotation = svd.matrixU() * svd.matrixV().transpose();

	RS[frame_count].Velocity = RS[frame_count - 1].Velocity
		+ RS[frame_count - 1]
			.Rotation //* BasicFunctions_Estimator::Left_Jacobian_SO3((IMU_Gyro_Previous - RS[frame_count-1].Bias_Gyro)*dt)
			* (IMU_Acc_Previous - RS[frame_count - 1].Bias_Acc) * dt
		+ gravity * dt;
	//
	RS[frame_count].Position = RS[frame_count - 1].Position + RS[frame_count - 1].Velocity * dt
		+ 0.5 * RS[frame_count - 1].Rotation * (IMU_Acc_Previous - RS[frame_count - 1].Bias_Acc) * dt * dt
		+ 0.5 * gravity * dt * dt;

	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {

	  //Estimating foot velocity
	  RS[frame_count].d_v.block(3 * k, 0, 3, 1) = RS[frame_count].Velocity
		  + RS[frame_count].Rotation *forkin_set.forkin_jacobian.at(k)
			  * ENCODERDOT.block<3, 1>(3 * k, 0)
		  + RS[frame_count].Rotation * Hat_so3(IMU_Gyro - RS[frame_count].Bias_Gyro)
			  * forkin_set.forkin_position.at(k);

	  RS[frame_count].Slip(k) = false;

	  //Slip rejection
	  if (slip_rejection_mode && RS[frame_count].Contact(k) &&
		  RS[frame_count].d_v.block(3 * k, 0, 3, 1).norm() > slip_threshold) {
		RS[frame_count].Slip(k) = true;
	  }
	  RS[frame_count].Hard_Contact(k) = RS[frame_count].Contact(k);

//            RS[frame_count].Hard_Contact(k)  = RS[frame_count].Contact(k)-RS[frame_count].Slip(k);

	}


	//estimating foot position
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[frame_count].Hard_Contact(k) && RS[frame_count - 1].Hard_Contact(k)) {
		RS[frame_count].d.block(3 * k, 0, 3, 1) = RS[frame_count - 1].d.block(3 * k, 0, 3, 1);

	  } else {
		RS[frame_count].d.block(3 * k, 0, 3, 1) = RS[frame_count].Position + RS[frame_count].Rotation
			* forkin_set.forkin_position.at(k);
	  }
	}

  }





  //contact number storage
  RS[frame_count].contact_leg_num = 0;
  for (int k = 0; k < 4; k++) {
	if (RS[frame_count].Hard_Contact(k) == true) {
	  RS[frame_count].contact_leg_num++;
	}
  }


  //state storage
  RS[frame_count].state_size = 6 + 15 + 3 * RS[frame_count].contact_leg_num;
  if (frame_count == 0) {
	RS[frame_count].state_idx = 0;
  } else {
	RS[frame_count].state_idx = RS[frame_count - 1].state_idx + RS[frame_count - 1].state_size;
  }







  //parameter storage
  //para_size might be smaller than it corresponding state_size by 6, since rotation parameter lives in 3d vectorspace, while state info stores rotation using 3D matrix.
  RS[frame_count].para_size = 6 + 9 + 3 * RS[frame_count].contact_leg_num;
  if (frame_count == 0) {
	RS[frame_count].para_idx = 0;
  } else {
	RS[frame_count].para_idx = RS[frame_count - 1].para_idx + RS[frame_count - 1].para_size;
  }

  //prior info storage
  if (frame_count == 0) {
	RS_Pri = RS[0];
  }

  int i = frame_count - 1;
  int j = frame_count;

  if (frame_count > 0) {

	int shared_contact = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[i].Hard_Contact(k) && RS[j].Hard_Contact(k)) {
		shared_contact = shared_contact + 1;
	  }
	}

	fac_info[i].shared_contact = shared_contact;

	int prop_para0_size = 9 + 3 * RS[i].contact_leg_num;
	int prop_para1_size = 9 + 3 * RS[j].contact_leg_num;
	int prop_res_size = 9 + 3 * shared_contact + 6;

	fac_info[i].prop_para0_size = prop_para0_size;
	fac_info[i].prop_para1_size = prop_para1_size;
	fac_info[i].prop_res_size = prop_res_size;

	//Converting original Xi_i(parameter 0) to common Xi size-----------------------------------------

	Eigen::MatrixXd Mi;

	if (RS[i].contact_leg_num == fac_info[i].shared_contact) {
	  Mi.resize(fac_info[i].prop_para0_size, fac_info[i].prop_para0_size);
	  Mi.setIdentity();
	} else {

	  Mi.resize(9 + 3 * fac_info[i].shared_contact, fac_info[i].prop_para0_size);
	  Mi.setZero();
	  Mi.block(0, 0, 9, 9).setIdentity();

	  int omitted_leg_count = 0;
	  int count = 0;
	  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		if (RS[i].Hard_Contact(k)) {
		  count++;
		  if (RS[i].Hard_Contact(k) == RS[j].Hard_Contact(k)) {
			Mi.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Identity();
		  } else {
			omitted_leg_count++;
			Mi.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Zero();
		  }
		}
	  }
	}

	fac_info[i].Mi.resize(fac_info[i].prop_para0_size, fac_info[i].prop_para0_size);
	fac_info[i].Mi = Mi;

	//Converting original Xi_j(parameter 1) to common Xi size-----------------------------------------
	Eigen::MatrixXd Mj;

	if (RS[j].contact_leg_num == fac_info[i].shared_contact) {
	  Mj.resize(fac_info[i].prop_para1_size, fac_info[i].prop_para1_size);
	  Mj.setIdentity();
	} else {

	  Mj.resize(9 + 3 * fac_info[i].shared_contact, fac_info[i].prop_para1_size);
	  Mj.setZero();
	  Mj.block(0, 0, 9, 9).setIdentity();

	  int omitted_leg_count = 0;
	  int count = 0;
	  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
		if (RS[j].Hard_Contact(k)) {
		  count++;
		  if (RS[j].Hard_Contact(k) == RS[i].Hard_Contact(k)) {
			Mj.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Identity();

		  } else {
			omitted_leg_count++;
			Mj.block(6 + 3 * (count - omitted_leg_count), 6 + 3 * count, 3, 3) << Eigen::Matrix3d::Zero();
		  }
		}
	  }
	}
	fac_info[i].Mj = Mj;
	//propagation covariance-------------------------------------------------------

	Eigen::MatrixXd prop_primitive_sqrt_info;
	prop_primitive_sqrt_info.resize(fac_info[i].prop_res_size, fac_info[i].prop_res_size);
	prop_primitive_sqrt_info.setZero();


	prop_primitive_sqrt_info.block(6,6,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Gyro;
	prop_primitive_sqrt_info.block(9,9,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Acc;
	prop_primitive_sqrt_info.block(12,12,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Acc / (dt / sqrt(2));

	//See below for contact covariance
	std::vector<Eigen::Vector3d> contact_cov_array_temp =
		estimator_common_struct_.Variable_Contact_Cov(RS[i].Hard_Contact, RS[i].d_v);
	int count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[i].Hard_Contact(k) && RS[j].Hard_Contact(k)) {

		Eigen::Matrix3d temp_sqrt_contact_info;
		temp_sqrt_contact_info.setZero();
		temp_sqrt_contact_info.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)),
			1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
		prop_primitive_sqrt_info.block(15 + 3 * count,15 + 3 * count,3,3) =temp_sqrt_contact_info;
		count++;

	  }
	}

	prop_primitive_sqrt_info.block(0,0,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Gyro;
	prop_primitive_sqrt_info.block(3,3,3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Acc;
	fac_info[i].prop_primitive_sqrt_info.resizeLike(prop_primitive_sqrt_info);
	fac_info[i].prop_primitive_sqrt_info = prop_primitive_sqrt_info;

	if(prop_primitive_sqrt_info.inverse().hasNaN())
	{
//	  std::cout << "fac_info[i].prop_res_size:\n" << fac_info[i].prop_res_size << std::endl;
//	  std::cout << "prop_primitive_sqrt_info:\n" << prop_primitive_sqrt_info << std::endl;
//	  std::cout << "fac_info[i].prop_primitive_sqrt_info:\n" << fac_info[i].prop_primitive_sqrt_info << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Acc is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Acc << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Gyro is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Gyro << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Slip is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Slip << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Contact is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Contact << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Encoder is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Encoder << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Bias_Gyro is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Gyro << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Bias_Acc is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Acc << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Prior_Orientation is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Orientation << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Prior_Velocity is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Velocity << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Prior_Position is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Position << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Prior_Bias_Gyro is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Gyro << std::endl;
//	  std::cout << "SQRT_INFO_Covariance_Prior_Bias_Acc is " << std::endl << estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Bias_Acc << std::endl;
	}



  }
  //measurement---------------------------------------------------------------
  fac_info[j].meas_para_size = 9 + 3 * RS[j].contact_leg_num;
  fac_info[j].Z = Estimation_Z.block(num_z * j, 0, num_z, 1);
  fac_info[j].leg_info.clear();

  if (preintegration_mode_) {
	fac_info[j].z_buffer_.resize(z_buffer_.size());
	fac_info[j].z_buffer_ = z_buffer_;
	z_buffer_.clear();
  }



  fac_info[j].X_GPS = X_GPS_i;
  fac_info[j].sqrt_GPS =
	  sqrt(estimator_common_struct_.cov_amplifier * estimator_common_struct_.gps_covariance_amplifier)
		  * sqrt_GPS_i;

  int count = 0;
  for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	if (RS[j].Hard_Contact(k)) {
	  kinematics_info kin_k(k);
	  kin_k.leg_num_in_state = count;
	  Eigen::Vector3d ENC = fac_info[j].Z.block(6 + 3 * k, 0, 3, 1);
	  kin_k.fk_kin = forkin_set.forkin_position.at(k);
	  kin_k.meas_primitive_sqrt_info =
		  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Encoder * forkin_set.forkin_jacobian.at(k).inverse();
//	  estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Encoder;
	  //std::cout << "Test meas sqrt_info: "<< (kin_k.meas_primitive_sqrt_info.transpose()*kin_k.meas_primitive_sqrt_info* LeggedRobotKinematics::GetJacobian(ENC,k+1) * estimator_common_struct_.Covariance_Encoder*LeggedRobotKinematics::GetJacobian(ENC,k+1).transpose() - Eigen::MatrixXd::Identity(3,3)).norm() << std::endl;
	  fac_info[j].leg_info.insert(kin_k);
	  count++;
	}
  }
  for (int p = 0; p <= frame_count; p++) {
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  RS[p].d_v.block(3 * k, 0, 3, 1) = RS[p].Velocity
		  + RS[p].Rotation * forkin_set.forkin_jacobian.at(k)
			  * Estimation_Z.block<3, 1>(num_z * p + num_z_imu + num_z_encoder + 3 * k, 0)
		  + RS[p].Rotation * Hat_so3(Estimation_Z.block<3, 1>(num_z * p, 0) - RS[p].Bias_Gyro)
			  *forkin_set.forkin_position.at(k);
	}
  }
  for (int p = 0; p < frame_count; p++) {
	std::vector<Eigen::Vector3d> contact_cov_array_temp =
		estimator_common_struct_.Variable_Contact_Cov(RS[p].Hard_Contact, RS[p].d_v);
	int count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[p].Hard_Contact(k) && RS[p + 1].Hard_Contact(k)) {
		Eigen::Matrix3d temp_sqrt_contact_info;
		temp_sqrt_contact_info.setZero();
		temp_sqrt_contact_info.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)),
			1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
		fac_info[p].prop_primitive_sqrt_info.block<3, 3>(15 + 3 * count, 15 + 3 * count) =temp_sqrt_contact_info;
		count++;
	  }
	}
  }

}

void InvariantSmoother::Optimization_Solve() {



//    if(temp_count == WINDOW_SIZE){
//        Hessian_S.resize(RS[frame_count].para_idx+RS[frame_count].para_size,RS[frame_count].para_idx+RS[frame_count].para_size);
//        Hessian_S.setZero();
//    }

  double cost_before = 0;
  double cost = 0;
  total_backppgn_number = 0;
  iteration_number = 0;

  Eigen::Matrix<double, Eigen::Dynamic, 1> gradient;
  gradient.resize(RS[frame_count].para_idx + RS[frame_count].para_size, 1);
  gradient.setZero();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> hessian;
  hessian.resize(RS[frame_count].para_idx + RS[frame_count].para_size,
				 RS[frame_count].para_idx + RS[frame_count].para_size);
	hessian.setZero();

  delta_Zeta_Xi.resizeLike(gradient);
  delta_Zeta_Xi.setZero();
  perturbation.resizeLike(gradient);
  perturbation.setZero();

  Inv_Factors factor;

  factor.preintegration_mode_ = preintegration_mode_;



  if (marginalization_flag) {

	factor.Marg_Initialize(RS,
						   Estimation_Z.block(0, 0, num_z * (frame_count + 1), 1),
						   Marginalized_H,
						   Marginalized_Hessian,
						   Marginalized_b,
						   Marginalized_Gradient,
						   fac_info,
						   estimator_common_struct_);
	factor.Marg_Update_n_Get_Gradient_Hess_Cost(0, RS, delta_Zeta_Xi, hessian, gradient, cost);







	for (int iter = 1; iter <= Max_Iteration; iter++) {

	  cost_before = cost;


//	  hessian = 0.5*(hessian + hessian.transpose());

//	  hessian.diagonal().array() += 1e-8;

//            perturbation = hessian.llt().solve(-gradient);
//	  perturbation = hessian.partialPivLu().solve(-gradient);
	  perturbation = hessian.colPivHouseholderQr().solve(-gradient);
//            perturbation = hessian.householderQr().solve(-gradient);

	  if(perturbation.hasNaN()) {

		perturbation.setZero();

//				  std::cout << "inside IS has nan!" << std::endl;
//
//		  hasnan = true;
//
//
//		  factor.Marg_Initialize(RS,
//								 Estimation_Z.block(0, 0, num_z * (frame_count + 1), 1),
//								 Marginalized_H,
//								 Marginalized_Hessian,
//								 Marginalized_b,
//								 Marginalized_Gradient,
//								 fac_info,
//								 estimator_common_struct_);
//		  factor.Marg_Update_n_Get_Gradient_Hess_Cost_Debug(0, RS, delta_Zeta_Xi, hessian, gradient, cost);
//
//
//		  hasnan=false;
//		  std::cout << "hessian: " << hessian << std::endl;
//
//
//		  std::cout << "gradient: " <<gradient << std::endl;

	  }

//		if(perturbation.hasNaN())
//		{
//
//		  perturbation.setZero();
//
//		  std::cout << "inside IS has nan!" << std::endl;
//
//		  hasnan = true;
//
//
//		  factor.Marg_Initialize(RS,
//								 Estimation_Z.block(0, 0, num_z * (frame_count + 1), 1),
//								 Marginalized_H,
//								 Marginalized_Hessian,
//								 Marginalized_b,
//								 Marginalized_Gradient,
//								 fac_info,
//								 estimator_common_struct_);
//		  factor.Marg_Update_n_Get_Gradient_Hess_Cost_Debug(0, RS, delta_Zeta_Xi, hessian, gradient, cost);
//
////		  std::cout << "hessian: " << hessian << std::endl;
////
////
////		  std::cout << "gradient: " <<gradient << std::endl;
//
////			break;
//		}
	  if (iter >= 1) {

		Eigen::Matrix<double, -1, 1> delta_Zeta_Xi_temp;
		delta_Zeta_Xi_temp.resizeLike(delta_Zeta_Xi);
		ROBOT_STATES RS_temp[WINDOW_SIZE + 1];
		double cost_temp = 0; //should NOT be 0
		double t = ALPHA; // test
//                double t = 0.5;
		int backpropagate_count = 0;
		//To turn off backpropagation, set num=0

		while (true) {

		  perturbation = t * perturbation;

		  for (int k = 0; k <= WINDOW_SIZE; k++) {
			RS_temp[k] = RS[k];
		  }

		  delta_Zeta_Xi_temp = delta_Zeta_Xi;
		  delta_Zeta_Xi += perturbation;

		  if (Retract_All_flag) {
			retract_manifold(0);
		  } else if (marginalization_flag) {
			retract_manifold(1);
		  } else {
			retract_manifold(0);
		  }

		  factor.Marg_Update_n_Get_Gradient_Hess_Cost(false, RS, delta_Zeta_Xi, hessian, gradient, cost_temp);
		  backpropagate_count++;

//cout<<"IS: cost at iter "<<iter<<" is "<<cost<<" and at backpropagate_count "<<backpropagate_count<<" is "<<cost_temp<<endl;
		  if (Max_backpropagate_num == 1000) {
			break;
		  } else if (backpropagate_count > Max_backpropagate_num) {

			if(delta_Zeta_Xi.hasNaN())
			{
			  delta_Zeta_Xi.setZero();
			}

			for (int k = 0; k <= WINDOW_SIZE; k++) {
			  RS[k] = RS_temp[k];
			}

			//total_backppgn_number++;
			break;
		  } else if (cost_temp > (cost)) {

			delta_Zeta_Xi = delta_Zeta_Xi_temp;
			for (int k = 0; k <= WINDOW_SIZE; k++) {
			  RS[k] = RS_temp[k];
			}

			t = t * backppgn_rate;
			cost_temp = 0;

			if (backpropagate_count == 1) {
			  total_backppgn_number++;
			}
		  } else {
			break;
		  }
		}

	  } else {

		delta_Zeta_Xi += perturbation;

		if (Retract_All_flag) {
		  retract_manifold(0);
		} else if (marginalization_flag) {
		  retract_manifold(1);
		} else {
		  retract_manifold(0);
		}

	  }

	  //update_dv(0);
	  factor.Marg_Update_n_Get_Gradient_Hess_Cost(0, RS, delta_Zeta_Xi, hessian, gradient, cost);

	  if ((iter > 1) && (((sqrt(pow(cost - cost_before, 2)) / cost_before) < Optimization_Epsilon)
		  || (iter == Max_Iteration))) //Max_Itertation) )
	  {
		iteration_number = iter;
		//cout<<"cost at marg is "<<cost<<endl;
		break;
	  } else if (Max_Iteration == 1) {
		iteration_number = iter;
		//cout<<"cost at marg is "<<cost<<endl;
		break;
	  }

	}

  } else {

	factor.Batch_Initialize(RS,
							RS_Pri,
							Estimation_Z.block(0, 0, num_z * (frame_count + 1), 1),
							fac_info,
							estimator_common_struct_);

	factor.Batch_Update_n_Get_Gradient_Hess_Cost(0, RS, hessian, gradient, cost);

	for (int iter = 1; iter <= Max_Iteration; iter++) {

	  cost_before = cost;

//	  hessian.diagonal().array() += 1e-8;
//            delta_Zeta_Xi = hessian.llt().solve(-gradient);
//	  delta_Zeta_Xi = hessian.partialPivLu().solve(-gradient);
	  delta_Zeta_Xi = hessian.colPivHouseholderQr().solve(-gradient);
//            delta_Zeta_Xi = hessian.householderQr().solve(-gradient);

	  if (iter >= 1) {

		Eigen::Matrix<double, -1, 1> delta_Zeta_Xi_temp;
		delta_Zeta_Xi_temp.resizeLike(delta_Zeta_Xi);
		ROBOT_STATES RS_temp[WINDOW_SIZE + 1];
		double cost_temp = 0; //should NOT be 0
		double t = ALPHA; // test
//                double t = 0.5;
		int backpropagate_count = 0;

		while (true) {

		  delta_Zeta_Xi = t * delta_Zeta_Xi;

		  delta_Zeta_Xi_temp = delta_Zeta_Xi;
		  for (int k = 0; k <= WINDOW_SIZE; k++) {
			RS_temp[k] = RS[k];
		  }
		  retract_manifold(0);

		  factor.Batch_Update_n_Get_Gradient_Hess_Cost(0, RS, hessian, gradient, cost_temp);
		  backpropagate_count++;

		  if (Max_backpropagate_num == 1000) {
			break;
		  } else if (backpropagate_count >= Max_backpropagate_num) {

			delta_Zeta_Xi.setZero();
			for (int k = 0; k <= WINDOW_SIZE; k++) {
			  RS[k] = RS_temp[k];
			}
			//total_backppgn_number++;
			break;
		  } else if (cost_temp > (cost)) {

			delta_Zeta_Xi = delta_Zeta_Xi_temp;
			for (int k = 0; k <= WINDOW_SIZE; k++) {
			  RS[k] = RS_temp[k];
			}

			t = t * backppgn_rate;
			cost_temp = 0;

			if (backpropagate_count == 1) {
			  total_backppgn_number++;
			}

		  } else {
			break;
		  }
		}

	  } else {
		retract_manifold(0);

	  }

	  //update_dv(0);
	  factor.Batch_Update_n_Get_Gradient_Hess_Cost(0, RS, hessian, gradient, cost);


//            if((iter>1) && (sqrt(pow(cost-cost_before,2))/cost_before<Optimization_Epsilon || iter == Max_Iteration)) //Max_Itertation) )
	  if ((iter > 1) && (((sqrt(pow(cost - cost_before, 2)) / cost_before) < Optimization_Epsilon)
		  || (iter == Max_Iteration))) //Max_Itertation) )
	  {
		iteration_number = iter;
		//out<<"final cost is "<<cost<<endl;
		break;
	  } else if (Max_Iteration == 1) {
		iteration_number = iter;
		//cout<<"final cost is "<<cost<<endl;
		break;
	  }

	}

  }





  /// Schur Complement Method for Marginalization
  if (frame_count == WINDOW_SIZE) {

	SAVE_BUFFER[idx_cost][time_count] = cost;

	if (marginalization_flag) {
	  factor.Marg_Update_n_Get_Gradient_Hess_Cost(true, RS, delta_Zeta_Xi, hessian, gradient, cost);
	} else {
	  factor.Batch_Update_n_Get_Gradient_Hess_Cost(true, RS, hessian, gradient, cost);
	}


	int m = RS[0].para_size;

	int n=0;

	for(int i=0; i < frame_count; i++)
	{
	  		n += RS[i+1].para_size;
	}

//	Eigen::MatrixXd Hmm = 0.5 * (hessian.block(0, 0, m, m) + hessian.block(0, 0, m, m).transpose());
//	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes_temp(Hmm);
//
//	//ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());
//
//	Eigen::MatrixXd Hmm_inv =
//		saes_temp.eigenvectors()
//			* Eigen::VectorXd((saes_temp.eigenvalues().array() > eps).select(saes_temp.eigenvalues().array().inverse(), 0)).asDiagonal()
//			* saes_temp.eigenvectors().transpose();
//	//printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());
//
//	//Shur
//	Eigen::VectorXd bmm = gradient.segment(0, m);
//	Eigen::MatrixXd Hmr = hessian.block(0, m, m, n);
//	Eigen::MatrixXd Hrm = hessian.block(m, 0, n, m);
//	Eigen::MatrixXd Hrr = hessian.block(m, m, n, n);
//	Eigen::VectorXd brr = gradient.segment(m, n);
//	Eigen::MatrixXd H_marginalized = Hrr - Hrm * Hmm_inv * Hmr;
//	Eigen::VectorXd b_marginalized = brr - Hrm * Hmm_inv * bmm;




//	hessian.diagonal().array() += 1e-8;

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> HMM;
	HMM.resize(RS[0].para_size, RS[0].para_size);
	HMM.setZero();
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> HMR;
	HMR.resize(RS[0].para_size, RS[1].para_size);
	HMR.setZero();
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> HRR;
	HRR.resize(RS[1].para_size, RS[1].para_size);
	HRR.setZero();

	HMM = hessian.block(0, 0, RS[0].para_size, RS[0].para_size);
	HMR = hessian.block(0, RS[0].para_size, RS[0].para_size, RS[1].para_size);
	HRR = hessian.block(RS[0].para_size, RS[0].para_size, RS[1].para_size, RS[1].para_size);

	Marginalized_H.resize(RS[1].para_size, RS[1].para_size);
	Marginalized_Hessian.resize(RS[1].para_size, RS[1].para_size);
	Marginalized_H.setZero();
	Marginalized_Hessian.setZero();
//        Marginalized_H = HRR - HMR.transpose()*HMM.llt().solve(HMR);
//	Marginalized_H = HRR - HMR.transpose()*HMM.partialPivLu().solve(HMR);
		Marginalized_H = HRR - HMR.transpose() * HMM.colPivHouseholderQr().solve(HMR);
//        Marginalized_H = HRR - HMR.transpose()*HMM.householderQr().solve(HMR);
	Marginalized_Hessian = Marginalized_H;

	Marginalized_b.resize(RS[1].para_size, 1);
	Marginalized_Gradient.resize(RS[1].para_size, 1);
	Marginalized_b.setZero();
	Marginalized_Gradient.setZero();

	Marginalized_b = gradient.block(RS[0].para_size, 0, RS[1].para_size, 1)
//                         - HMR.transpose()*HMM.llt().solve(gradient.block(0,0, RS[0].para_size,1));
//	- HMR.transpose()*HMM.partialPivLu().solve(gradient.block(0,0, RS[0].para_size,1));
		- HMR.transpose() * HMM.colPivHouseholderQr().solve(gradient.block(0, 0, RS[0].para_size, 1));
//                - HMR.transpose()*HMM.householderQr().solve(gradient.block(0,0, RS[0].para_size,1));

	Marginalized_Gradient = Marginalized_b;

//	Marginalized_H.diagonal().array() += 1e-8;

	if(!Marginalized_Hessian.allFinite())
	{
	  Marginalized_Hessian = Marginalized_Hessian_bef;
	  Marginalized_H = Marginalized_Hessian;
	}
	if(!Marginalized_Gradient.allFinite())
	{
	  Marginalized_Gradient = Marginalized_Gradient_bef;
	  Marginalized_b = Marginalized_Gradient;
	}

	Marginalized_Hessian_bef = Marginalized_Hessian;
	Marginalized_Gradient_bef = Marginalized_Gradient;


	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Marginalized_H);
	Eigen::VectorXd S = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array(), 0));
	Eigen::VectorXd
		S_inv = Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0));
	Eigen::VectorXd S_sqrt = S.cwiseSqrt();
	Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

	///Notwithstanding its name, Marginalized_H is not H, but J s.t. J'*J=H
	Marginalized_H = S_sqrt.asDiagonal() * saes.eigenvectors().transpose();
	Marginalized_b = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose() * Marginalized_b;

	marginalization_flag = true;

  }

}

void InvariantSmoother::retract_manifold(int start_frame) {



  ////////////////////////////////////////////////////////////////////////////

  for (int i = start_frame; i <= frame_count; i++) {

	Eigen::MatrixXd X_s;
	X_s.resize(5 + RS[i].contact_leg_num, 5 + RS[i].contact_leg_num);
	X_s.setIdentity();

	X_s.block<3, 3>(0, 0) = RS[i].Rotation;
	X_s.block<3, 1>(0, 3) = RS[i].Velocity;
	X_s.block<3, 1>(0, 4) = RS[i].Position;

	int count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[i].Hard_Contact(k)) {
		X_s.block<3, 1>(0, 5 + count) = RS[i].d.block(3 * k, 0, 3, 1);
		count++;
	  }
	}
//                if(time_count==1){
//        std::cout<<"before retract: X4 at frame "<<i<<" is"<<std::endl
//                << X_s<<endl;
//                }

	X_s = Expm_seK_Vec(delta_Zeta_Xi.block(RS[i].para_idx + 6, 0, RS[i].para_size - 6, 1), 2 + RS[i].contact_leg_num)
		* X_s;

	RS[i].Rotation = X_s.block<3, 3>(0, 0);
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(RS[i].Rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
	RS[i].Rotation = svd.matrixU() * svd.matrixV().transpose();

	RS[i].Velocity = X_s.block<3, 1>(0, 3);
	RS[i].Position = X_s.block<3, 1>(0, 4);

	count = 0;
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {
	  if (RS[i].Hard_Contact(k)) {
		RS[i].d.block(3 * k, 0, 3, 1) = X_s.block<3, 1>(0, 5 + count);
		count++;
	  }

	}

	RS[i].Bias_Gyro = RS[i].Bias_Gyro + delta_Zeta_Xi.block(RS[i].para_idx, 0, 3, 1);
	RS[i].Bias_Acc = RS[i].Bias_Acc + delta_Zeta_Xi.block(RS[i].para_idx + 3, 0, 3, 1);

	delta_Zeta_Xi.block(RS[i].para_idx, 0, RS[i].para_size, 1).setZero();

  }

}

void InvariantSmoother::update_dv(int start_frame, const MEAS_FORWARD_KINEMATICS & forkin_set) {

  for (int i = start_frame; i <= frame_count; i++) {
	for (int k = 0; k < estimator_common_struct_.leg_no; k++) {

	  //Estimating foot velocity
	  RS[i].d_v.block(3 * k, 0, 3, 1) = RS[i].Velocity
		  + RS[i].Rotation * forkin_set.forkin_jacobian.at(k)
			  * Estimation_Z.block<3, 1>(num_z * i + num_z_imu + num_z_encoder + 3 * k, 0)
		  + RS[i].Rotation * Hat_so3(Estimation_Z.block<3, 1>(num_z * i, 0) - RS[i].Bias_Gyro)
			  * forkin_set.forkin_position.at(k);

	  if (slip_rejection_mode && RS[i].Contact(k) &&
		  RS[i].d_v.block(3 * k, 0, 3, 1).norm() > slip_threshold) {
		RS[i].Slip(k) = true;
	  } else {
		RS[i].Slip(k) = false;
	  }
	}
  }

}

void InvariantSmoother::send_states(ROBOT_STATES &state_) {
  //Sending Estimated States----------------------------------------------------------------------------
  Eigen::Vector3d W_pos_imu2bd = RS[frame_count].Rotation * estimator_common_struct_.IMU2BD;
  state_.Position = RS[frame_count].Position + W_pos_imu2bd;
  Eigen::Vector3d w_gyro;
  w_gyro = RS[frame_count].Rotation * (Estimation_Z.block(num_z * (frame_count), 0, 3, 1) - RS[frame_count].Bias_Gyro);

  state_.Velocity = RS[frame_count].Velocity + w_gyro.cross(W_pos_imu2bd);
  state_.Bias_Gyro = RS[frame_count].Bias_Gyro;
  state_.Bias_Acc = RS[frame_count].Bias_Acc;
  state_.Rotation = RS[frame_count].Rotation;

  if (frame_count == 0) {
	state_.Hard_Contact = RS[frame_count].Hard_Contact;
	state_.Contact = RS[frame_count].Contact;
	state_.Slip = RS[frame_count].Slip;
	state_.d = RS[frame_count].d;
	state_.d_v = RS[frame_count].d_v;
  } else {
	state_.Hard_Contact = RS[frame_count - 1].Hard_Contact;
	state_.Contact = RS[frame_count - 1].Contact;
	state_.Slip = RS[frame_count - 1].Slip;
	state_.d = RS[frame_count].d;
	state_.d_v = RS[frame_count - 1].d_v;
  }

}

void InvariantSmoother::sliding_window() {

  Estimation_Z.block(0, 0, num_z * WINDOW_SIZE, 1) = Estimation_Z.block(num_z, 0, num_z * WINDOW_SIZE, 1);
  delta_Zeta_Xi.setZero();

  for (int i = 0; i < WINDOW_SIZE; i++) {

	std::swap(RS[i], RS[i + 1]);

	if (i > 0) {
	  RS[i].state_idx = RS[i - 1].state_idx + RS[i - 1].state_size;
	  RS[i].para_idx = RS[i - 1].para_idx + RS[i - 1].para_size;
	} else {
	  RS[1].state_idx = RS[0].state_idx;
	  RS[0].state_idx = 0;
	  RS[1].para_idx = RS[0].para_idx;
	  RS[0].para_idx = 0;
	}

	std::swap(fac_info[i], fac_info[i + 1]);

  }

}

void InvariantSmoother::SaveOneStep(int cnt) {
  if (cnt < SAVEMAXCNT) {
	if (textfile_flag) {
	  //Sending True States----------------------------------------------------------------------------
	  int j = time_count + 1;
		for (int row = 0; row < SAVEMAX; ++row) {
			SAVE_BUFFER[row][cnt] = 0.0;
		}

		Eigen::Matrix<double,9,1> R_true;
		R_true.setZero();
		for (int k =0; k < 9; ++k) {
		R_true(k) = GroundTruth[j][k];
			SAVE_BUFFER[0 + k][cnt] = GroundTruth[j][k];
		}

		Eigen::Matrix<double,3,1> v_true;
		v_true.setZero();
		for (int k = 0; k < 3; ++k) {
		v_true(k) = GroundTruth[j][9+k];
			SAVE_BUFFER[9 + k][cnt] = GroundTruth[j][9 + k]; // velocity
		}

		Eigen::Matrix<double,3,1> p_true;
		p_true.setZero();
		for (int k = 0; k < 3; ++k) {
		p_true(k) = GroundTruth[j][12+k];
			SAVE_BUFFER[12 + k][cnt] = GroundTruth[j][12 + k]; //position
		}
		Eigen::Matrix<double,9,1> R_temp = Eigen::Map<Eigen::Matrix<double,9,1>>(RS[frame_count].Rotation.data(), 9);
		for (int k = 0; k < 9; ++k) {
			SAVE_BUFFER[15 + k][cnt] = R_temp(k);
		}
		for (int k = 0; k < 3; ++k) {
			SAVE_BUFFER[24 + k][cnt] = RS[frame_count].Velocity(k);
		}
		for (int k = 0; k < 3; ++k) {
			SAVE_BUFFER[27 + k][cnt] = RS[frame_count].Position(k);
		}

	} else {

	  ///True state saving
	}


	//Sending Estimated States----------------------------------------------------------------------------




	Eigen::Vector3d temp_eul = Rotation_to_EulerZYX(RS[frame_count].Rotation);
	for (int i = 0; i < 3; i++) {
	  SAVE_BUFFER[idx_ESTIMATED_Position + i][cnt] = RS[frame_count].Position(i);
	  SAVE_BUFFER[idx_ESTIMATED_Velocity + i][cnt] = RS[frame_count].Velocity(i);
	  SAVE_BUFFER[idx_ESTIMATED_Bias_Gyro + i][cnt] = RS[frame_count].Bias_Gyro(i);
	  SAVE_BUFFER[idx_ESTIMATED_Bias_Acc + i][cnt] = RS[frame_count].Bias_Acc(i);
	  SAVE_BUFFER[idx_ESTIMATED_rpy + i][cnt] = temp_eul(i);
	}

	for (int i = 0; i < 3; i++) {
	  for (int j = 0; j < 3; j++) {
		SAVE_BUFFER[idx_ESTIMATED_Rotation + i * 3 + j][cnt] = RS[frame_count].Rotation(i, j);
	  }
	}

	for (int i = 0; i < estimator_common_struct_.leg_no; i++) {

	  for (int k = 0; k < 3; k++) {

		if (cnt >= WINDOW_SIZE) {
		  SAVE_BUFFER[idx_ESTIMATED_dv + 3 * i + k][cnt] = RS[frame_count].d_v(3 * i + k);
		}
	  }
	}

	for (int i = 0; i < estimator_common_struct_.leg_no; i++) {
	  SAVE_BUFFER[idx_ESTIMATED_Contact + i][cnt] = RS[frame_count].Contact(i);
	  SAVE_BUFFER[idx_ESTIMATED_Slip + i][cnt] = RS[frame_count].Slip(i);
	  SAVE_BUFFER[idx_ESTIMATED_Hard_Contact + i][cnt] = RS[frame_count].Hard_Contact(i);

	}

	SAVE_BUFFER[idx_iteration_No][cnt] = iteration_number;
	SAVE_BUFFER[idx_backppgn_No][cnt] = total_backppgn_number;

  } else {
	cnt = SAVEMAXCNT - 1;
	std::cout << "over Max SAVE_cnt!!" << std::endl;
  }
}

void InvariantSmoother::DoSaveAll(const std::string & file_path_) {

  std::cout << "DO SAVE_BUFFER ALL" << std::endl;

  FILE *ffp = nullptr;
  std::string str;
    str = file_path_ + "/Estimation_Result_IS.txt";


  ffp = fopen(str.c_str(), "w");

  for (int i = 0; i < time_count; i++) {
	for (auto &j : SAVE_BUFFER) {

	  fprintf(ffp, "%f\t", j[i]);
	}
	fprintf(ffp, "\n");
	if (i / 10000 == 0) {
	  //printf("saving... %d / %d\n",i+1,time_count);
	}
  }

  fclose(ffp);

  SAVE_cnt = 0;

  printf("%s\n", str.c_str());
  printf("*** SAVE_BUFFER DONE ***\n");
  //    cout<<"!!"<<endl;

}

void InvariantSmoother::SensorDataBuffering(Eigen::Matrix<double, num_z, 1> Sensor_i) {
  z_buffer_.push_back(Sensor_i);
//  std::cout << "z_buffer size: " << z_buffer_.size() << std::endl;
}

void InvariantSmoother::Onestep(Eigen::Matrix<double, num_z, 1> Sensor_i,
								Eigen::Matrix<bool, 4, 1> Contact_i,const MEAS_FORWARD_KINEMATICS & forkin_set,
								ROBOT_STATES &state_) {

  //std::cout<<endl<<"Now step "<<time_count<<" starts!"<<endl;

  clock_t start, finish;
  start = clock();

  new_measurement(Sensor_i, Contact_i, forkin_set);

  if (frame_count > 0) {
	Optimization_Solve();
  }

  if (RS[frame_count].Velocity.norm() > 100 && dt != 0) {
//        cout<<"Diverged!! from time "<<time_count<<endl;
//        dt=0;
  }

  send_states(state_);
 if (textfile_flag) {
	SaveOneStep(time_count);
  }

//    temp_count++;
  frame_count++;
  if (frame_count > WINDOW_SIZE) {
	frame_count = WINDOW_SIZE;
	sliding_window_flag = true;
  }
  if (textfile_flag) {
	time_count++;
  }

  if (sliding_window_flag) {
	sliding_window();
  }

  finish = clock();
  time_per_step = (double)(finish - start) / CLOCKS_PER_SEC;

  if (textfile_flag) {
	SAVE_BUFFER[idx_time_per_step][time_count - 1] = time_per_step;
  }
  //cout<< "Operating time is "<<time_per_step<<", iter no is "<<iteration_number<<endl<<endl;


}

void InvariantSmoother::Onestep(Eigen::Matrix<double, num_z, 1> Sensor_i,
								Eigen::Matrix<bool, 4, 1> Contact_i,
								ROBOT_STATES &state_,
								const MEAS_FORWARD_KINEMATICS & forkin_set,
								Eigen::Matrix<double, 3, 1> X_GPS_i,
								Eigen::Matrix<double, 3, 1> sqrt_GPS_i,
								bool GPS_In_i) {

  //std::cout<<endl<<"Now step "<<time_count<<" starts!"<<endl;



  clock_t start, finish;
  start = clock();

  new_measurement(Sensor_i, Contact_i,forkin_set, X_GPS_i, sqrt_GPS_i, GPS_In_i);

  if (frame_count > 0) {
	Optimization_Solve();
  }

  if (RS[frame_count].Velocity.norm() > 100 && dt != 0) {
//        cout<<"Diverged!! from time "<<time_count<<endl;
//        dt=0;
  }

  send_states(state_);

  if (textfile_flag) {
	SaveOneStep(time_count);
  }

//    temp_count++;
  frame_count++;
  if (frame_count > WINDOW_SIZE) {
	frame_count = WINDOW_SIZE;
	sliding_window_flag = true;
  }
  if (textfile_flag) {
	time_count++;
  }

  if (sliding_window_flag) {
	sliding_window();
  }

  finish = clock();
  time_per_step = (double)(finish - start) / CLOCKS_PER_SEC;

  if (textfile_flag) {
	SAVE_BUFFER[idx_time_per_step][time_count - 1] = time_per_step;
  }
  //cout<< "Operating time is "<<time_per_step<<", iter no is "<<iteration_number<<endl<<endl;


}
