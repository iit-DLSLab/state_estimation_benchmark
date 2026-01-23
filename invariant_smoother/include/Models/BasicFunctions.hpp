// Copyright (c) 2023. Dynamic Robot Control and Design Laboratory , KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
//
// Modified by Junny on 2023.

//
// Created by junny on 7/1/20.
//

#pragma once
#include <cmath>
#include <cassert>
#include <cstring>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iomanip>
const double TOLERANCE = 1e-10;



template <typename T>
static std::string to_string_n_signficant_figures(const T a_value, const int n = 6)
{
  std::ostringstream out;
  out << std::setprecision(n) << a_value;
  return out.str();
}

template <typename T>
static std::string to_string_n_figures_under_0(const T a_value, const int n = 6)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(n) << a_value;
  return out.str();
}

template<typename _Matrix_Type_>
_Matrix_Type_ PseudoInverse(const _Matrix_Type_ &a, double epsilon =
std::numeric_limits<double>::epsilon())
{
  Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
  return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

  template <typename Derived>
    static bool NearZero(const Eigen::MatrixBase<Derived> &vec)
    {
        bool check = (vec.norm() < TOLERANCE);
        return check;
    }
    template <typename T>
    static bool NearZero_value(const T &val)
    {
        bool check = (fabs(val) < TOLERANCE);
        return check;
    }

static void QuatToRotation(const Eigen::Vector4d& quaternion, Eigen::Matrix3d& rotation)
{
  rotation(0) = quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] - quaternion[2] * quaternion[2] - quaternion[3] * quaternion[3];
  rotation(1) = 2 * quaternion[0] * quaternion[3] + 2 * quaternion[1] * quaternion[2];
  rotation(2) = 2 * quaternion[1] * quaternion[3] - 2 * quaternion[0] * quaternion[2];

  rotation(3) = 2 * quaternion[1] * quaternion[2] - 2 * quaternion[0] * quaternion[3];
  rotation(4) = quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] - quaternion[3] * quaternion[3];
  rotation(5) = 2 * quaternion[0] * quaternion[1] + 2 * quaternion[2] * quaternion[3];

  rotation(6) = 2 * quaternion[0] * quaternion[2] + 2 * quaternion[1] * quaternion[3];
  rotation(7) = 2 * quaternion[2] * quaternion[3] - 2 * quaternion[0] * quaternion[1];
  rotation(8) = quaternion[0] * quaternion[0] - quaternion[1] * quaternion[1] - quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3];
}

static Eigen::Vector4d QuatProduct(const Eigen::Vector4d &a,
								   const Eigen::Vector4d &b)
{
  Eigen::Vector4d ab;
  ab(0) = a(0)*b(0) - a(1)*b(1) - a(2)*b(2) - a(3)*b(3);
  ab(1) = a(0)*b(1) + a(1)*b(0) + a(2)*b(3) - a(3)*b(2);
  ab(2) = a(0)*b(2) - a(1)*b(3) + a(2)*b(0) + a(3)*b(1);
  ab(3) = a(0)*b(3) + a(1)*b(2) - a(2)*b(1) + a(3)*b(0);

  return ab;
}

template <typename T>
static Eigen::Matrix<T, 3, 3> Rot_X(const T &theta)
{
  Eigen::Matrix<T, 3, 3> R_matrix; R_matrix << 1.0,0.0,0.0, 0.0,cos(theta),-sin(theta), 0.0,sin(theta),cos(theta);

  return R_matrix;
}

template <typename T>
static Eigen::Matrix<T, 3, 3> Rot_Y(const T &theta)
{
  Eigen::Matrix<T, 3, 3> R_matrix; R_matrix << cos(theta),0.0,sin(theta), 0.0,1.0,0.0, -sin(theta),0.0,cos(theta);

  return R_matrix;
}

template <typename T>
static Eigen::Matrix<T, 3, 3> Rot_Z(const T &theta)
{
  Eigen::Matrix<T, 3, 3> R_matrix; R_matrix << cos(theta),-sin(theta),0.0, sin(theta),cos(theta),0.0, 0.0,0.0,1.0;

  return R_matrix;
}

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Hat_so3(const Eigen::MatrixBase<Derived> &vec)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> hat_matrix;
        hat_matrix << typename Derived::Scalar(0), -vec(2), vec(1),
                vec(2), typename Derived::Scalar(0), -vec(0),
                -vec(1), vec(0), typename Derived::Scalar(0);
        return hat_matrix;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> Vee_so3(const Eigen::MatrixBase<Derived> &so3)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> vee_vec;
        vee_vec << so3(2,1), so3(0,2), so3(1,0);
        return vee_vec;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Expm_so3(const Eigen::MatrixBase<Derived> &so3)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> exponential_map;

        Eigen::Matrix<typename Derived::Scalar, 3, 1> vec_tmp = Vee_so3(so3);
        typename Derived::Scalar norm = vec_tmp.norm();

        if (NearZero(vec_tmp))
        {
            exponential_map.setIdentity();
        }
        else
        {
            exponential_map = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + (sin(norm)/norm)*so3 + ((1.0-cos(norm))/(norm*norm))*so3*so3;
        }


//        exponential_map = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + so3;


        return exponential_map;

    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Logm_so3(const Eigen::MatrixBase<Derived> &R)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> logarithm_so3;
        logarithm_so3 = Hat_so3(Logm_Vec(R));
        return logarithm_so3;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> Logm_Vec(const Eigen::MatrixBase<Derived> &R)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> omega;
        typename Derived::Scalar tr = R.trace();
        typename Derived::Scalar acosinput = (tr - 1.0)/2.0;

        // note switch to base 1

        const typename Derived::Scalar &R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
        const typename Derived::Scalar &R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
        const typename Derived::Scalar &R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);


        // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
        // we do something special
        if ((tr + 1.0) < TOLERANCE) {
            if (std::abs(R33 + 1.0) > 1e-5)
            {
                Eigen::Matrix<typename Derived::Scalar, 3, 1> tmpvec(R13, R23, 1.0 + R33);
                omega = (M_PI / sqrt(2.0 + 2.0 * R33)) * tmpvec;
            }
            else if (std::abs(R22 + 1.0) > 1e-5)
            {
                Eigen::Matrix<typename Derived::Scalar, 3, 1> tmpvec(R12, 1.0 + R22, R32);
                omega = (M_PI / sqrt(2.0 + 2.0 * R22)) * tmpvec;
            }
            else
            {
                // if(std::abs(R.r1_.x()+1.0) > 1e-5)  This is implicit
                Eigen::Matrix<typename Derived::Scalar, 3, 1> tmpvec(1.0 + R11, R21, R31);
                omega = (M_PI / sqrt(2.0 + 2.0 * R11)) * tmpvec;
            }
        } else {
            typename Derived::Scalar magnitude;
            const typename Derived::Scalar tr_3 = tr - 3.0;  // always negative
            if (tr_3 < -1e-7) {
                typename Derived::Scalar theta = acos((tr - 1.0) / 2.0);
                magnitude = theta / (2.0 * sin(theta));
            } else {
                // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
                // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
                magnitude = 0.5 - tr_3 * tr_3 / 12.0;
            }


            Eigen::Matrix<typename Derived::Scalar, 3, 1> tmpvec2(R32 - R23, R13 - R31, R21 - R12);
            omega = magnitude * tmpvec2;
        }

        return omega;
    }


    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Expm_Vec(const Eigen::MatrixBase<Derived> &vec)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> exponential_map;
        exponential_map = Expm_so3(Hat_so3(vec));
        return exponential_map;
    }
   template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Right_Jacobian_SO3(const Eigen::MatrixBase<Derived> &vec)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> jacobian;
        Eigen::Matrix<typename Derived::Scalar, 3, 3> so3;
        so3 = Hat_so3(vec);
        typename Derived::Scalar norm = vec.norm();

        if (NearZero(vec))
        {
            jacobian.setIdentity();
        }
        else
        {
            Eigen::Matrix<typename Derived::Scalar, 3, 1> vec_pow3;
            vec_pow3 << vec(0)*vec(0)*vec(0),vec(1)*vec(1)*vec(1),vec(2)*vec(2)*vec(2);
            jacobian = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - ((1.0-cos(norm))/(norm*norm))*so3 + ((norm-sin(norm))/vec_pow3.norm())*so3*so3;
        }

        return jacobian;
    }




static Eigen::Matrix<double, 3, 3> Inv_Left_Jacobian_SO3(const Eigen::Vector3d &vec)
{
  Eigen::Matrix<double, 3, 3> jacobian;
  Eigen::Matrix<double, 3, 3> so3;


  if (NearZero(vec))
  {
	jacobian.setIdentity();
  }
  else
  {
	so3 = Hat_so3(vec);
	double norm = vec.norm();
	Eigen::Matrix<double, 3, 1> a = vec/norm;
	double norm_div_tan = 0.5*norm/tan(0.5*norm);
	jacobian = norm_div_tan*Eigen::Matrix<double, 3, 3>::Identity() + (1-norm_div_tan)*a*a.transpose() - 0.5*norm*Hat_so3(a);
  }
  return jacobian;
}





template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> Inv_Right_Jacobian_SO3(const Eigen::MatrixBase<Derived> &vec)
{
  Eigen::Matrix<typename Derived::Scalar, 3, 3> jacobian;
  Eigen::Matrix<typename Derived::Scalar, 3, 3> so3;
  so3 = Hat_so3(vec);
  typename Derived::Scalar norm = vec.norm();

  if (NearZero(vec))
  {
	jacobian.setIdentity();
  }
  else
  {
	jacobian = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + 0.5*so3 + ((1.0/(norm*norm))+(1.0+cos(norm))/(2.0*norm*sin(norm)))*so3*so3;
  }
  return jacobian;
}


static Eigen::Matrix<double, 3, 3> Left_Jacobian_SO3(const Eigen::Vector3d &vec)
{

  Eigen::Matrix<double, 3, 3> left_jacobian;

  if (NearZero(vec))
  {
	left_jacobian.setIdentity();
  }
  else
  {
	Eigen::Matrix<double, 3, 3> so3;
	so3 = Hat_so3(vec);
	double norm = vec.norm();
	Eigen::Matrix<double, 3, 1> a = vec/norm;
	double sin_div_norm = sin(norm)/norm;
	Eigen::Matrix<double, 3, 1> vec_pow3;
	vec_pow3 << vec(0)*vec(0)*vec(0),vec(1)*vec(1)*vec(1),vec(2)*vec(2)*vec(2);
	left_jacobian = sin_div_norm*Eigen::Matrix<double, 3, 3>::Identity() - (1.0-sin_div_norm)*a*a.transpose() + ((1-cos(norm))/norm)*Hat_so3(a);

  }

  return left_jacobian;
}




static Eigen::Matrix<double, Eigen::Dynamic, 1> Logm_seK_Vec(const Eigen::MatrixXd &X, const int k)
{
//        Eigen::Matrix<double, 3, 3> R;
//        R = X.block(0,0,3,3);
//        Eigen::Matrix<double, 3, 1> phi;
//        phi = Logm_Vec(R);

  Eigen::Matrix<double, 3, 1> phi;
  phi = Logm_Vec(X.block(0,0,3,3));
  Eigen::Matrix<double, 3, 3> inv_Left_Jacobian;
//        inv_Left_Jacobian = Inv_Left_Jacobian_SO3(phi);
  inv_Left_Jacobian = Inv_Left_Jacobian_SO3(phi);

  Eigen::Matrix<double, Eigen::Dynamic, 1> Xi_seK;
//        Xi_seK.resize();
  Xi_seK.setZero(3+3*k, 1);

  Xi_seK(0) = phi(0);
  Xi_seK(1) = phi(1);
  Xi_seK(2) = phi(2);

//        Xi_seK.block(0,0, 3,1) = phi;

  for (int i=0; i<k; i++){
	Xi_seK.block(3+3*i,0, 3,1) = inv_Left_Jacobian * X.block(0,3 + i, 3,1);

  }

  return Xi_seK;
}


static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Expm_seK_Vec(const Eigen::VectorXd &Xi, const int k)
{
  Eigen::Matrix<double, 3, 1> phi;
  phi = Xi.block(0,0, 3,1);
  Eigen::Matrix<double, 3, 3> R;
  Eigen::Matrix<double, 3, 3> jacobian_L_phi;

  R = Expm_Vec(phi);
  jacobian_L_phi = Left_Jacobian_SO3(phi);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> X_seK;
//        X_seK.resize();
  X_seK.setZero(3+k, 3+k);

  X_seK.block(0,0, 3,3) = R;

  for (int i=0; i<k; i++){
	X_seK.block(0, 3+i, 3,1) = jacobian_L_phi * Xi.block(3+3*i,0, 3,1);
	X_seK(3+i,3+i) = 1;
  }

  return X_seK;
}



static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Left_Jacobian_SEk(const Eigen::VectorXd &Xi, const int k)
{
  //Right_Jacobia_SEk(Xi) = Left_Jacobian_SEk(-Xi)


  Eigen::Matrix<double, 3, 1> phi;
  phi = Xi.block(0,0, 3,1);

  Eigen::Matrix<double, 3, 3> jacobian_L_phi;
  jacobian_L_phi = Left_Jacobian_SO3(phi);
  double norm = phi.norm();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J_left_SEk;
//        J_left_SEk.resize();
  J_left_SEk.setZero(3+3*k, 3+3*k);

  if (NearZero(phi))
  {
	J_left_SEk.setIdentity();

  }else{

	J_left_SEk.block(0,0, 3,3) = jacobian_L_phi;

	for (int i=1; i<=k; i++){
	  J_left_SEk.block(3*i,3*i, 3,3) = jacobian_L_phi;

	  Eigen::Matrix<double, 3, 1> tk;
	  tk = Xi.block(3*i,0, 3,1);

	  J_left_SEk.block(3*i,0, 3,3) = 0.5*Hat_so3(tk)
		  + ( (norm - sin(norm))/pow(norm,3) )*(Hat_so3(phi)*Hat_so3(tk) + Hat_so3(tk)*Hat_so3(phi) + Hat_so3(phi)*Hat_so3(tk)*Hat_so3(phi) )
		  + ( (norm*norm+2*cos(norm)-2)/(2*pow(norm,4)) )*( Hat_so3(phi)*Hat_so3(phi)*Hat_so3(tk) + Hat_so3(tk)*Hat_so3(phi)*Hat_so3(phi) - 3*Hat_so3(phi)*Hat_so3(tk)*Hat_so3(phi) )
		  + ( (2*norm-3*sin(norm)+norm*cos(norm))/(2*pow(norm,5)) )*( Hat_so3(phi)*Hat_so3(tk)*Hat_so3(phi)*Hat_so3(phi) + Hat_so3(phi)*Hat_so3(phi)*Hat_so3(tk)*Hat_so3(phi) );
	}

  }


  return J_left_SEk;
}




static Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Inv_Left_Jacobian_SEk(const Eigen::VectorXd &Xi, const int k)
{
  //Right_Jacobia_SEk(Xi) = Left_Jacobian_SEk(-Xi)


  Eigen::Matrix<double, 3, 1> phi;
  phi = Xi.block(0,0, 3,1);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inv_J_left_SEk;
//        inv_J_left_SEk.resize();
  inv_J_left_SEk.setZero(3+3*k, 3+3*k);






  if (NearZero(phi))
  {
	inv_J_left_SEk.setIdentity();

  }else{


	double norm = phi.norm();
	double sin_norm = sin(norm);
	double cos_norm = cos(norm);
	double norm4 = pow(norm,4);
	Eigen::Matrix<double, 3, 3> inv_jacobian_L_phi;
	inv_jacobian_L_phi = Inv_Left_Jacobian_SO3(phi);

	inv_J_left_SEk.block(0,0, 3,3) = inv_jacobian_L_phi;

	for (int i=1; i<=k; i++){
	  inv_J_left_SEk.block(3*i,3*i, 3,3) = inv_jacobian_L_phi;

	  Eigen::Matrix<double, 3, 1> tk;
	  tk = Xi.block(3*i,0, 3,1);

	  Eigen::Matrix<double, 3, 3> Qk;

	  Eigen::Matrix<double, 3, 3> phi_tk = Hat_so3(phi)*Hat_so3(tk);
	  Eigen::Matrix<double, 3, 3> tk_phi = Hat_so3(tk)*Hat_so3(phi);
	  Eigen::Matrix<double, 3, 3> phi_tk_phi = Hat_so3(phi)*Hat_so3(tk)*Hat_so3(phi);
	  Qk = 0.5*Hat_so3(tk)
		  + ( (norm - sin_norm)/pow(norm,3) )*(phi_tk + tk_phi + phi_tk_phi )
		  + ( (norm*norm+2*cos_norm-2)/(2*pow(norm,4)) )*( Hat_so3(phi)*phi_tk + tk_phi*Hat_so3(phi) - 3*phi_tk_phi )
		  + ( (2*norm-3*sin_norm+norm*cos_norm)/(2*pow(norm,5)) )*( phi_tk_phi*Hat_so3(phi) + Hat_so3(phi)*phi_tk_phi );

	  inv_J_left_SEk.block(3*i,0, 3,3) = -inv_jacobian_L_phi* Qk *inv_jacobian_L_phi;

	}
  }

  return inv_J_left_SEk;
}


template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> Rotation_to_EulerZYX(const Eigen::MatrixBase<Derived> &R)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> euler;

        if (R(2,0)<1.0)
        {

            if(R(2,0)>(-1.0))
            {
                euler<<atan2(R(2,1),R(2,2)),asin(-R(2,0)),atan2(R(1,0),R(0,0));
            }
            else
            {
                euler<<0.0,-M_PI/2.0,-atan2(-R(1,2),R(1,1));
            }

        }
        else
        {
            euler<<0.0,-M_PI/2.0,-atan2(-R(1,2),R(1,1));
        }


        return euler;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> EulerXYZ_to_R_bw(const Eigen::MatrixBase<Derived> &eul)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> R_matrix;


        R_matrix = Rot_X(eul(0))*Rot_Y(eul(1))*Rot_Z(eul(2));

        return R_matrix;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> EulerZYX_to_R_bw(const Eigen::MatrixBase<Derived> &eul)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> R_matrix;


        R_matrix = Rot_Z(eul(2))*Rot_Y(eul(1))*Rot_X(eul(0));

        return R_matrix;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> EulerZYX_to_R_wb(const Eigen::MatrixBase<Derived> &eul)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> R_matrix;

        R_matrix = EulerZYX_to_R_bw(eul);
        R_matrix.transposeInPlace();

        return R_matrix;
    }



    template <typename T>
    static T sign(const T val)
    {
        if(val>T(0.0))
        {
            return T(1.0);
        }
        else if(val<T(0.0))
        {
            return T(-1.0);
        }
        else
        {
            return val;
        }

    }

    template <typename T>
    static T Sqrtp(const T a)
    {
        return (a>=0 ? sqrt(a) : 0);
    }

    template <typename Derived>
    static Eigen::MatrixXd blkdiag(const Eigen::MatrixBase<Derived>& a, int count)
    {
        Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(a.rows() * count, a.cols() * count);
        for (int i = 0; i < count; ++i)
        {
            bdm.block(i * a.rows(), i * a.cols(), a.rows(), a.cols()) = a;
        }

        return bdm;
    }


static double Sqrtp(const double a)
{
  return (a>=0 ? sqrt(a) : 0);
}



static double sign(double val)
{
  if(val>double(0.0))
  {
	return double(1.0);
  }
  else if(val<double(0.0))
  {
	return double(-1.0);
  }
  else
  {
	return val;
  }

}


static Eigen::Matrix<double, 3, 3> EulerZYX_to_R_bw(const Eigen::Vector3d &eul)
{
  Eigen::Matrix<double, 3, 3> R_matrix;


  R_matrix = Rot_Z(eul(2))*Rot_Y(eul(1))*Rot_X(eul(0));

  return R_matrix;
}



static Eigen::Matrix<double, 3, 1> Rotation_to_EulerZYX(const Eigen::Matrix3d &R)
{
  Eigen::Matrix<double, 3, 1> euler;

  if (R(2,0)<1.0)
  {

	if(R(2,0)>(-1.0))
	{
	  euler<<atan2(R(2,1),R(2,2)),asin(-R(2,0)),atan2(R(1,0),R(0,0));
	}
	else
	{
	  euler<<0.0,-M_PI/2.0,-atan2(-R(1,2),R(1,1));
	}

  }
  else
  {
	euler<<0.0,-M_PI/2.0,-atan2(-R(1,2),R(1,1));
  }


  return euler;
}


template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 1> Rotation_Matrix_to_Quaternion(const Eigen::MatrixBase<Derived> &R)
    {
        Eigen::Matrix<typename Derived::Scalar, 4, 1> quat;


        quat(0) = Sqrtp(1.0+R(0,0)+R(1,1)+R(2,2));
        quat(1) = sign(R(2,1)-R(1,2))*Sqrtp(1+R(0,0)-R(1,1)-R(2,2));
        quat(2) = sign(R(0,2)-R(2,0))*Sqrtp(1-R(0,0)+R(1,1)-R(2,2));
        quat(3) = sign(R(1,0)-R(0,1))*Sqrtp(1-R(0,0)-R(1,1)+R(2,2));

        quat = 0.5*quat;
        if((quat(0)*quat(0) + quat(1)*quat(1) + quat(2)*quat(2) + quat(3)*quat(3)) < TOLERANCE)
        {
            quat(0) = 1.0;
        }



        return quat;
    }

    inline static Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg) {
        Eigen::Matrix3d M;
        M << 0, -omg(2), omg(1),
                omg(2), 0, -omg(0),
                -omg(1), omg(0), 0;
        return M;
    }

    inline static Eigen::Matrix3d MatrixExp(const Eigen::Vector3d &w_theta) {
        Eigen::Matrix3d I;
        I << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
        Eigen::Matrix3d R;
        double theta = w_theta.norm();
        Eigen::VectorXd omghat = w_theta / theta;
        Eigen::Matrix3d omgmat;
        if (w_theta.norm() < 1e-6)
            R = I;
        else
            omgmat = VecToso3(omghat);
        R = I + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
        return R;
    }





static Eigen::Matrix<double, 4, 1> Rotation_Matrix_to_Quaternion(const Eigen::Matrix3d &R)
{
  // R -> q(wxyz)
  Eigen::Matrix<double, 4, 1> quat;
  quat(0) = Sqrtp(1.0+R(0,0)+R(1,1)+R(2,2));
  quat(1) = sign(R(2,1)-R(1,2))*Sqrtp(1+R(0,0)-R(1,1)-R(2,2));
  quat(2) = sign(R(0,2)-R(2,0))*Sqrtp(1-R(0,0)+R(1,1)-R(2,2));
  quat(3) = sign(R(1,0)-R(0,1))*Sqrtp(1-R(0,0)-R(1,1)+R(2,2));

  quat = 0.5*quat;
  if((quat(0)*quat(0) + quat(1)*quat(1) + quat(2)*quat(2) + quat(3)*quat(3)) < TOLERANCE)
  {
	quat(0) = 1.0;
  }



  return quat;
}


static Eigen::Matrix<double, 3, 3> Quaternion_to_Rotation_Matrix(const Eigen::Vector4d &q) {
  // q(wxyz) -> R
  Eigen::Matrix<double, 3, 3> R;
  R(0) = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
  R(1) = 2 * q(0) * q(3) + 2 * q(1) * q(2);
  R(2) = 2 * q(1) * q(3) - 2 * q(0) * q(2);

  R(3) = 2 * q(1) * q(2) - 2 * q(0) * q(3);
  R(4) = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
  R(5) = 2 * q(0) * q(1) + 2 * q(2) * q(3);

  R(6) = 2 * q(0) * q(2) + 2 * q(1) * q(3);
  R(7) = 2 * q(2) * q(3) - 2 * q(0) * q(1);
  R(8) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

  return R;
}


