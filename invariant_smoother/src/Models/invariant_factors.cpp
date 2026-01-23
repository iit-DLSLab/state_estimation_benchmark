#include <Models/invariant_factors.hpp>

InvFactors::InvFactors(){}; 	
InvFactors::~InvFactors(){};

void InvFactors::batch_initialize
(
    ROBOT_STATES *rs, const ROBOT_STATES &rs_prior,
    const Eigen::Matrix<double,Eigen::Dynamic,1> &estimation_z,
    factor_info *fac_info,
    const EstimatorCommonStruct &estimator_common_struct
)
{
    gravity << 0.0, 0.0, -9.80665;
    frame_count = estimation_z.rows()/num_z - 1;
    estimator_common_struct_ = estimator_common_struct;
    dt = estimator_common_struct_.dt;

    fac_info_ = fac_info;
    rs_temp_ = rs;

    // prior
    X_prior.resize(5,5);
    X_prior.setIdentity();

    X_prior.block<3,3>(0,0) = rs_prior.Rotation;
    X_prior.block<3,1>(0,3) = rs_prior.Velocity;
    X_prior.block<3,1>(0,4) = rs_prior.Position;

    sqrt_info_prior.setZero();
    sqrt_info_prior.block<3,3>(0,0) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Orientation;
    sqrt_info_prior.block<3,3>(3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Velocity;
    sqrt_info_prior.block<3,3>(6,6) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Prior_Position;

    bias_prior << rs_prior.Bias_Gyro, rs_prior.Bias_Acc;

    sqrt_info_prior_bias.setZero();
    sqrt_info_prior_bias.block<3,3>(0,0) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Gyro;
    sqrt_info_prior_bias.block<3,3>(3,3) = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Bias_Acc;

}

// compute prior factor
void InvFactors::invariant_prior_rvp_factor
(
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
)
{
    Xhat.setIdentity();
    Xhat.block<3,3>(0,0) = rs_temp_[0].Rotation;
    Xhat.block<3,1>(0,3) = rs_temp_[0].Velocity;
    Xhat.block<3,1>(0,4) = rs_temp_[0].Position;

    X_prior_inv.resize(5,5);
    X_prior_inv.setIdentity();
    X_prior_inv.block<3,3>(0,0) =  X_prior.block<3,3>(0,0).transpose();
    X_prior_inv.block<3,1>(0,3) = -X_prior.block<3,3>(0,0).transpose()*X_prior.block<3,1>(0,3);
    X_prior_inv.block<3,1>(0,4) = -X_prior.block<3,3>(0,0).transpose()*X_prior.block<3,1>(0,4);

    X_prior_hat_inv_ret.setZero();
    X_prior_hat_inv_ret = Logm_seK_Vec(Xhat*X_prior_inv,2);

    jac_left_inv.setZero();
    jac_left_inv = Inv_Left_Jacobian_SEk(X_prior_hat_inv_ret,2);

    // residual
    residual_prior.setZero();
    residual_prior = sqrt_info_prior*(X_prior_hat_inv_ret);

    cost += residual_prior.transpose()*residual_prior;

    // jacobian
    partial_xi0.setZero();
    partial_xi0 = sqrt_info_prior;

    // hessian and gradient
    hessian.block(6,6,9,9)  += partial_xi0.transpose()*partial_xi0;
    gradient.block(6,0,9,1) += partial_xi0.transpose()*residual_prior; 
    
} // end invariant_prior_rvp_factor

// compute prior bias factor
void InvFactors::invariant_prior_bias_factor
(
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
)
{
    // residual
    residual_prior_bias.setZero();
    residual_prior_bias << rs_temp_[0].Bias_Gyro, rs_temp_[0].Bias_Acc;
    residual_prior_bias = residual_prior_bias - bias_prior;
    residual_prior_bias = sqrt_info_prior_bias*residual_prior_bias;

    cost += residual_prior_bias.transpose()*residual_prior_bias;

    // jacobian
    partial_bias0.setIdentity();
    partial_bias0 = sqrt_info_prior_bias*partial_bias0;

    // hessian and gradient
    hessian.block(0,0,6,6)  += partial_bias0.transpose()*partial_bias0;
    gradient.block(0,0,6,1) += partial_bias0.transpose()*residual_prior_bias;  

}

// compute propagation factor
void InvFactors::invariant_propagation_factor
(
    int i, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost, 
    bool lidar_in_i, Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i
)
{
    int j = i+1;

    A.resize(fac_info_[i].prop_res_size,fac_info_[i].prop_res_size);
    A.setZero();
    A.block(9,6,3,3) = Hat_so3(gravity);
    A.block(12,6,3,3) = 0.5*Hat_so3(gravity)*dt;
    A.block(12,9,3,3) = Eigen::Matrix3d::Identity();

    A.block(6,0,3,3) = -rs_temp_[i].Rotation;                                   // Rotation
    A.block(9,0,3,3) = -Hat_so3(rs_temp_[i].Velocity)*rs_temp_[i].Rotation;     // Rotation
    A.block(9,3,3,3) = -rs_temp_[i].Rotation;                                   // Rotation
    A.block(12,0,3,3) = -Hat_so3(rs_temp_[i].Position)*rs_temp_[i].Rotation;    // Position
    

    int count = 0;
    int count_i = 0;
    for (int k = 0; k < estimator_common_struct_.leg_no; k++)
    {
        if (rs_temp_[i].Hard_Contact(k))
        {
            if (rs_temp_[j].Hard_Contact(k))
            {
                d = rs_temp_[i].d.block(3*k,0,3,1);
                A.block(15+3*count,0,3,3) = -Hat_so3(d)*rs_temp_[i].Rotation;   // Rotation                
                count++;
            }
            count_i++;            
        }        
    }

    // covariance calculation
    // adjoint of X_bar
    Adj_inv_aug.resize(fac_info_[i].prop_res_size,fac_info_[i].prop_res_size);
    Adj_inv_aug.setZero();
    Adj_inv_aug.block(0,0,6,6).setIdentity();

    for (int k = 0; k < 3+fac_info_[i].shared_contact; k++)
    {
        Adj_inv_aug.block<3,3>(6+3*k,6+3*k) = rs_temp_[i].Rotation.transpose();     // Rotation 
    }
    
    Adj_inv_aug.block<3,3>(9,6) = -rs_temp_[i].Rotation.transpose()*Hat_so3(rs_temp_[i].Velocity);      // Rotation
    Adj_inv_aug.block<3,3>(12,6) = -rs_temp_[i].Rotation.transpose()*Hat_so3(rs_temp_[i].Position);     // Rotation and Position

    count = 0;
    count_i = 0;
    for (int k = 0; k < estimator_common_struct_.leg_no; k++)
    {
        if (rs_temp_[i].Hard_Contact(k))
        {
            if (rs_temp_[j].Hard_Contact(k))
            {
                d_adj = rs_temp_[i].d.block(3*k,0,3,1);
                Adj_inv_aug.block<3,3>(15+3*count,6) = -rs_temp_[i].Rotation.transpose()*Hat_so3(d_adj);    // Rotation                
                count++;
            }
            count_i++;            
        }        
    }

    sqrt_info_prop.resize(fac_info_[i].prop_res_size,fac_info_[i].prop_res_size);
    sqrt_info_prop = fac_info_[i].prop_primitive_sqrt_info;
    sqrt_info_prop = sqrt_info_prop*Adj_inv_aug/dt;

    // residual
    residual_prop.resize(fac_info_[i].prop_res_size,1);

    f_X_i.resize(5+fac_info_[i].shared_contact,5+fac_info_[i].shared_contact);
    f_X_i.setIdentity();

    imu_gyro_i = fac_info_[i].Z.block(0,0,3,1);
    imu_acc_i = fac_info_[i].Z.block(3,0,3,1);

    integrated_acc.setZero(); integrated_acc2.setZero(); integrated_gyro.setZero(); integrated_vel.setZero();
    integ_comp_acc.setZero(); integ_comp_acc2.setZero(); integ_comp_gyro.setZero(); integ_comp_vel.setZero();
    
    buffer_size = fac_info_[i].z_buffer_.size();
    
    if (i==0 && marginalization_flag_)
    {
        R0 = X0_bar.block(0,0,3,3);
        v0 = X0_bar.block(0,3,3,1);
        p0 = X0_bar.block(0,4,3,1);
        bg0 = bias0_bar.block(0,0,3,1);
        ba0 = bias0_bar.block(3,0,3,1);
        
        if (preintegration_mode_)
        {
            if (buffer_size!=0)
            {
                for (int preint_i = 0; preint_i < fac_info_[j].z_buffer_.size(); preint_i++)
                {
                    integrated_vel  += (v0 + integrated_acc)*(dt/fac_info_[j].z_buffer_.size());
                    integrated_acc  += R0*Expm_Vec(integrated_gyro)*(fac_info_[j].z_buffer_[preint_i].block(3,0,3,1)-ba0)*(dt/fac_info_[j].z_buffer_.size()); 
                    integrated_acc2 += 0.5*R0*Expm_Vec(integrated_gyro)*(fac_info_[j].z_buffer_[preint_i].block(3,0,3,1)-ba0)
                                       *(dt/fac_info_[j].z_buffer_.size())*(dt/fac_info_[j].z_buffer_.size());
                    integrated_gyro += (fac_info_[j].z_buffer_[preint_i].block(0,0,3,1)-bg0)*(dt/fac_info_[j].z_buffer_.size());
                }                
            }            
        }  

        integ_comp_acc = (imu_acc_i-ba0)*dt;
        integ_comp_gyro = (imu_gyro_i-bg0)*dt;
        integ_comp_acc2 = 0.5*R0*(imu_acc_i-ba0)*dt*dt;
        integ_comp_vel = v0*dt;

        if (preintegration_mode_)
        {
            if (buffer_size!=0)
            {
                f_X_i.block<3,3>(0,0) = R0*Expm_Vec(integrated_gyro);
            }
            else 
            {
                f_X_i.block<3,3>(0,0) = R0*Expm_Vec((imu_gyro_i-bg0)*dt);
            }            
        }
        else
        {
                f_X_i.block<3,3>(0,0) = R0*Expm_Vec((imu_gyro_i-bg0)*dt);            
        }  

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(f_X_i.block<3,3>(0,0), Eigen::ComputeFullU | Eigen::ComputeFullV);
        f_X_i.block<3,3>(0,0) = svd.matrixU()*svd.matrixV().transpose();

        if (preintegration_mode_)
        {
            if (buffer_size!=0)
            {
                f_X_i.block<3,1>(0,3) = v0 + integrated_acc + gravity*dt;
                f_X_i.block<3,1>(0,4) = p0 + integrated_vel + integrated_acc2 + 0.5*gravity*dt*dt;
            }
            else
            {
                f_X_i.block<3,1>(0,3) = v0 + R0*(imu_acc_i-ba0)*dt + gravity*dt;
                f_X_i.block<3,1>(0,4) = p0 + v0*dt + 0.5*R0*(imu_acc_i-ba0)*dt*dt + 0.5*gravity*dt*dt;
            }          
        }
        else
        {
                f_X_i.block<3,1>(0,3) = v0 + R0*(imu_acc_i-ba0)*dt + gravity*dt;
                f_X_i.block<3,1>(0,4) = p0 + v0*dt + 0.5*R0*(imu_acc_i-ba0)*dt*dt + 0.5*gravity*dt*dt;            
        }

        count = 0;
        count_i = 0;
        for (int k = 0; k < estimator_common_struct_.leg_no; k++)
        {
            if (rs_temp_[i].Hard_Contact(k))
            {
                if (rs_temp_[j].Hard_Contact(k))
                {
                    f_X_i.block<3,1>(0,5+count) = X0_bar.block(0,5+count_i,3,1);
                    count++;
                }
                count_i++;            
            }        
        }

        Xj_bar.resize(5+fac_info_[i].shared_contact,5+fac_info_[i].shared_contact);
        Xj_bar.setIdentity();

        Xj_bar.block<3,1>(0,3) = rs_temp_[j].Velocity;
       
        Xj_bar.block<3,3>(0,0) = rs_temp_[j].Rotation;      // Rotation
        Xj_bar.block<3,1>(0,4) = rs_temp_[j].Position;      // Position        

        Xj_bar_inv.resize(5+fac_info_[i].shared_contact,5+fac_info_[i].shared_contact);
        Xj_bar_inv.setIdentity();


        Xj_bar_inv.block<3,3>(0,0) =  rs_temp_[j].Rotation.transpose();                             // Rotation
        Xj_bar_inv.block<3,1>(0,3) = -rs_temp_[j].Rotation.transpose()*rs_temp_[j].Velocity;        // Rotation
        Xj_bar_inv.block<3,1>(0,4) = -rs_temp_[j].Rotation.transpose()*rs_temp_[j].Position;        // Rotation and Position  


        count = 0;
        for (int k = 0; k < estimator_common_struct_.leg_no; k++)
        {
            if (rs_temp_[i].Hard_Contact(k) && rs_temp_[j].Hard_Contact(k))
            {
                Xj_bar.block<3,1>(0,5+count) = rs_temp_[j].d.block(3*k,0,3,1);
                Xj_bar_inv.block<3,1>(0,5+count) = -rs_temp_[j].Rotation.transpose()*rs_temp_[j].d.block(3*k,0,3,1);    // Rotation
                count++;
            }            
        }

        delta.resize(9+3*fac_info_[i].shared_contact,1);
        delta.setZero();
        delta = Logm_seK_Vec(f_X_i*Xj_bar_inv,2+fac_info_[i].shared_contact);

        residual_prop.block(0,0,3,1) = bg0 - rs_temp_[j].Bias_Gyro;
        residual_prop.block(3,0,3,1) = ba0 - rs_temp_[j].Bias_Acc;
        residual_prop.block(6,0,fac_info_[i].prop_res_size-6,1) = delta;   

    }   // if (i==0 && marginalization_flag_)
    else
    {
        if (preintegration_mode_)
        {
            if (buffer_size!=0)
            {
                for (int preint_iter = 0; preint_iter < fac_info_[j].z_buffer_.size(); preint_iter++)
                {
                    integrated_vel  += (rs_temp_[i].Velocity + integrated_acc)*(dt/fac_info_[j].z_buffer_.size());
                    integrated_acc  += rs_temp_[i].Rotation*Expm_Vec(integrated_gyro)*(fac_info_[j].z_buffer_[preint_iter].block(3,0,3,1)
                                      - rs_temp_[i].Bias_Acc)*(dt/fac_info_[j].z_buffer_.size());
                    integrated_acc2 += 0.5*rs_temp_[i].Rotation*Expm_Vec(integrated_gyro)*(fac_info_[j].z_buffer_[preint_iter].block(3,0,3,1)
                                       - rs_temp_[i].Bias_Acc)*(dt/fac_info_[j].z_buffer_.size())*(dt/fac_info_[j].z_buffer_.size());
                    integrated_gyro += (fac_info_[j].z_buffer_[preint_iter].block(0,0,3,1) - rs_temp_[i].Bias_Gyro)*(dt/fac_info_[j].z_buffer_.size());
                }                
            }            
        }

        if (preintegration_mode_)
        {
            if (buffer_size!=0)
            {
                f_X_i.block<3,3>(0,0) = rs_temp_[i].Rotation*Expm_Vec(integrated_gyro);
            }
            else
            {
                f_X_i.block<3,3>(0,0) = rs_temp_[i].Rotation*Expm_Vec((imu_gyro_i-rs_temp_[i].Bias_Gyro)*dt);
            }            
        }
        else
        {
                f_X_i.block<3,3>(0,0) = rs_temp_[i].Rotation*Expm_Vec((imu_gyro_i-rs_temp_[i].Bias_Gyro)*dt);       // Rotation            
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(f_X_i.block<3,3>(0,0),Eigen::ComputeFullU | Eigen::ComputeFullV);
        f_X_i.block<3,3>(0,0) = svd.matrixU()*svd.matrixV().transpose();

        if (preintegration_mode_)
        {
            if (buffer_size!=0)
            {
                f_X_i.block<3,1>(0,3) = rs_temp_[i].Velocity + integrated_acc + gravity*dt;
                f_X_i.block<3,1>(0,4) = rs_temp_[i].Position + integrated_vel + integrated_acc2 + 0.5*gravity*dt*dt;
            }
            else
            {
                f_X_i.block<3,1>(0,3) = rs_temp_[i].Velocity + rs_temp_[i].Rotation*(fac_info_[i].Z.block(3,0,3,1) - rs_temp_[i].Bias_Acc)*dt + gravity*dt;
                f_X_i.block<3,1>(0,4) = rs_temp_[i].Position + rs_temp_[i].Velocity*dt + 0.5*rs_temp_[i].Rotation*(fac_info_[i].Z.block(3,0,3,1) - rs_temp_[i].Bias_Acc)*dt*dt + 0.5*gravity*dt*dt;
            }            
        }
        else
        {
                f_X_i.block<3,1>(0,3) = rs_temp_[i].Velocity + rs_temp_[i].Rotation*(fac_info_[i].Z.block(3,0,3,1) - rs_temp_[i].Bias_Acc)*dt + gravity*dt;     // Rotation
                f_X_i.block<3,1>(0,4) = rs_temp_[i].Position + rs_temp_[i].Velocity*dt + 0.5*rs_temp_[i].Rotation * (fac_info_[i].Z.block(3,0,3,1) - rs_temp_[i].Bias_Acc)*dt*dt + 0.5*gravity*dt*dt;          
        }

        count = 0;
        for (int k = 0; k < estimator_common_struct_.leg_no; k++)
        {
            if (rs_temp_[i].Hard_Contact(k) && rs_temp_[j].Hard_Contact(k))
            {
                f_X_i.block<3,1>(0,5+count) = rs_temp_[i].d.block(3*k,0,3,1);
                count++;
            }
        }

        Xj_bar.resize(5+fac_info_[i].shared_contact, 5+fac_info_[i].shared_contact);
        Xj_bar.setIdentity();

	    Xj_bar.block<3, 1>(0, 3) = rs_temp_[j].Velocity;

        Xj_bar.block<3, 3>(0, 0) = rs_temp_[j].Rotation;    // Rotation
        Xj_bar.block<3, 1>(0, 4) = rs_temp_[j].Position;    // Position    

        Xj_bar_inv.resize(5+fac_info_[i].shared_contact, 5+fac_info_[i].shared_contact);
        Xj_bar_inv.setIdentity();


        Xj_bar_inv.block<3, 3>(0, 0) =  rs_temp_[j].Rotation.transpose();                           // Rotation
        Xj_bar_inv.block<3, 1>(0, 3) = -rs_temp_[j].Rotation.transpose() * rs_temp_[j].Velocity;    // Rotation
        Xj_bar_inv.block<3, 1>(0, 4) = -rs_temp_[j].Rotation.transpose() * rs_temp_[j].Position;    // Rotation and Position
        

	    count = 0;
	    for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        if (rs_temp_[i].Hard_Contact(k) && rs_temp_[j].Hard_Contact(k)) 
            {
		        Xj_bar.block<3, 1>(0,5+count) = rs_temp_[j].d.block(3*k,0,3,1);
                Xj_bar_inv.block<3, 1>(0,5+count) = -rs_temp_[j].Rotation.transpose()*rs_temp_[j].d.block(3 * k, 0, 3, 1);      // Rotation
		        count++;
	        }
	    }

        delta.resize(9 + 3 * fac_info_[i].shared_contact, 1);
	    delta.setZero();
	    delta = Logm_seK_Vec(f_X_i*Xj_bar_inv,2+fac_info_[i].shared_contact);

        residual_prop.block(0, 0, 3, 1) = rs_temp_[i].Bias_Gyro - rs_temp_[j].Bias_Gyro;
	    residual_prop.block(3, 0, 3, 1) = rs_temp_[i].Bias_Acc - rs_temp_[j].Bias_Acc;
	    residual_prop.block(6, 0, fac_info_[i].prop_res_size - 6, 1) = delta;     
                
    }

    residual_prop = sqrt_info_prop*residual_prop;
    cost += residual_prop.transpose() * residual_prop;

    // jacobian
    I.setIdentity(fac_info_[i].prop_res_size, fac_info_[i].prop_res_size);

    IAdt.resize(fac_info_[i].prop_res_size, fac_info_[i].prop_res_size);
    IAdt = (I + A * dt);
      
    partial_i.resize(fac_info_[i].prop_res_size, fac_info_[i].prop_para0_size + 6);
    partial_i.block(0, 0, fac_info_[i].prop_res_size, 6) = (IAdt).block(0, 0, fac_info_[i].prop_res_size, 6);
    partial_i.block(0, 6, fac_info_[i].prop_res_size, fac_info_[i].prop_para0_size) = (IAdt).block(0, 6, fac_info_[i].prop_res_size, fac_info_[i].prop_res_size - 6) * fac_info_[i].Mi;
    partial_i = sqrt_info_prop*partial_i;

    partial_j.resize(fac_info_[i].prop_res_size, fac_info_[i].prop_para1_size + 6);
    partial_j.block(0, 0, fac_info_[i].prop_res_size, 6) = (-I).block(0, 0, fac_info_[i].prop_res_size, 6);
    partial_j.block(0, 6, fac_info_[i].prop_res_size, fac_info_[i].prop_para1_size) = (-I).block(0, 6, fac_info_[i].prop_res_size, fac_info_[i].prop_res_size - 6) * fac_info_[i].Mj;
    partial_j = sqrt_info_prop * partial_j;

    // hessian
    hessian.block(rs_temp_[i].para_idx, rs_temp_[i].para_idx, rs_temp_[i].para_size, rs_temp_[i].para_size) += partial_i.transpose()*partial_i;
    hessian.block(rs_temp_[j].para_idx, rs_temp_[j].para_idx, rs_temp_[j].para_size, rs_temp_[j].para_size) += partial_j.transpose()*partial_j;
    hessian.block(rs_temp_[i].para_idx, rs_temp_[j].para_idx, rs_temp_[i].para_size, rs_temp_[j].para_size) += partial_i.transpose()*partial_j;
    hessian.block(rs_temp_[j].para_idx, rs_temp_[i].para_idx, rs_temp_[j].para_size, rs_temp_[i].para_size) += partial_j.transpose()*partial_i;

    // gradient
    gradient.block(rs_temp_[i].para_idx, 0, rs_temp_[i].para_size, 1) += partial_i.transpose()*residual_prop;
    gradient.block(rs_temp_[j].para_idx, 0, rs_temp_[j].para_size, 1) += partial_j.transpose()*residual_prop;
    
} // end invariant_propagation_factor




// compute LiDAR propagation factor using only LiDAR rotation
void InvFactors::invariant_lidar_propagation_factor
(
    int i, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost, 
    Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i
)
{
    // R_lidar_i = R_lidar_i.transpose();
    Eigen::Matrix3d R_lid_i = R_lidar_i.transpose();
    Eigen::Vector3d x_lid_i = rs_temp_[i].Rotation*x_lidar_i;
    int j = i + 1;  // Next frame

    // Construct the SE(3) transformation from the LiDAR measurements
    Eigen::Matrix<double, 5, 5> lidar_seK, X_next, X_next_inv, lidar_seK_inv;
    lidar_seK.setIdentity();
    X_next.setIdentity();
    X_next_inv.setIdentity();
    lidar_seK_inv.setIdentity();

    // SE(3) from LiDAR measurement (R_lidar_i and x_lidar_i)
    lidar_seK.block<3,3>(0,0) = R_lidar_i;
    lidar_seK.block<3,1>(0,3) = rs_temp_[i].Velocity;
    lidar_seK.block<3,1>(0,4) = x_lidar_i;

    // SE(3) from current state estimate
    X_next.block<3,3>(0,0) = rs_temp_[j].Rotation;      // Rotation
    X_next.block<3,1>(0,3) = rs_temp_[j].Velocity;      // Velocity
    X_next.block<3,1>(0,4) = rs_temp_[j].Position;      // Position

    // Compute the inverse of X_next for SE(3) operations
    X_next_inv.block<3,3>(0,0) = rs_temp_[j].Rotation.transpose();
    X_next_inv.block<3,1>(0,3) = -rs_temp_[j].Rotation.transpose() * rs_temp_[j].Velocity;
    X_next_inv.block<3,1>(0,4) = -rs_temp_[j].Rotation.transpose() * rs_temp_[j].Position;

    // Inverse of LiDAR SE(3)
    lidar_seK_inv.block<3,3>(0,0) = R_lidar_i.transpose();
    lidar_seK_inv.block<3,1>(0,3) = -R_lidar_i.transpose() * rs_temp_[i].Velocity;
    lidar_seK_inv.block<3,1>(0,4) = -R_lidar_i.transpose() * x_lidar_i;

    // Calculate the relative transformation residual using SE(3)
    Eigen::Matrix<double, 5, 5> relative_transform = lidar_seK * X_next_inv;

    // Check the residual magnitude
    Eigen::Matrix<double, 9, 1> seK_residual = Logm_seK_Vec(relative_transform, 2);  // SE(3) residual
    std::cout << "SEk(3) propagation residual magnitude: " << seK_residual.norm() << std::endl;

    // Apply sqrt information (weighting matrix) for the LiDAR measurements
    Eigen::Matrix<double, 15, 15> sqrt_info_lidar_prop;
    sqrt_info_lidar_prop.setIdentity();
    
    // Adjust weights for LiDAR position and rotation
    double lidar_position_weight_x = 1e-1;    // indoor: 10;  outdoor: 
    double lidar_position_weight_y = 1e-1;    // indoor: 10;  outdoor:  
    double lidar_position_weight_z = 1e-3;    // indoor: 800; outdoor:  
    Eigen::Vector3d lidar_position_weight(lidar_position_weight_x, lidar_position_weight_y, lidar_position_weight_z);

    double lidar_rotation_weight = 1e-2; // indoor: 500; outdoor: 

    sqrt_info_lidar_prop.block<3,3>(6,6).diagonal() *= lidar_rotation_weight;  // Weight for rotation
    sqrt_info_lidar_prop.block<3,3>(12,12).diagonal() = lidar_position_weight;  // Weight for position

    // Residual for cost calculation
    Eigen::Matrix<double, 15, 1> residual_lidar_prop;
    residual_lidar_prop.setZero();
    residual_lidar_prop.block(6,0,9,1) = seK_residual;

    // Compute weighted residual
    residual_lidar_prop = sqrt_info_lidar_prop * residual_lidar_prop;

    // Compute cost
    cost += residual_lidar_prop.transpose() * residual_lidar_prop;

    // Jacobian calculation (partial derivatives)
    Eigen::MatrixXd IAdt = Eigen::MatrixXd::Identity(15, 15);
    Eigen::MatrixXd partial_i = sqrt_info_lidar_prop * IAdt;
    
    // Update Hessian and gradient
    hessian.block(rs_temp_[j].para_idx, rs_temp_[j].para_idx, 15, 15) += partial_i.transpose() * partial_i;
    gradient.block(rs_temp_[j].para_idx, 0, 15, 1) += partial_i.transpose() * residual_lidar_prop;




} // end invariant_lidar_propagation_factor





// compute observation cost
void InvFactors::invariant_measurement_factor
(
    int i, int leg_num, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
)
{
    kinematics_info kin_k = *fac_info_[i].leg_info.find(kinematics_info(leg_num));

    // covariance
    sqrt_info_meas = kin_k.meas_primitive_sqrt_info * rs_temp_[i].Rotation.transpose();
    
    // residual
    if (i == 0 && marginalization_flag_) 
    {
	    residual_meas = X0_bar.block(0, 0, 3, 3) * kin_k.fk_kin + X0_bar.block(0, 4, 3, 1) - X0_bar.block(0, 5 + kin_k.leg_num_in_state, 3, 1);
    } 
    else 
    {
	    residual_meas = (rs_temp_[i].Rotation * kin_k.fk_kin + rs_temp_[i].Position - rs_temp_[i].d.block(3 * leg_num, 0, 3, 1));// = R*hp +p -d
    }
  
    residual_t = residual_meas;
    residual_meas = sqrt_info_meas * residual_meas;

    cost += residual_meas.transpose() * residual_meas;

    // jacobians
    partial_xi_i.resize(3, fac_info_[i].meas_para_size);
    partial_xi_i.setZero();
    partial_xi_i.block(0, 6, 3, 3) = sqrt_info_meas;
    partial_xi_i.block(0, 9 + 3 * (kin_k.leg_num_in_state), 3, 3) = -sqrt_info_meas;

    //Hessian and Gradient
    gradient.block(rs_temp_[i].para_idx + 6, 0, rs_temp_[i].para_size - 6, 1) += partial_xi_i.transpose() * residual_meas;
    hessian.block(rs_temp_[i].para_idx + 6, rs_temp_[i].para_idx + 6, rs_temp_[i].para_size - 6, rs_temp_[i].para_size - 6) += partial_xi_i.transpose() * partial_xi_i;

} // end invariant_measurement_factor

void InvFactors::invariant_gps_measurement_factor
(
    int j, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
) 
{
    // std::cout << "inside gps measurement factor" << std::endl;
    //residual------------------------------------------------------------------------------
    Eigen::Vector3d residual;
    Eigen::Matrix3d sqrt_information_matrix_GPS;

    sqrt_information_matrix_GPS.setIdentity();

    sqrt_information_matrix_GPS(0, 0) = 100; // 1.0 / (fac_info_[j].sqrt_GPS(0) + 1e-10);
    sqrt_information_matrix_GPS(1, 1) = 100; // 1.0 / (fac_info_[j].sqrt_GPS(1) + 1e-10);
    sqrt_information_matrix_GPS(2, 2) = 100; // 15.0; // 1.0 / (fac_info_[j].sqrt_GPS(2) + 1e-10);

    if (j == 0 && marginalization_flag_) {
	    residual = X0_bar.block(0, 4, 3, 1) - fac_info_[j].X_GPS;
    } else {
	    residual = rs_temp_[j].Position - fac_info_[j].X_GPS;
    }
    Eigen::Vector3d residual_t = residual;
    residual = sqrt_information_matrix_GPS * residual;

    cost += residual.transpose() * residual;


    //Jacobians--------------------------------------------------------------------------------------
    Eigen::MatrixXd partial_xi_i;
    partial_xi_i.resize(3, fac_info_[j].meas_para_size);
    partial_xi_i.setZero();

    partial_xi_i.block(0, 0, 3, 3) = -sqrt_information_matrix_GPS * Hat_so3(fac_info_[j].X_GPS);
    partial_xi_i.block(0, 6, 3, 3) = sqrt_information_matrix_GPS;

    //Hessian and Gradient
    gradient.block(rs_temp_[j].para_idx + 6, 0, rs_temp_[j].para_size - 6, 1) += partial_xi_i.transpose() * residual;
    hessian.block(rs_temp_[j].para_idx + 6, rs_temp_[j].para_idx + 6, rs_temp_[j].para_size - 6, rs_temp_[j].para_size - 6) +=
	partial_xi_i.transpose() * partial_xi_i;

} // end invariant_gps_measurement_factor

void InvFactors::invariant_lidar_measurement_factor
(
    int j, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
)
{
    // POSITION AND ROTATION
    // Residual
    Eigen::Matrix3d sqrt_info_lidar_pos = Eigen::Matrix3d::Identity();
    sqrt_info_lidar_pos(0, 0) = 100; // 1.0 / (fac_info_[j].sqrt_LiDAR(0) + 1e-10); // indoor: 1e2
    sqrt_info_lidar_pos(1, 1) = 100; // 1.0 / (fac_info_[j].sqrt_LiDAR(1) + 1e-10); // indoor: 1e2
    sqrt_info_lidar_pos(2, 2) = 100; // 1.0 / (fac_info_[j].sqrt_LiDAR(2) + 1e-10); // indoor: 2*1e2

    Eigen::Vector3d position_residual;
    Eigen::Matrix3d rotation_residual;

    if (j == 0 && marginalization_flag_) {
	    position_residual = X0_bar.block(0, 4, 3, 1) - fac_info_[j].X_LiDAR;
        // rotation_residual = X0_bar.block<3, 3>(0, 0) * fac_info_[j].R_LiDAR; //.transpose();
    } else {
	    position_residual = rs_temp_[j].Position - fac_info_[j].X_LiDAR;
        // rotation_residual = rs_temp_[j].Rotation * fac_info_[j].R_LiDAR; //.transpose();
    }

    std::cout << "SEk(3) meas residual: " << position_residual.norm() << std::endl;

    cost += position_residual.transpose() * position_residual;

    Eigen::Matrix<double, 3, Eigen::Dynamic> partial_xi_i(3, fac_info_[j].meas_para_size);
    // partial_xi_i.resize(3, fac_info_[j].meas_para_size);
    // Eigen::MatrixXd partial_xi_i;
    partial_xi_i.setZero();
    partial_xi_i.block<3, 3>(0, 0) = -sqrt_info_lidar_pos * Hat_so3(fac_info_[j].X_LiDAR);
    partial_xi_i.block<3, 3>(0, 6) = sqrt_info_lidar_pos;
    // partial_xi_i.block<3, 3>(3, 3) = sqrt_info_lidar_rot;

    // partial_xi_i.resize(3, fac_info_[j].meas_para_size);
    // partial_xi_i.setZero();


    //Hessian and Gradient
    gradient.block(rs_temp_[j].para_idx + 6, 0, rs_temp_[j].para_size - 6, 1) += partial_xi_i.transpose() * position_residual;
    hessian.block(rs_temp_[j].para_idx + 6, rs_temp_[j].para_idx + 6, rs_temp_[j].para_size - 6, rs_temp_[j].para_size - 6) += partial_xi_i.transpose() * partial_xi_i;


} // end invariant_lidar_measurement_factor


void InvFactors::marg_initialize
(
    ROBOT_STATES *rs, const Eigen::Matrix<double,Eigen::Dynamic,1> &estimation_z,
    const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> H,
    const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> hessian_marg,
    const Eigen::Matrix<double,Eigen::Dynamic,1> b,
    Eigen::Matrix<double,Eigen::Dynamic,1> gradient_marg,
    factor_info *fac_info, const EstimatorCommonStruct &estimator_common_struct
)
{
    gravity << 0.0, 0.0, -9.80665;
    frame_count = estimation_z.rows()/num_z - 1;
    estimator_common_struct_ = estimator_common_struct;
    dt = estimator_common_struct_.dt;

    marginalization_flag_ = true;

    fac_info_ = fac_info;
    rs_temp_ = rs;

    // prior 
    X_prior.resize(5+rs_temp_[0].contact_leg_num,5+rs_temp_[0].contact_leg_num);
    X_prior.setIdentity();
    X_prior.block<3,3>(0,0) = rs_temp_[0].Rotation;
    X_prior.block<3,1>(0,3) = rs_temp_[0].Velocity;
    X_prior.block<3,1>(0,4) = rs_temp_[0].Position;

    int count=0;
    for (int k = 0; k < estimator_common_struct_.leg_no; k++)
    {
        if (rs_temp_[0].Hard_Contact(k))
        {
            X_prior.block<3,1>(0,5+count) = rs_temp_[0].d.block(3*k,0,3,1);
            count++;
        }        
    }

    // prior bias
    bias_prior << rs_temp_[0].Bias_Gyro, rs_temp_[0].Bias_Acc;

    // marginalization factor
    hessian_marg_factor.resizeLike(hessian_marg);
    hessian_marg_factor.setZero();
    hessian_marg_factor = hessian_marg;
    H_matrix = H; 

    gradient_marg_factor.resizeLike(gradient_marg);
    gradient_marg_factor.setZero();
    gradient_marg_factor = gradient_marg;
    b_vec = b;   

} // end marg_initialize


void InvFactors::long_term_stationary_foot_factor
(
    int start, int end, int xyz, int leg_num,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost
)
{
    kinematics_info kin_k_start = *fac_info_[start].leg_info.find(kinematics_info(leg_num));
    kinematics_info kin_k_end = *fac_info_[end].leg_info.find(kinematics_info(leg_num));

    // covariance
    sqrt_info_long.setIdentity();
    sqrt_info_long = estimator_common_struct_.estimator_covariances_.SQRT_INFO_Covariance_Contact/(estimator_common_struct_.dt);

    // residual
    residual_long.setZero();
    residual_long = rs_temp_[start].d.block(3 * leg_num, 0, 3, 1) - rs_temp_[end].d.block(3 * leg_num, 0, 3, 1);
    residual_long = sqrt_info_long * residual_long;

    cost += residual_long.transpose() * residual_long;

    // jacobians
    partial_xi_i.resize(3, fac_info_[start].meas_para_size);
    partial_xi_i.setZero();
    partial_xi_i.block(0, 9 + 3 * (kin_k_start.leg_num_in_state), 3, 3) = sqrt_info_long;

    partial_xi_j.resize(3, fac_info_[end].meas_para_size);
    partial_xi_j.setZero();
    partial_xi_j.block(0, 9 + 3 * (kin_k_end.leg_num_in_state), 3, 3) = -sqrt_info_long;

    // hessian and gradient
    gradient.block(rs_temp_[start].para_idx + 6, 0, rs_temp_[start].para_size - 6, 1) += partial_xi_i.transpose() * residual_long;
    gradient.block(rs_temp_[end].para_idx + 6, 0, rs_temp_[end].para_size - 6, 1) += partial_xi_j.transpose() * residual_long;

    hessian.block(rs_temp_[start].para_idx + 6, rs_temp_[start].para_idx + 6, rs_temp_[start].para_size - 6, rs_temp_[start].para_size - 6) += partial_xi_i.transpose() * partial_xi_i;
    hessian.block(rs_temp_[end].para_idx + 6, rs_temp_[end].para_idx + 6, rs_temp_[end].para_size - 6, rs_temp_[end].para_size - 6) += partial_xi_j.transpose() * partial_xi_j;

} // end long_term_stationary_foot_factor


void InvFactors::marg_update_nget_grad_hess_cost_debug
(
    bool is_for_marginalization, ROBOT_STATES *rs,
    Eigen::Matrix<double, -1, 1> zeta_xi,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost,
    bool lidar_in_i, Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i
)
{
    hasnan = true;

    if (!is_for_marginalization)
    {
        rs_temp_ = rs;
        if (cost!=0)
        {
            for (int p = 0; p < frame_count; p++)
            {
                contact_cov_temp = estimator_common_struct_.Variable_Contact_Cov(rs_temp_[p].Hard_Contact, rs_temp_[p].d_v);
                int count = 0;
                for (int k = 0; k < estimator_common_struct_.leg_no; k++)
                {
                    if (rs_temp_[p].Hard_Contact(k) && rs_temp_[p + 1].Hard_Contact(k))
                    {
                        sqrt_contact_info_temp.setZero();
                        sqrt_contact_info_temp.diagonal() << 1.0/sqrt(contact_cov_temp[k](0)), 1.0/sqrt(contact_cov_temp[k](1)), 1.0/sqrt(contact_cov_temp[k](2));
                        fac_info_[p].prop_primitive_sqrt_info.block<3,3>(15+3*count,15+3*count) = sqrt_contact_info_temp;
                        count++; 
                    }
                }                
            }
        }
    }

    cost = 0;

    if (is_for_marginalization == false)
    {
        hessian.setZero();
        gradient.setZero();

        bias0_bar.block(0,0,3,1) = rs_temp_[0].Bias_Gyro + zeta_xi.block(0,0,3,1);
        bias0_bar.block(3,0,3,1) = rs_temp_[0].Bias_Acc + zeta_xi.block(3,0,3,1);

        Xi0.resize(9+3*rs_temp_[0].contact_leg_num,1);
        Xi0 = zeta_xi.block(6,0,9+3*rs_temp_[0].contact_leg_num,1);

        X_prior_inv.resize(5+rs_temp_[0].contact_leg_num,5+rs_temp_[0].contact_leg_num);
        X_prior_inv.setIdentity();
	    X_prior_inv.block<3, 3>(0, 0) =  X_prior.block<3,3>(0,0).transpose();
	    X_prior_inv.block<3, 1>(0, 3) = -X_prior.block<3,3>(0,0).transpose()*X_prior.block<3,1>(0,3);
	    X_prior_inv.block<3, 1>(0, 4) = -X_prior.block<3,3>(0,0).transpose()*X_prior.block<3,1>(0,4);

    	X0_bar.resize(5 + rs_temp_[0].contact_leg_num, 5 + rs_temp_[0].contact_leg_num);
	    X0_bar.setIdentity();
	    X0_bar.block<3, 3>(0, 0) = rs_temp_[0].Rotation;
	    X0_bar.block<3, 1>(0, 3) = rs_temp_[0].Velocity;
	    X0_bar.block<3, 1>(0, 4) = rs_temp_[0].Position;

        int count = 0;
        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        if (rs_temp_[0].Hard_Contact(k)) 
            {
		        X0_bar.block<3, 1>(0, 5 + count) = rs_temp_[0].d.block(3 * k, 0, 3, 1);
		        X_prior_inv.block<3,1>(0,5+count) = -X_prior.block<3,3>(0,0).transpose()*X_prior.block<3,1>(0,5+count);
		        count++;
            }
	    }
        X0_bar = Expm_seK_Vec(Xi0, 2 + rs_temp_[0].contact_leg_num) * X0_bar;

        bias_Xbar_vee_inv_ret.resize(6+9+3*rs_temp_[0].contact_leg_num,1);
    	bias_Xbar_vee_inv_ret.setZero();
	    bias_Xbar_vee_inv_ret.block(0, 0, 6, 1) = bias0_bar - bias_prior;
	    bias_Xbar_vee_inv_ret.block(6, 0, 9 + 3 * rs_temp_[0].contact_leg_num, 1) = Logm_seK_Vec(X0_bar * X_prior_inv, 2 + rs_temp_[0].contact_leg_num);

        hessian.block(0, 0, rs_temp_[0].para_size, rs_temp_[0].para_size) = hessian_marg_factor;
	    gradient.block(0, 0, rs_temp_[0].para_size, 1) = hessian_marg_factor * (bias_Xbar_vee_inv_ret) + gradient_marg_factor;

        residual_marg_debug = hessian_marg_factor*gradient_marg_factor + bias_Xbar_vee_inv_ret;

        temp_cost = (b_vec + H_matrix * bias_Xbar_vee_inv_ret).norm();

        cost += temp_cost * temp_cost;

        {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(hessian);
	    cond = svd.singularValues()(0)/svd.singularValues()(svd.singularValues().size()-1);
        }

        for (int i = 0; i < frame_count; i++) 
        {
            Eigen::Vector3d p_lid;
            Eigen::Matrix3d R_lid;
	        invariant_propagation_factor(i, hessian, gradient, cost, lidar_in_i, x_lidar_i, R_lidar_i);
            // if (lidar_in_i)
            // {
            //     std::cout << "LiDAR in" << std::endl;
            //     invariant_lidar_propagation_factor(i, hessian, gradient, cost, x_lidar_i, R_lidar_i);
            // }
            

		    Eigen::JacobiSVD<Eigen::MatrixXd> svd(hessian);
		    cond = svd.singularValues()(0)/svd.singularValues()(svd.singularValues().size()-1);
            
	    }

        for (int i = 0; i <= frame_count; i++) 
        {
	        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
            {
		        if (rs_temp_[i].Hard_Contact(k)) 
                {
		            invariant_measurement_factor(i, k, hessian, gradient, cost);  
                    {
			        Eigen::JacobiSVD<Eigen::MatrixXd> svd(hessian);
			        cond = svd.singularValues()(0)/svd.singularValues()(svd.singularValues().size()-1); 
                    }
		        }
	        }

            if (rs_temp_[i].LiDAR_In) 
            {
		        invariant_lidar_measurement_factor(i, hessian, gradient, cost);
	        }

            if (rs_temp_[i].GPS_In) 
            {
		        invariant_gps_measurement_factor(i, hessian, gradient, cost);
	        }
	    }

        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        for (int xy_z = 1; xy_z < 3; xy_z++) 
            {
                v_threshold = 0;
                a_threshold = 0;

		        if (xy_z == 1) 
                {
		            v_threshold = estimator_common_struct_.long_term_v_threshold;
		            a_threshold = estimator_common_struct_.long_term_a_threshold;
		        } 
                else if (xy_z == 2) 
                {
		            v_threshold = estimator_common_struct_.long_term_v_threshold;
		            a_threshold = estimator_common_struct_.long_term_a_threshold;
		        }

                start = 0;
                end = -1;
                contact_count = 0;

		        for (int i = frame_count; i > 1; i--) 
                {		
                    dv_mag_i = 0;
                    da_mag_i = 0;            
		            if (xy_z == 1) 
                    {
			            dv_mag_i = fabs(rs_temp_[i].d_v.block(3 * k, 0, 2, 1).norm());
			            da_mag_i = fabs(((rs_temp_[i].d_v.block(3 * k, 0, 2, 1) - rs_temp_[i - 1].d_v.block(3 * k, 0, 2, 1)) / dt).norm());
		            } else if (xy_z == 2) 
                    {
			            dv_mag_i = fabs(rs_temp_[i].d_v(3 * k + xy_z));
		            	da_mag_i = fabs((rs_temp_[i].d_v(3 * k + xy_z) - rs_temp_[i - 1].d_v(3 * k + xy_z)) / dt);
		            }

        		    if (contact_count == 0 && rs_temp_[i].Hard_Contact(k) && (dv_mag_i < v_threshold) && (da_mag_i < a_threshold))
                    {
			            end = i;
			            contact_count++;

		            } else if (contact_count > 0 && rs_temp_[i].Hard_Contact(k) && (dv_mag_i < v_threshold) && (da_mag_i < a_threshold))
                    {
			            contact_count++;

		            } else if (contact_count > 0 && (!rs_temp_[i].Hard_Contact(k) || (dv_mag_i > v_threshold)|| (da_mag_i > a_threshold)))
                    {
			            contact_count = 0;
			            start = i + 1;
		            }

		            if ((start > 0) && (end - start > 1)) {
			            long_term_stationary_foot_factor(start, end, xy_z, k, hessian, gradient, cost);
			            start = 0;
			            end = -1;

		            }

		        }
	        }
	    }
    }
    else
    {
        hessian = -hessian;
        gradient = -gradient;

        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        if (rs_temp_[1].Hard_Contact(k))     
            {
		        invariant_measurement_factor(1, k, hessian, gradient, cost);
	        }
	    }
	    if (rs_temp_[1].LiDAR_In) 
        {
	        invariant_lidar_measurement_factor(1, hessian, gradient, cost);
	    }

        if (frame_count > 1) 
        {
            bool is_LiDAR_In = false;
            Eigen::Vector3d p_lid;
            Eigen::Matrix3d R_lid;
	        invariant_propagation_factor(1, hessian, gradient, cost, lidar_in_i, x_lidar_i, R_lidar_i);
            // if (lidar_in_i)
            // {
            //     std::cout << "LiDAR in" << std::endl;
            //     invariant_lidar_propagation_factor(1, hessian, gradient, cost, x_lidar_i, R_lidar_i);
            // }
            

	    }
        
        hessian = -hessian;
        gradient = -gradient;    
    }

    hasnan = false;

} // end marg_update_nget_grad_hess_cost_debug

void InvFactors::batch_update_nget_grad_hess_cost
(
    bool is_for_marginalization, ROBOT_STATES *rs, 
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost,
    bool lidar_in_i, Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i
)
{
    if (!is_for_marginalization) 
    {
	    rs_temp_ = rs;
        if (cost != 0) 
        {
	        for (int p = 0; p < frame_count; p++) 
            {
		        contact_cov_array_temp = estimator_common_struct_.Variable_Contact_Cov(rs_temp_[p].Hard_Contact, rs_temp_[p].d_v);
		        int count = 0;
		        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
                {
		            if (rs_temp_[p].Hard_Contact(k) && rs_temp_[p + 1].Hard_Contact(k)) 
                    {			
			            sqrt_contact_info_temp.setZero();
			            sqrt_contact_info_temp.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)), 1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
			            fac_info_[p].prop_primitive_sqrt_info.block<3, 3>(15 + 3 * count, 15 + 3 * count) = sqrt_contact_info_temp;
			            count++;
		            }
		        }
	        }
        }
    }
    cost = 0;

    if (!is_for_marginalization) 
    {
	    hessian.setZero();
	    gradient.setZero();

	    invariant_prior_rvp_factor(hessian, gradient, cost);
	    invariant_prior_bias_factor(hessian, gradient, cost);

	    for (int i = 0; i < frame_count; i++) 
        {
            bool is_LiDAR_In = false;
            Eigen::Vector3d p_lid;
            Eigen::Matrix3d R_lid;
	        invariant_propagation_factor(i, hessian, gradient, cost, lidar_in_i, x_lidar_i, R_lidar_i);
            // if (lidar_in_i)
            // {
            //     std::cout << "LiDAR in" << std::endl;
            //     invariant_lidar_propagation_factor(i, hessian, gradient, cost, x_lidar_i, R_lidar_i);
            // }
            

	    }

	    for (int i = 0; i <= frame_count; i++) 
        {
	        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
            {
		        if (rs_temp_[i].Hard_Contact(k)) 
                {
		            invariant_measurement_factor(i, k, hessian, gradient, cost);
		        }
	        }
            if (rs_temp_[i].LiDAR_In) 
            {
                invariant_lidar_measurement_factor(i, hessian, gradient, cost);
	        }

            if (rs_temp_[i].GPS_In) 
            {
                invariant_gps_measurement_factor(i, hessian, gradient, cost);
            }
	    }
    }
    else
    {
        hessian = -hessian;
	    gradient = -gradient;

	    for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        if (rs_temp_[1].Hard_Contact(k)) 
            {
		        invariant_measurement_factor(1, k, hessian, gradient, cost);
	        }
	    }

        if (rs_temp_[1].LiDAR_In) 
        {
	        invariant_lidar_measurement_factor(1, hessian, gradient, cost);
	    }

	    if (frame_count > 1) 
        {
            bool is_LiDAR_In = false;
            Eigen::Vector3d p_lid;
            Eigen::Matrix3d R_lid;
	        invariant_propagation_factor(1, hessian, gradient, cost, lidar_in_i, x_lidar_i, R_lidar_i);
            // if (lidar_in_i)
            // {
            //     std::cout << "LiDAR in" << std::endl;
            //     invariant_lidar_propagation_factor(1, hessian, gradient, cost, x_lidar_i, R_lidar_i);
            // }

	    }

	    hessian = -hessian;
	    gradient = -gradient;
    }



} // end batch_update_nget_grad_hess_cost


void InvFactors::marg_update_nget_grad_hess_cost
(
    bool is_for_marginalization, ROBOT_STATES *rs,
    Eigen::Matrix<double, -1, 1> zeta_xi,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &hessian,
    Eigen::Matrix<double, Eigen::Dynamic, 1> &gradient, double &cost,
    bool lidar_in_i, Eigen::Matrix<double,3,1> x_lidar_i, Eigen::Matrix<double,3,3> R_lidar_i
)
{
    if (is_for_marginalization == false) 
    {
	    rs_temp_ = rs;

	    for (int p = 0; p < frame_count; p++) 
        {
		    contact_cov_array_temp = estimator_common_struct_.Variable_Contact_Cov(rs_temp_[p].Hard_Contact, rs_temp_[p].d_v);
		    int count = 0;
		    for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
            {
		        if (rs_temp_[p].Hard_Contact(k) && rs_temp_[p + 1].Hard_Contact(k)) 
                {
			        sqrt_contact_info_temp.setZero();
			        sqrt_contact_info_temp.diagonal() << 1.0 / sqrt(contact_cov_array_temp[k](0)), 1.0 / sqrt(contact_cov_array_temp[k](1)), 1.0 / sqrt(contact_cov_array_temp[k](2));
			        fac_info_[p].prop_primitive_sqrt_info.block<3, 3>(15 + 3 * count, 15 + 3 * count) = sqrt_contact_info_temp;
			        count++;
		        }
		    }
	    }
    } 

    cost = 0;

    if (is_for_marginalization == false) 
    {
        hessian.setZero();
        gradient.setZero();

        bias0_bar.block(0,0,3,1) = rs_temp_[0].Bias_Gyro + zeta_xi.block(0,0,3,1);
        bias0_bar.block(3,0,3,1) = rs_temp_[0].Bias_Acc + zeta_xi.block(3,0,3,1);

        Xi0.resize(9 + 3 * rs_temp_[0].contact_leg_num, 1);
        Xi0 = zeta_xi.block(6, 0, 9 + 3 * rs_temp_[0].contact_leg_num, 1);

        X_prior_inv.resize(5 + rs_temp_[0].contact_leg_num, 5 + rs_temp_[0].contact_leg_num);
        X_prior_inv.setIdentity();
	    X_prior_inv.block<3, 3>(0, 0) =  X_prior.block<3,3>(0,0).transpose();
	    X_prior_inv.block<3, 1>(0, 3) = -X_prior.block<3,3>(0,0).transpose()*X_prior.block<3,1>(0,3);
	    X_prior_inv.block<3, 1>(0, 4) = -X_prior.block<3,3>(0,0).transpose()*X_prior.block<3,1>(0,4);

        X0_bar.resize(5 + rs_temp_[0].contact_leg_num, 5 + rs_temp_[0].contact_leg_num);
	    X0_bar.setIdentity();
	    X0_bar.block<3, 3>(0, 0) = rs_temp_[0].Rotation;
	    X0_bar.block<3, 1>(0, 3) = rs_temp_[0].Velocity;
	    X0_bar.block<3, 1>(0, 4) = rs_temp_[0].Position;

        int count = 0;
	    for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        if (rs_temp_[0].Hard_Contact(k)) 
            {
		        X0_bar.block<3, 1>(0, 5 + count) = rs_temp_[0].d.block(3 * k, 0, 3, 1);
		        X_prior_inv.block<3,1>(0,5+count) = -X_prior.block<3,3>(0,0).transpose()*X_prior.block<3,1>(0,5+count);
		        count++;
	        }
	    }  
        X0_bar = Expm_seK_Vec(Xi0, 2 + rs_temp_[0].contact_leg_num) * X0_bar; 

        bias_Xbar_vee_inv_ret.resize(6 + 9 + 3 * rs_temp_[0].contact_leg_num, 1);
	    bias_Xbar_vee_inv_ret.setZero();
	    bias_Xbar_vee_inv_ret.block(0, 0, 6, 1) = bias0_bar - bias_prior;
	    bias_Xbar_vee_inv_ret.block(6, 0, 9 +3*rs_temp_[0].contact_leg_num, 1) = Logm_seK_Vec(X0_bar * X_prior_inv, 2 + rs_temp_[0].contact_leg_num);

        hessian.block(0, 0, rs_temp_[0].para_size, rs_temp_[0].para_size) = hessian_marg_factor;
	    gradient.block(0, 0, rs_temp_[0].para_size, 1) = hessian_marg_factor * (bias_Xbar_vee_inv_ret) + gradient_marg_factor;

        residual_update = hessian_marg_factor*bias_Xbar_vee_inv_ret + gradient_marg_factor;
        temp_cost = (b_vec + H_matrix * bias_Xbar_vee_inv_ret).norm();
	    cost += temp_cost * temp_cost;
        

	    for (int i = 0; i < frame_count; i++) 
        {
            bool is_LiDAR_In = false;
            Eigen::Vector3d p_lid;
            Eigen::Matrix3d R_lid;
	        invariant_propagation_factor(i, hessian, gradient, cost, lidar_in_i, x_lidar_i, R_lidar_i);
            // if (lidar_in_i)
            // {
            //     std::cout << "LiDAR in" << std::endl;
            //     invariant_lidar_propagation_factor(i, hessian, gradient, cost, x_lidar_i, R_lidar_i);
            // }

        }

        for (int i = 0; i <= frame_count; i++) 
        {
	        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
            {
		        if (rs_temp_[i].Hard_Contact(k)) 
                {
		            invariant_measurement_factor(i, k, hessian, gradient, cost);
		        }
	        }
            if (rs_temp_[i].LiDAR_In) 
            {
		        invariant_lidar_measurement_factor(i, hessian, gradient, cost);
	        }

            if (rs_temp_[i].GPS_In) 
            {
                invariant_gps_measurement_factor(i, hessian, gradient, cost);
            }
	    }

        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
            for (int xy_z = 1; xy_z < 3; xy_z++) 
            {
                v_threshold = 0;
                a_threshold = 0;

                if (xy_z == 1) 
                {
		            v_threshold = estimator_common_struct_.long_term_v_threshold;
		            a_threshold = estimator_common_struct_.long_term_a_threshold;
		        } 
                else if (xy_z == 2) 
                {
		            v_threshold = estimator_common_struct_.long_term_v_threshold;
		            a_threshold = estimator_common_struct_.long_term_a_threshold;
		        }

                start = 0;
                end = -1;
                contact_count = 0;

                for (int i = frame_count; i > 1; i--) 
                {
                    dv_mag_i = 0;
                    da_mag_i = 0;
                    if (xy_z == 1) 
                    {
			            dv_mag_i = fabs(rs_temp_[i].d_v.block(3 * k, 0, 2, 1).norm());
			            da_mag_i = fabs(((rs_temp_[i].d_v.block(3 * k, 0, 2, 1) - rs_temp_[i - 1].d_v.block(3 * k, 0, 2, 1)) / dt).norm());
		            } else if (xy_z == 2) 
                    {
			            dv_mag_i = fabs(rs_temp_[i].d_v(3 * k + xy_z));
			            da_mag_i = fabs((rs_temp_[i].d_v(3 * k + xy_z) - rs_temp_[i - 1].d_v(3 * k + xy_z)) / dt);
		            }

		            if (contact_count == 0 && rs_temp_[i].Hard_Contact(k) && (dv_mag_i < v_threshold) && (da_mag_i < a_threshold))
                    {
			            end = i;
			            contact_count++;
		            } else if (contact_count > 0 && rs_temp_[i].Hard_Contact(k) && (dv_mag_i < v_threshold) && (da_mag_i < a_threshold))
                    {
			            contact_count++;
		            } else if (contact_count > 0 && (!rs_temp_[i].Hard_Contact(k)|| (dv_mag_i > v_threshold)|| (da_mag_i > a_threshold))) 
                    {
			            contact_count = 0;
			            start = i + 1;
		            }

                    if ((start > 0) && (end - start > 1)) 
                    {
			            long_term_stationary_foot_factor(start, end, xy_z, k, hessian, gradient, cost);
			            start = 0;
			            end = -1;
		            }
                }
            }
        } 
    }
    else
    {
        hessian = -hessian;
	    gradient = -gradient;

        for (int k = 0; k < estimator_common_struct_.leg_no; k++) 
        {
	        if (rs_temp_[1].Hard_Contact(k)) 
            {
		        invariant_measurement_factor(1, k, hessian, gradient, cost);
	        } 
	    }
	    if (rs_temp_[1].LiDAR_In) 
        {
	        invariant_lidar_measurement_factor(1, hessian, gradient, cost);
	    }


        if (frame_count > 1) 
        {
            bool is_LiDAR_In = false;
            Eigen::Vector3d p_lid;
            Eigen::Matrix3d R_lid;
	        invariant_propagation_factor(1, hessian, gradient, cost, lidar_in_i, x_lidar_i, R_lidar_i);
            // if (lidar_in_i)
            // {
            //     std::cout << "LiDAR in" << std::endl;
            //     invariant_lidar_propagation_factor(1, hessian, gradient, cost, x_lidar_i, R_lidar_i);
            // }

	    }
        
        hessian = -hessian;
        gradient = -gradient;
    }

} // end marg_update_nget_grad_hess_cost