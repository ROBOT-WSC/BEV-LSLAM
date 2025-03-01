#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct LidarICPFactor
{
    LidarICPFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_, Eigen::Vector3d factor_)
    :curr_point(curr_point_), last_point(last_point_), factor(factor_){}

    template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lp{T(last_point.x()), T(last_point.y()), T(last_point.z())};
		Eigen::Matrix<T, 3, 1> factor_xyz{T(factor.x()), T(factor.y()), T(factor.z())};
		
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 1> fp;
		fp = q_last_curr * cp + t_last_curr;

        residual[0] = factor_xyz[0]*(fp[0]-lp[0]);
		residual[1] = factor_xyz[1]*(fp[1]-lp[1]);
		residual[2] = factor_xyz[2]*(fp[2]-lp[2]);

		return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_, const Eigen::Vector3d factor_)
	{
		return (new ceres::AutoDiffCostFunction<LidarICPFactor, 3, 4, 3>(
			new LidarICPFactor(curr_point_, last_point_, factor_)));
	}

	Eigen::Vector3d curr_point, last_point, factor;
};

template <typename T> inline
void Quaternion2EulerAngle(const T q[4], T ypr[3])
{
	// roll (x-axis rotation)
	T sinr_cosp = T(2) * (q[0] * q[1] + q[2] * q[3]);
	T cosr_cosp = T(1) - T(2) * (q[1] * q[1] + q[2] * q[2]);
	ypr[2] = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	T sinp = T(2) * (q[0] * q[2] - q[1] * q[3]);
	if (sinp >= T(1))
	{
		ypr[1] = T(M_PI / 2); // use 90 degrees if out of range
	}
	else if (sinp <= T(-1))
	{
		ypr[1] = -T(M_PI / 2); // use 90 degrees if out of range
	}
	else
	{
		ypr[1] = asin(sinp);
	}
	
	// yaw (z-axis rotation)
	T siny_cosp = T(2) * (q[0] * q[3] + q[1] * q[2]);
	T cosy_cosp = T(1) - T(2) * (q[2] * q[2] + q[3] * q[3]);
	ypr[0] = atan2(siny_cosp, cosy_cosp);
};

template <typename T> 
void YawToRotationMatrix(const T yaw, Eigen::Matrix<T, 3, 3> R)
{
	T y = yaw;
	R(0,0) = cos(y);
	R(0,1) = -sin(y);
	R(0,2) = T(0);
	R(1,0) = sin(y);
	R(1,1) = cos(y);
	R(1,2) = T(0);
	R(2,0) = T(0);
	R(2,1) = T(0);
	R(2,2) = T(1);
};

struct LidarICPFactor_xy
{
    LidarICPFactor_xy(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_)
    :curr_point(curr_point_), last_point(last_point_){}

    template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(1)};
		Eigen::Matrix<T, 3, 1> lp{T(last_point.x()), T(last_point.y()), T(1)};

		T q_last_curr[4];
		q_last_curr[0] = q[3]; // ceres in w, x, y, z order
		q_last_curr[1] = q[0];
		q_last_curr[2] = q[1];
		q_last_curr[3] = q[2];

		T ypr[3];

		Quaternion2EulerAngle(q_last_curr, ypr);

		Eigen::Matrix<T, 3, 3> R_last_curr;
		YawToRotationMatrix(ypr[0], R_last_curr);


		Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], T(0)};

		Eigen::Matrix<T, 3, 1> fp;
		fp = R_last_curr * cp + t_last_curr;

        residual[0] = fp[0]-lp[0];
		residual[1] = fp[1]-lp[1];

		return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_)
	{
		return (new ceres::AutoDiffCostFunction<LidarICPFactor_xy, 2, 4, 3>(
			new LidarICPFactor_xy(curr_point_, last_point_)));
	}

	Eigen::Vector3d curr_point, last_point;
};



template <typename T> 
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll,  Eigen::Matrix<T, 3, 3> R)
{
	T y = yaw ;
	T p = pitch ;
	T r = roll ;

	R(0,0) = cos(y) * cos(p);
	R(0,1) = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R(0,2) = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R(1,0) = sin(y) * cos(p);
	R(1,1) = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R(1,2) = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R(2,0) = -sin(p);
	R(2,1) = cos(p) * sin(r);
	R(2,2) = cos(p) * cos(r);
};

template <typename T> 
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{
	T y = yaw / T(180.0) * T(M_PI);
	T p = pitch / T(180.0) * T(M_PI);
	T r = roll / T(180.0) * T(M_PI);

	R[0] = cos(y) * cos(p);
	R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R[3] = sin(y) * cos(p);
	R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R[6] = -sin(p);
	R[7] = cos(p) * sin(r);
	R[8] = cos(p) * cos(r);
};


template <typename T> 
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
	inv_R[0] = R[0];
	inv_R[1] = R[3];
	inv_R[2] = R[6];
	inv_R[3] = R[1];
	inv_R[4] = R[4];
	inv_R[5] = R[7];
	inv_R[6] = R[2];
	inv_R[7] = R[5];
	inv_R[8] = R[8];
};

template <typename T> 
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
	r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
	r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
	r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};

template <typename T>
T NormalizeAngle(const T& angle_degrees)
{
	if (angle_degrees > T(180.0))
		return angle_degrees - T(360.0);
	else if (angle_degrees < T(-180.0))
		return angle_degrees + T(360.0);
	else
		return angle_degrees;
};

struct LidarICPFactor_z
{
    LidarICPFactor_z(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_)
    :curr_point(curr_point_), last_point(last_point_){}

    template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lp{T(last_point.x()), T(last_point.y()), T(last_point.z())};
		
		T q_last_curr[4];
		q_last_curr[0] = q[3]; // ceres in w, x, y, z order
		q_last_curr[1] = q[0];
		q_last_curr[2] = q[1];
		q_last_curr[3] = q[2];

		T ypr[3];

		Quaternion2EulerAngle(q_last_curr, ypr);

		Eigen::Matrix<T, 3, 3> R_last_curr;
		YawPitchRollToRotationMatrix(T(0), ypr[1], ypr[2], R_last_curr);


		Eigen::Matrix<T, 3, 1> t_last_curr{T(0), T(0), t[2]};

		Eigen::Matrix<T, 3, 1> fp;
		fp = R_last_curr * cp + t_last_curr;

        residual[0] = fp[2]-lp[2];

		return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_)
	{
		return (new ceres::AutoDiffCostFunction<LidarICPFactor_z, 1, 4, 3>(
			new LidarICPFactor_z(curr_point_, last_point_)));
	}

	Eigen::Vector3d curr_point, last_point;
};

struct Reprojection_Factor
{
    Reprojection_Factor(Eigen::Vector3d curr_point_, Eigen::Vector2d last_point_, Eigen::Vector3d image_param_)
    :curr_point(curr_point_), last_point(last_point_), image_param(image_param_){}

    template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 2, 1> lp{T(last_point[0]), T(last_point[1])};

		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 1> fp;
		fp = q_last_curr * cp + t_last_curr;

		T image_height = T(image_param.x());
		T image_length = T(image_param.y());
		T image_resolution = T(image_param.z());

		T BEV_now_x = (image_height/T(2)-fp[0])/image_resolution;
		T BEV_now_y = (image_length/T(2)-fp[1])/image_resolution;

		// T q_i_tmp[4];
		// q_i_tmp[0] = q[3]; // ceres in w, x, y, z order
		// q_i_tmp[1] = q[0];
		// q_i_tmp[2] = q[1];
		// q_i_tmp[3] = q[2];
		// T ypr[3];
		// Quaternion2EulerAngle(q_i_tmp, ypr);
		// Eigen::Matrix<T, 3, 3> R_last_curr;
		// YawToRotationMatrix(ypr[0], R_last_curr);

		// Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], T(0)};

		// Eigen::Matrix<T, 3, 1> fp;
		// fp = R_last_curr * cp + t_last_curr;

        residual[0] = BEV_now_x-lp[0];
		residual[1] = BEV_now_y-lp[1];

		return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector2d last_point_, const Eigen::Vector3d image_param_)
	{
		return (new ceres::AutoDiffCostFunction<Reprojection_Factor, 2, 4, 3>(
			new Reprojection_Factor(curr_point_, last_point_, image_param_)));
	}

	Eigen::Vector3d curr_point, image_param;
	Eigen::Vector2d last_point;
};

struct local_BA_Factor
{
    local_BA_Factor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_ ,Eigen::Vector3d factor_)
    :curr_point(curr_point_), last_point(last_point_), factor(factor_){}

    template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lp{T(last_point.x()), T(last_point.y()), T(last_point.z())};
		Eigen::Matrix<T, 3, 1> factor_xyz{T(factor.x()), T(factor.y()), T(factor.z())};
		
		Eigen::Quaternion<T> q_world_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_world_curr{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 1> fp;
		fp = q_world_curr * cp + t_world_curr;

        residual[0] = factor_xyz[0]*(fp[0]-lp[0]);
		residual[1] = factor_xyz[1]*(fp[1]-lp[1]);
		residual[2] = factor_xyz[2]*(fp[2]-lp[2]);

		return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_, const Eigen::Vector3d factor_)
	{
		return (new ceres::AutoDiffCostFunction<local_BA_Factor, 3, 4, 3>(
			new local_BA_Factor(curr_point_, last_point_, factor_)));
	}

	Eigen::Vector3d curr_point, last_point, factor;
};

struct local_BA_Factor_xy
{
    local_BA_Factor_xy(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_)
    :curr_point(curr_point_), last_point(last_point_){}

    template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(1)};
		Eigen::Matrix<T, 3, 1> lp{T(last_point.x()), T(last_point.y()), T(1)};

		T q_last_curr[4];
		q_last_curr[0] = q[3]; // ceres in w, x, y, z order
		q_last_curr[1] = q[0];
		q_last_curr[2] = q[1];
		q_last_curr[3] = q[2];

		T ypr[3];

		Quaternion2EulerAngle(q_last_curr, ypr);

		Eigen::Matrix<T, 3, 3> R_last_curr;
		YawToRotationMatrix(ypr[0], R_last_curr);


		Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], T(0)};

		Eigen::Matrix<T, 3, 1> fp;
		fp = R_last_curr * cp + t_last_curr;

        residual[0] = fp[0]-lp[0];
		residual[1] = fp[1]-lp[1];

		return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_)
	{
		return (new ceres::AutoDiffCostFunction<local_BA_Factor_xy, 2, 4, 3>(
			new local_BA_Factor_xy(curr_point_, last_point_)));
	}

	Eigen::Vector3d curr_point, last_point;
};

struct local_BA_Factor_z
{
    local_BA_Factor_z(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_)
    :curr_point(curr_point_), last_point(last_point_){}

    template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lp{T(last_point.x()), T(last_point.y()), T(last_point.z())};
		
		T q_last_curr[4];
		q_last_curr[0] = q[3]; // ceres in w, x, y, z order
		q_last_curr[1] = q[0];
		q_last_curr[2] = q[1];
		q_last_curr[3] = q[2];

		T ypr[3];

		Quaternion2EulerAngle(q_last_curr, ypr);

		Eigen::Matrix<T, 3, 3> R_last_curr;
		YawPitchRollToRotationMatrix(T(0), ypr[1], ypr[2], R_last_curr);


		Eigen::Matrix<T, 3, 1> t_last_curr{T(0), T(0), t[2]};

		Eigen::Matrix<T, 3, 1> fp;
		fp = R_last_curr * cp + t_last_curr;

        residual[0] = fp[2]-lp[2];

		return true;
    }
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_)
	{
		return (new ceres::AutoDiffCostFunction<local_BA_Factor_z, 1, 4, 3>(
			new local_BA_Factor_z(curr_point_, last_point_)));
	}

	Eigen::Vector3d curr_point, last_point;
};

struct FourDOFError
{
	FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
		: t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){}

	template <typename T>
	bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T w_R_i[9];
		YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);

		T i_R_w[9];
		RotationMatrixTranspose(w_R_i, i_R_w);//旋转矩阵是酉矩阵

		T t_i_ij[3];
		RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x));
		residuals[1] = (t_i_ij[1] - T(t_y));
		residuals[2] = (t_i_ij[2] - T(t_z));
		residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double relative_yaw, const double pitch_i, const double roll_i) 
	{
		return (new ceres::AutoDiffCostFunction<FourDOFError, 4, 1, 3, 1, 3>(
			new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
	}

	double t_x, t_y, t_z;
	double relative_yaw, pitch_i, roll_i;
};

class AngleLocalParameterization
{
  public:
	template <typename T>
	bool operator()(const T* theta_radians, const T* delta_theta_radians, T* theta_radians_plus_delta) const
	{
		*theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
		return true;
	}

	static ceres::LocalParameterization* Create()
	{
		return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
	}
};

struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		// 考虑运动补偿，ktti点云已经补偿过所以可以忽略下面的对四元数slerp插值以及对平移的线性插值
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		// Odometry线程时，下面是将当前帧Lidar坐标系下的cp点变换到上一帧的Lidar坐标系下，然后在上一帧的Lidar坐标系计算点到线的残差距离
		// Mapping线程时，下面是将当前帧Lidar坐标系下的cp点变换到world坐标系下，然后在world坐标系下计算点到线的残差距离
		lp = q_last_curr * cp + t_last_curr;

		// 点到线的计算如下图所示
		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		// 最终的残差本来应该是residual[0] = nu.norm() / de.norm(); 为啥也分成3个，我也不知
		// 道，从我试验的效果来看，确实是下面的残差函数形式，最后输出的pose精度会好一点点，这里需要
		// 注意的是，所有的residual都不用加fabs，因为Ceres内部会对其求 平方 作为最终的残差项
		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 3, 4, 3>(
//					             ^  ^  ^
//					             |  |  |
//			      残差的维度 ____|  |  |
//			 优化变量q的维度 _______|  |
//			 优化变量t的维度 __________|
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};