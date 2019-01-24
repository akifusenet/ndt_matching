#ifndef NDT_MATCHING__NDT_LIB_HPP_
#define NDT_MATCHING__NDT_LIB_HPP_

#include "ndt_matching/visibility_control.h"
#include "ndt_matching/VoxelGrid.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace ndt_matching
{
	using PointType = pcl::PointXYZ;
	using PointSourceType = pcl::PointXYZ;
	using PointTargetType = pcl::PointXYZ;

	class NdtLib
	{

	public:
		
		NdtLib();

		virtual ~NdtLib();

		void align(const Eigen::Matrix<float, 4, 4> guess);

		void setInputSource(sensor_msgs::msg::PointCloud2::SharedPtr input); // filtered_points

		void setInputTarget(sensor_msgs::msg::PointCloud2::SharedPtr input); // map

		void setInputTarget(pcl::PointCloud<PointTargetType>::Ptr target_cloud); // map

		bool isMapLoaded() const;

		bool isConverged() const;

		void setStepSize(double step_size);

		double getStepSize() const;

		void setResolution(float resolution);

		float getResolution() const;

		void setOutlierRatio(double olr);

		double getOutlierRatio() const;
		
		void setTransformationEpsilon(double trans_eps);

		double getTransformationEpsilon() const;

		double getTransformationProbability() const;

		void setMaximumIterations(int max_itr);

		int getMaximumIterations() const;

		int getFinalNumIteration() const;

		int getRealIterations() const;

		double getFitnessScore(double max_range = DBL_MAX);

		Eigen::Matrix<float, 4, 4> getFinalTransformation() const;		

		geometry_msgs::msg::PoseStamped getNdtPose();


	private:

		pcl::PointCloud<PointSourceType>::Ptr source_cloud_; // Ptr = boost::shared_ptr<PointCloud<PointT>>
		
		sensor_msgs::msg::PointCloud2::SharedPtr points_msg_;

		pcl::PointCloud<PointTargetType>::Ptr target_cloud_;

		geometry_msgs::msg::PoseStamped ndt_pose_msg_;

		bool map_loaded_;
		bool converged_;
		double step_size_;
		float resolution_;
		double outlier_ratio_;
		double transformation_epsilon_;
		double trans_probability_;
		int max_iterations_;
		int nr_iterations_;
		int real_iterations_;		

		Eigen::Matrix<float, 4, 4> final_transformation_, transformation_, previous_transformation_;


		void copyPointCloud2ToPclPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr input, 
											pcl::PointCloud<PointType>::Ptr& pcl_cloud);

		void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess);

		void computeAngleDerivatives(Eigen::Matrix<double, 6, 1> pose, 
									bool compute_hessian = true);

		double computeDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, 
								Eigen::Matrix<double, 6, 6> &hessian,
								pcl::PointCloud<PointSourceType> &trans_cloud,
								Eigen::Matrix<double, 6, 1> pose, 
								bool compute_hessian = true);
		
		void computePointDerivatives(Eigen::Vector3d &x, 
									Eigen::Matrix<double, 3, 6> &point_gradient, 
									Eigen::Matrix<double, 18, 6> &point_hessian, 
									bool computeHessian = true);
		
		double updateDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, 
								Eigen::Matrix<double, 6, 6> &hessian,
								Eigen::Matrix<double, 3, 6> point_gradient, 
								Eigen::Matrix<double, 18, 6> point_hessian,
								Eigen::Vector3d &x_trans, 
								Eigen::Matrix3d &c_inv, 
								bool compute_hessian = true);

		double computeStepLengthMT(const Eigen::Matrix<double, 6, 1> &x, 
								Eigen::Matrix<double, 6, 1> &step_dir,
								double step_init, 
								double step_max, 
								double step_min, 
								double &score,
								Eigen::Matrix<double, 6, 1> &score_gradient, 
								Eigen::Matrix<double, 6, 6> &hessian,
								pcl::PointCloud<PointSourceType> &trans_cloud);

		//Copied from ndt.h
	    double auxilaryFunction_PsiMT (double a, double f_a, double f_0, double g_0, double mu = 1.e-4);

	    //Copied from ndt.h
	    double auxilaryFunction_dPsiMT (double g_a, double g_0, double mu = 1.e-4);

   		double updateIntervalMT (double &a_l, double &f_l, double &g_l,
								double &a_u, double &f_u, double &g_u,
								double a_t, double f_t, double g_t);

		void computeHessian(Eigen::Matrix<double, 6, 6> &hessian, 
							pcl::PointCloud<PointSourceType> &trans_cloud, 
							Eigen::Matrix<double, 6, 1> &p);

		void updateHessian(Eigen::Matrix<double, 6, 6> &hessian,
						Eigen::Matrix<double, 3, 6> point_gradient, 
						Eigen::Matrix<double, 18, 6> point_hessian,
						Eigen::Vector3d &x_trans, 
						Eigen::Matrix3d &c_inv);

   		double trialValueSelectionMT (double a_l, double f_l, double g_l,
								double a_u, double f_u, double g_u,
								double a_t, double f_t, double g_t);


		pcl::PointCloud<PointSourceType> trans_cloud_;

		double gauss_d1_, gauss_d2_;
		Eigen::Vector3d j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;

		Eigen::Vector3d h_ang_a2_, h_ang_a3_,
						h_ang_b2_, h_ang_b3_,
						h_ang_c2_, h_ang_c3_,
						h_ang_d1_, h_ang_d2_, h_ang_d3_,
						h_ang_e1_, h_ang_e2_, h_ang_e3_,
						h_ang_f1_, h_ang_f2_, h_ang_f3_;

		VoxelGrid<PointSourceType> voxel_grid_;

	};


}  // namespace ndt_matching

#endif  // NDT_MATCHING__NDT_LIB_HPP_
