#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <cstdlib>
#include <ctime>

namespace PegInHole
{
    static Eigen::Vector3d straightMove(const Eigen::Vector3d origin,
        const Eigen::Vector3d current_position,
        const Eigen::Matrix<double, 6, 1> current_velocity,
        const int dir,
        const double speed,
        const double current_time,
        const double init_time)
    {
        // double desent_speed = -0.02; //-0.005; // 5cm/s

        Eigen::Vector3d desired_position;
        Eigen::Vector3d desired_linear_velocity;
        Eigen::Vector3d f_star;
        Eigen::Matrix3d K_p; 
        Eigen::Matrix3d K_v;
        
        K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
        K_v << 100, 0, 0, 0, 100, 0, 0, 0, 100;

        
        desired_position = origin;
        desired_position(dir) = origin(dir) + speed*(current_time - init_time);
        
        desired_linear_velocity.setZero();
        desired_linear_velocity(dir) = speed;
                
        
        f_star = K_p * (desired_position - current_position) + K_v * ( desired_linear_velocity- current_velocity.head<3>());  
        
       
        return f_star;
    }

    static Eigen::Vector3d oneDofMove(const Eigen::Vector3d origin,
        const Eigen::Vector3d current_position,
        const double target_position,
        const Eigen::Matrix<double, 6, 1> current_velocity,
        const double current_time,
        const double init_time,
        const double duration,
        const double desired_speed, //only positive value!!!
        const int direction) // 0 -> x, 1 -> y, 2 -> z //DESIRED DIRECTION!!
    {
        double speed;
        Eigen::Vector3d desired_position;
        Eigen::Vector3d desired_velocity;
        Eigen::Vector3d f_star;
        Eigen::Matrix3d K_p; 
        Eigen::Matrix3d K_v;
        
        K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
        K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

        if(origin(direction) < target_position) speed = desired_speed;
        else speed = -desired_speed;
        
        if(direction == 0)
        {
            desired_position(0) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(0), target_position, 0, 0);
            desired_position.tail<2>() = origin.tail<2>();
            desired_velocity << speed, 0, 0;
        }
        if(direction == 1.0)
        {
            desired_position(0) = origin(0);
            desired_position(1) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(1), target_position, 0, 0);
            desired_position(2) = origin(2);
            desired_velocity << 0, speed, 0;
        }
        if(direction == 2.0)
        {
            desired_position.head<2>() = origin.head<2>();
            desired_position(2) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(2), target_position, 0, 0);
            desired_velocity << 0, 0, speed;
        }

        f_star = K_p * (desired_position - current_position) + K_v * ( desired_velocity- current_velocity.head<3>());  
       
        return f_star;    
    }

    static Eigen::Vector3d twoDofMove(const Eigen::Vector3d origin,
        const Eigen::Vector3d current_position,
        const Eigen::Vector3d target_position,
        const Eigen::Matrix<double, 6, 1> current_velocity,
        const double current_time,
        const double init_time,
        const double duration,
        const double desired_speed,
        const int direction) // 0 -> x, 1 -> y, 2 -> z //NOT DESIRED DIRECTION!!
    {
        Eigen::Vector3d speed; // x, y, z
        Eigen::Vector3d desired_position;
        Eigen::Vector3d desired_velocity;
        Eigen::Vector3d f_star;
        Eigen::Matrix3d K_p; 
        Eigen::Matrix3d K_v;
        
        K_p << 5500, 0, 0, 0, 5500, 0, 0, 0, 5500;
        K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

        for(size_t i = 0; i < 3; i++)
        {
            if(origin(i) < target_position(i)) speed(i) = desired_speed;
            else speed(i) = -desired_speed;
        }

        if(direction == 0.0) //NOT WANT TO MOVE ALONG X - AXIS
        {
            desired_position(0) = target_position(0);
            desired_position(1) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(1), target_position(1), 0, 0);
            desired_position(2) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(2), target_position(2), 0, 0);
            desired_velocity << 0, speed(1), speed(2);
        }
        if(direction == 1.0)
        {
            desired_position(0) = dyros_math::cubic(current_time, init_time, init_time + duration,origin(0),target_position(0),0,0);
            desired_position(1) = target_position(1);
            desired_position(2) = dyros_math::cubic(current_time, init_time, init_time + duration,origin(2),target_position(2),0,0);
            desired_velocity << speed(0), 0, speed(2);
        }
        if(direction == 2.0) //NOT WANT TO MOVE ALONG Z - AXIS
        {
            desired_position(0) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(0), target_position(0), 0, 0);
            desired_position(1) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(1), target_position(1), 0, 0);
            desired_position(2) = target_position(2);
            desired_velocity << speed(0), speed(1), 0;
        }
        
        f_star = K_p * (desired_position - current_position) + K_v * ( desired_velocity- current_velocity.head<3>());  
        // std::cout<<"-------------------------------------------"<<std::endl;
        // std::cout<<"target_position : "<<target_position.transpose()<<std::endl;
        // std::cout<<"current_position : "<<current_position.transpose()<<std::endl;
        // std::cout<<"desired speed : "<<desired_velocity.transpose()<<std::endl;
        // std::cout<<"current speed : "<<current_velocity.head<3>().transpose()<<std::endl;
        // std::cout<<"f_star : "<<f_star.transpose()<<std::endl;
        return f_star;    
    }


    static Eigen::Matrix<double, 6, 1> keepCurrentState(const Eigen::Vector3d initial_position,
        const Eigen::Matrix3d initial_rotation,
        const Eigen::Vector3d position,
        const Eigen::Matrix3d rotation,
        const Eigen::Matrix<double, 6, 1> current_velocity, 
        const double kp = 5000, const double kv = 100)
    {
        Eigen::Vector3d desired_position;
        Eigen::Vector3d desired_linear_velocity;
        Eigen::Vector3d delphi_delta;
        Eigen::Vector3d f_star;
        Eigen::Vector3d m_star;
        Eigen::Vector6d f_star_zero;
        Eigen::Matrix3d kp_m; 
        Eigen::Matrix3d kv_m;

        kp_m = Eigen::Matrix3d::Identity() * kp;
        kv_m = Eigen::Matrix3d::Identity() * kv;

        desired_position = initial_position;
        desired_linear_velocity.setZero();

        delphi_delta = -0.5 * dyros_math::getPhi(rotation, initial_rotation);

        f_star = kp_m * (desired_position - position) + kv_m * ( desired_linear_velocity- current_velocity.head<3>());  
        m_star = (1.0) * 200.0* delphi_delta+ 5.0*(-current_velocity.tail<3>()) ;

        f_star_zero.head<3>() = f_star;
        f_star_zero.tail<3>() = m_star;

        return f_star_zero;    
    }    

    static Eigen::Vector3d  generateSpiral(const Eigen::Vector3d origin, 
        const Eigen::Vector3d current_position,
        const Eigen::Matrix<double, 6, 1> current_velocity,
        const double pitch,
        const double lin_vel,
        const int dir, //the direction where a peg is inserted
        const double current_time,
        const double init_time,
        const double spiral_duration)
    {
        // double pitch = 0.0010; 
        // double lin_vel = 0.005; 
        Eigen::Vector2d start_point;
        Eigen::Vector2d traj;
        Eigen::Vector3d desired_position;
        Eigen::Vector3d desired_linear_velocity;
        Eigen::Vector3d f_star;
        Eigen::Matrix3d K_p;
        Eigen::Matrix3d K_v;

        Eigen::Vector3d real_origin;
        Eigen::Vector3d real_current_position;

        K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
        K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

        if(dir == 0) start_point << origin(1), origin(2);
        if(dir == 1) start_point << origin(0), origin(2);
        if(dir == 2) start_point << origin(0), origin(1);
        
        traj = dyros_math::spiral(current_time, init_time, init_time + spiral_duration, start_point, lin_vel, pitch);

        if(dir == 0) desired_position << origin(dir), traj(0), traj(1);
        if(dir == 1) desired_position << traj(0), origin(dir), traj(1);
        if(dir == 2) desired_position << traj(0), traj(1), origin(dir);
        
        desired_linear_velocity.setZero();
    
        f_star = K_p * (desired_position - current_position) + K_v * (desired_linear_velocity- current_velocity.head<3>());  
            
        return f_star;
    }

    static Eigen::Matrix<double, 3, 1>  generateSpiralWithRotation(const Eigen::Matrix3d initial_rotation_M, 
        const Eigen::Matrix3d rotation_M,
        const Eigen::Matrix<double, 3, 1> current_angular_velocity,
        const double current_time,
        const double init_time,
        const double duration,
        const double direction,  //0 = the first motion, 1 = CCW, 2 == CW
        const double axis) // 0 = x-axis, 1 = y-axis, 2 = z-axis
    {
        double target_angle = 3.0*M_PI/180;
        double ori_change_theta;
        
        Eigen::Matrix3d rotation_matrix;
        Eigen::Matrix3d target_rotation_M;  
        Eigen::Vector3d delphi_delta;
        Eigen::Vector3d m_star;
                
        if(direction == 0.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration/2, 0, target_angle, 0, 0);    
        }
        
        if(direction == 1.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, target_angle, -target_angle, 0, 0);    
        }          

        if(direction == 2.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, -target_angle, target_angle, 0, 0);
        }            
        
        if(axis == 0) rotation_matrix << 1, 0, 0, 0, cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta);
        if(axis == 1) rotation_matrix << cos(ori_change_theta), 0, sin(ori_change_theta), 0, 1, 0, -sin(ori_change_theta), 0, cos(ori_change_theta);
        if(axis == 2) rotation_matrix << cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta), 0, 0, 0, 1;
        
        target_rotation_M = rotation_matrix * initial_rotation_M;

        delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);
        
        m_star = (1.0) * 200.0* delphi_delta+ 5.0*(-current_angular_velocity);
                
        return m_star;
    }

    static Eigen::Matrix<double, 3, 1>  generateSpiralWithRotationGainEe(const Eigen::Matrix3d initial_rotation_M, 
        const Eigen::Matrix3d rotation_M,
        const Eigen::Matrix<double, 3, 1> current_angular_velocity,
        const double current_time,
        const double init_time,
        const double duration,
        const double direction,  //0 = the first motion, 1 = CCW, 2 == CW
        const double axis,  // 0 = x-axis, 1 = y-axis, 2 = z-axis
        const double kp,
        const double kv)
    {
        double target_angle = 3.0*M_PI/180;
        double ori_change_theta;
        
        Eigen::Matrix3d rotation_matrix;
        Eigen::Matrix3d target_rotation_M;  
        Eigen::Vector3d delphi_delta;
        Eigen::Vector3d m_star;
        Eigen::Matrix3d K_p; 
        Eigen::Matrix3d K_v;

        K_p << kp, 0, 0, 0, kp, 0, 0, 0, kp;
        K_v << kv, 0, 0, 0, kv, 0, 0, 0, kv;
                
        if(direction == 0.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration/2, 0, target_angle, 0, 0);    
        }
        
        if(direction == 1.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, target_angle, -target_angle, 0, 0);    
        }          

        if(direction == 2.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, -target_angle, target_angle, 0, 0);
        }            
        
        if(axis == 0) rotation_matrix << 1, 0, 0, 0, cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta);
        if(axis == 1) rotation_matrix << cos(ori_change_theta), 0, sin(ori_change_theta), 0, 1, 0, -sin(ori_change_theta), 0, cos(ori_change_theta);
        if(axis == 2) rotation_matrix << cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta), 0, 0, 0, 1;
        
        target_rotation_M = initial_rotation_M * rotation_matrix;

        delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);
        
        m_star = (1.0) * K_p * delphi_delta+ K_v * (-current_angular_velocity);
                
        return m_star;
    }

    static Eigen::Matrix<double, 3, 1>  generateSpiralWithRotationEe(const Eigen::Matrix3d initial_rotation_M, 
        const Eigen::Matrix3d rotation_M,
        const Eigen::Matrix<double, 3, 1> current_angular_velocity,
        const double current_time,
        const double init_time,
        const double duration,
        const double direction,  //0 = the first motion, 1 = CCW, 2 == CW
        const double axis,
        const int rand) // 0 = x-axis, 1 = y-axis, 2 = z-axis
    {
        double target_angle = 1.5*M_PI/180;
        double ori_change_theta;
        
        Eigen::Matrix3d rotation_matrix;
        Eigen::Matrix3d target_rotation_M;  
        Eigen::Vector3d delphi_delta;
        Eigen::Vector3d m_star;

        
        if(direction == 0.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration/2, 0, target_angle, 0, 0);    
        }
        
        if(direction == 1.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, target_angle, -target_angle, 0, 0);    
        }          

        if(direction == 2.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, -target_angle, target_angle, 0, 0);
        }

        if(direction == 3.0)
        {
            ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration/2, 0, -target_angle, 0, 0);    
        }            
        
        if(axis == 0) rotation_matrix << 1, 0, 0, 0, cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta);
        if(axis == 1) rotation_matrix << cos(ori_change_theta), 0, sin(ori_change_theta), 0, 1, 0, -sin(ori_change_theta), 0, cos(ori_change_theta);
        if(axis == 2) rotation_matrix << cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta), 0, 0, 0, 1;
        
        target_rotation_M = initial_rotation_M * rotation_matrix;

        delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);
        
        m_star = (1.0) * 200.0* delphi_delta+ 5.0*(-current_angular_velocity);
                
        return m_star;
    }

    static Eigen::Vector3d keepOrientationPerpenticular(const Eigen::Matrix3d initial_rotation_M,
		const Eigen::Matrix3d rotation_M,
		const Eigen::Matrix<double, 6, 1> current_velocity,
		const double duration,
		const double current_time,
		const double init_time)
	{
		Eigen::Matrix3d target_rotation_M;
		Eigen::Vector3d delphi_delta;
		Eigen::Vector3d m_star;
		Eigen::Vector5d angle_set_45;
		Eigen::Vector5d angle_set_error;
		Eigen::Vector3d euler_angle;

		double val;
		double e;

		double roll, alpha;
		double pitch, beta;
		double yaw, gamma;

		double min;
		int index;

		euler_angle = dyros_math::rot2Euler(initial_rotation_M);
		roll = euler_angle(0);
		pitch = euler_angle(1);
		yaw = euler_angle(2);

		val = initial_rotation_M(2, 2);
		e = 1.0 - fabs(val);
        
		// angle_set_45 << -135, -45, 45, 135;
        angle_set_45 << -180.0, -90.0, 0.0, 90.0, 180.0;
		angle_set_45 = angle_set_45 * DEG2RAD;

		if (val > 0 && e <= 0.01) //upward
		{
			roll = 0;
			pitch = 0;

			for (size_t i = 0; i < 5; i++)
			{
				angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(2));
			}

			for (size_t i = 0; i < 5; i++)
			{
				if (angle_set_error(i) == angle_set_error.minCoeff()) index = i;
			}


			yaw = angle_set_45(index);

		}
		else if (val < 0 && e <= 0.01) //downward
		{
			if (roll > 0) roll = 180 * DEG2RAD;
			else roll = -180 * DEG2RAD;

			pitch = 0;

			for (size_t i = 0; i < 5; i++)
			{
				angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(2));
			}
			for (size_t i = 0; i < 5; i++)
			{
				if (angle_set_error(i) == angle_set_error.minCoeff()) index = i;
			}

			yaw = angle_set_45(index);
		}

		else //on xy plane
		{
			roll = euler_angle(0);
			yaw = euler_angle(2);

			for (size_t i = 0; i < 5; i++)
			{
				angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(1));
			}

			for (size_t i = 0; i < 5; i++)
			{
				if (angle_set_error(i) == angle_set_error.minCoeff()) index = i;
			}

			pitch = angle_set_45(index);
		}

		alpha = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
		beta = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch, 0, 0);
		gamma = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

		target_rotation_M = dyros_math::rotateWithZ(gamma) * dyros_math::rotateWithY(beta) * dyros_math::rotateWithX(alpha);

		delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

		m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

		return m_star;
	}    
    
    static Eigen::Vector3d keepOrientationPerpenticularOnlyXY(const Eigen::Matrix3d initial_rotation_M,
        const Eigen::Matrix3d rotation_M,
        const Eigen::Matrix<double, 6, 1> current_velocity,
        const double duration,
        const double current_time,
        const double init_time)
    {
        Eigen::Matrix3d target_rotation_M;
        Eigen::Vector3d delphi_delta;
        Eigen::Vector3d m_star;
        Eigen::Vector3d euler_angle;

        double val;
        double e;

        double roll, alpha;
        double pitch, beta;
        double yaw, gamma;

        euler_angle = dyros_math::rot2Euler(initial_rotation_M);
        roll = euler_angle(0);
        pitch = euler_angle(1);
        yaw = euler_angle(2);

        val = initial_rotation_M(2, 2);
        e = 1.0 - fabs(val);

        if (val > 0) //upward
        {
            roll = 0;
            pitch = 0;
        }
        
        else //downward
        {
            if (roll > 0) roll = 180 * DEG2RAD;
            else roll = -180 * DEG2RAD;
            pitch = 0;
        }

        alpha = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
        beta = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch, 0, 0);
        gamma = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

        target_rotation_M = dyros_math::rotateWithZ(gamma) * dyros_math::rotateWithY(beta) * dyros_math::rotateWithX(alpha);

        delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

        m_star = (1.0) * 250.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

        return m_star;
    }

    static Eigen::Vector3d rotateWithGlobalAxis(const Eigen::Matrix3d initial_rotation_M,
        const Eigen::Matrix3d rotation_M,
        const Eigen::Matrix<double, 6, 1> current_velocity,
        const double goal,
        const double init_time,
        const double current_time,
        const double end_time,
        const int dir) // 0 = x-axis, 1 = y-axis, 2 = z-axis
    {
        Eigen::Matrix3d rot;
        Eigen::Matrix3d target_rotation_M;
        Eigen::Vector3d delphi_delta;
        Eigen::Vector3d m_star;
        

        double theta;
        double run_time;
        double duration = (end_time - init_time)/4.0; //0.5s
        
        run_time = current_time - init_time;

        
        if(run_time < duration)
        {
            theta = dyros_math::cubic(current_time, init_time, init_time + duration, 0, goal, 0, 0);        
        } 

        else if(duration <= run_time && run_time < 3*duration)
        {
            theta = dyros_math::cubic(current_time, init_time + duration, init_time + 3*duration, goal, -goal, 0, 0);            
        }

        else// if(3*duration/2 <= run_time && run_time < 2*duration)
        {
            theta = dyros_math::cubic(current_time, init_time + 3*duration, init_time + 4*duration, -goal, 0, 0, 0);
        }
                
        
        if(dir == 0)    rot << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
        if(dir == 1)    rot << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
        if(dir == 2)    rot << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
        
        
        target_rotation_M = rot * initial_rotation_M;

        delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

        m_star = (1.0) * 200.0 * delphi_delta + 5.0*(-current_velocity.tail<3>());

                
        // std::cout<<"----------------------"<<std::endl;
        // std::cout<<"time: "<<run_time<<std::endl;
        // std::cout<<"duration : "<<duration<<std::endl;
        // std::cout<<"theta: "<<theta*180/M_PI<<std::endl;
        return m_star;
    }

    static double calculateFriction(const int dir, const Eigen::Vector3d f_measured, double friction)
    {
        if(dir == 0) friction = sqrt(f_measured(1)*f_measured(1) +f_measured(2)*f_measured(2));
        if(dir == 1) friction = sqrt(f_measured(0)*f_measured(0) +f_measured(2)*f_measured(2));
        if(dir == 2) friction = sqrt(f_measured(0)*f_measured(0) +f_measured(1)*f_measured(1));

        return friction;
    }

   
    static Eigen::Vector3d vectorNormalization(const Eigen::Vector3d v)
    {
        Eigen::Vector3d n;

        n = v/sqrt(pow(v(0),2) + pow(v(1),2) + pow(v(2),2));

        return n;
    }
    static Eigen::Vector3d computeNomalVector(const Eigen::Vector3d p1,
        const Eigen::Vector3d p2,
        const Eigen::Vector3d p3)
    {
        Eigen::Vector3d r1;
        Eigen::Vector3d r2;
        Eigen::Vector3d r3;
        Eigen::Vector3d result;

        r1 = p1 - p2;
        r2 = p1 - p3;

        r3 = r1.cross(r2);
        r3 = r3/sqrt(pow(r3(0),2) + pow(r3(1),2) + pow(r3(2),2));

        result = r3;
        return result;
    }
   

};
