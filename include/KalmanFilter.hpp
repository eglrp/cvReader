#pragma once
//Create by steve in 16-12-21 at 下午11:01
//
// Created by steve on 16-12-21.
//

#include <Eigen/Dense>

#ifndef CVREADER_KALMANFILTER_HPP
#define CVREADER_KALMANFILTER_HPP

namespace own{
    template <typename T,int state_num,int observe_num>
    class KalmanFilter{
    public:
        KalmanFilter(T eval_sigma,T noise_sigma,Eigen::VectorXd state_sigma_vec)
        {

            //Check
            if(state_sigma_vec.rows() != state_num)
            {
                std::cout << "ERROR: state num is not equal to state_sigam_vec.rows()" << std::endl;
            }

            Q_.setIdentity();
            Q_ = Q_ * noise_sigma;

            R_.setIdentity();
            R_ = R_*eval_sigma;

            P_.setZero();
            for(int i(0);i<P_.rows();++i)
            {
                for(int j(0);j<P_.cols();++j)
                {
                    P_(i,j) = state_sigma_vec(i) * state_sigma_vec(j);
                }
            }

            X_.setZero();

            A_.setZero();

            C_.setZero();

            //ToDo:For test.
            InitialMatrix();



        }

        bool InitialState(Eigen::VectorXd state_vec)
        {
            X_ = state_vec;
            return true;
        }

        bool InitialMatrix()
        {
            A_.setZero();
            A_(0,0) = 1.0;
            A_(0,1) = 0.0;
            A_(0,2) = 1.0;
            A_(0,3) = 0.0;

            A_(1,0) = 0.0;
            A_(1,1) = 1.0;
            A_(1,2) = 0.0;
            A_(1,3) = 1.0;

            A_(2,0) = 0.0;
            A_(2,1) = 0.0;
            A_(2,2) = 1.0;
            A_(2,3) = 0.0;

            A_(3,0) = 0.0;
            A_(3,1) = 0.0;
            A_(3,2) = 0.0;
            A_(3,3) = 1.0;

            C_.setZero();
            C_(0,0) = 1.0;
            C_(1,1) = 1.0;

        }

        Eigen::VectorXd OneStep(Eigen::VectorXd observe_val)
        {
            return OneStep(observe_val,1.0);
        };
        Eigen::VectorXd OneStep(Eigen::VectorXd observe_val,T time_step)
        {
            last_X_ = X_;

            //predictor
            Eigen::MatrixXd t_A(A_);
            t_A(0,2) = time_step;
            t_A(1,3) = time_step;

            X_ = (A_ ) * X_ ;
//        std::cout << X_ << std::endl;

            P_ =  ( t_A) * P_ * (t_A.transpose().eval() *time_step)+ Q_;
//        std::cout << P_ << std::endl;

            // Kalman gain
//        std::cout << C_ << std::endl;
//        std::cout << (C_ * P_ * C_.transpose().eval() + R_) << std::endl;

            K_ = P_ * C_.transpose().eval() * (C_ * P_ * C_.transpose().eval() + R_).inverse();

            //correct

            Eigen::MatrixXd I;
            I.resizeLike(K_ * C_);
            I.setIdentity();
//        std::cout << "I:" << I << std::endl;

//        std::cout << "K:" << K_ << std::endl;

            P_ = (I - K_ * C_) * P_;
            X_ = X_ + K_*(observe_val - C_ * X_);

            //numerous modified
            P_ = (P_ * 0.5  + P_.transpose().eval() * 0.5);

            //Special to this situation.
//            X_(2) = (X_(0)-last_X_(0))/time_step;
//            X_(3) = (X_(1)-last_X_(1))/time_step;


            return X_;
        };

        Eigen::VectorXd Predict(T time_step)
        {
            Eigen::MatrixXd t_A(A_);
            t_A(0,2) = time_step;
            t_A(1,3) = time_step;
            predict_x_ = (t_A ) * X_;
            std::cout << "x:" << X_.transpose() << "pre:" << predict_x_.transpose() << std::endl;
            return predict_x_;
        }

    protected:
        Eigen::Matrix<T,state_num,state_num> Q_;//process noise

        Eigen::Matrix<T,observe_num,observe_num> R_;//measurement noise


        Eigen::Matrix<T,state_num,state_num> P_; // covariance matrix to state

        Eigen::Matrix<T,state_num,1> X_; // Last state(posterior)

        Eigen::Matrix<T,state_num,state_num> A_; // state transition matrix

        Eigen::Matrix<T,observe_num,state_num> C_; // observation matrix

        Eigen::Matrix<T,state_num,observe_num> K_; // Kalman gain.

    private:
        Eigen::Matrix<T,state_num,1> predict_x_; // save state before compute prior and use to compute velocity after compute posterior.
        Eigen::Matrix<T,state_num,1> last_X_; // save state before compute prior and use to compute velocity after compute posterior.





    };
}



#endif //CVREADER_KALMANFILTER_HPP
