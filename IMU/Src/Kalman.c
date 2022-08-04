/*
 * Kalman.c
 *
 *  Created on: 2 août 2022
 *      Author: EXtrA
 */


#include "Inc/Kalman.h"


        void KalmanFilterACC(float buffer[])
        {
          static const float R = 400 ; //  mesurement uncertainty
          static const float H = 1 ; // observability matrix
          static float Q = 10 ; // process noise (estimated covariance)
          static float P = 0 ; // initial uncertainty
          static float U_est_x = 0 ;
          static float U_est_y = 0 ;
          static float U_est_z = 0 ;  //initial estimation
          static float K = 0 ; // initial Kalman gain

            K=(P*H)/(H*P*H+R) ; //Calculation of KALMAN gain
            U_est_x = U_est_x+K*(buffer[0]-H*U_est_x) ; //estimation
            U_est_y = U_est_y+K*(buffer[1]-H*U_est_y) ;
            U_est_z = U_est_z+K*(buffer[2]-H*U_est_z) ;
          P=(1-K*H)*P+Q ; //uncertainty

          buffer[0] = U_est_x ;
          buffer[1] = U_est_y ;
          buffer[2] = U_est_z ;
        }

        /*********************************Kalman Filter Magnetometer ************************************/
        void KalmanFilterMAG(float buffer[])
        {
          static const float R = 400 ; //  mesurement uncertainty
          static const float H = 1 ; // observability matrix
          static float Q = 10 ; // process noise (estimated covariance)
          static float P = 0 ; // initial uncertainty
          static float U_est_x = 0 ;
          static float U_est_y = 0 ;
          static float U_est_z = 0 ;  //initial estimation
          static float K = 0 ; // initial Kalman gain

            K=(P*H)/(H*P*H+R) ; //Calculation of KALMAN gain
            U_est_x = U_est_x+K*(buffer[0]-H*U_est_x) ; //estimation
            U_est_y = U_est_y+K*(buffer[1]-H*U_est_y) ;
            U_est_z = U_est_z+K*(buffer[2]-H*U_est_z) ;
          P=(1-K*H)*P+Q ; //uncertainty

          buffer[0] = U_est_x ;
          buffer[1] = U_est_y ;
          buffer[2] = U_est_z ;
        }
