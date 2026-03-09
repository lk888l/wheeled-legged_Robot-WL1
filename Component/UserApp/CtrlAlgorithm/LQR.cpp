/********************************************************************************
  * @file           : LQR.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-7
  *******************************************************************************/


#include "LQR.hpp"

LQR::LQR(double *_k)
{
    K[0] = _k[0];
    K[1] = _k[1];
    K[2] = _k[2];
    K[3] = _k[3];
}

void LQR::setHight(double _highvalue) {

}

/**
 * @brief
 * @param angle
 * @param angularspeed
 * @param position
 * @param velocity
 * @return
 */
double LQR::Calculate_LQR(const double &angle, const double &angularspeed, const double &position, const double &velocity) {
    return (-(K[0]*angle + K[1]*angularspeed + K[2]*position + K[3]*velocity));
}


