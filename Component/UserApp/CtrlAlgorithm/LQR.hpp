/********************************************************************************
  * @file           : LQR.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-7
  *******************************************************************************/


#ifndef __LQR_HPP
#define __LQR_HPP



class LQR {
private:
    double Height{};
    double K[4];
public:
    explicit LQR(double _k[4]);
    ~LQR() = default;
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double WheelRadius = 0.022;            //m
//    static constexpr double Rated_Torque = 0.4;             //kg.cm
    static constexpr double Rated_Torque = 0.4*9.8*0.01;    //N.m
    static constexpr double Rated_Torque_PWMValue = 800;
    static constexpr double TorqueToPWM_Coefficient = (WheelRadius/(Rated_Torque * 2)) * Rated_Torque_PWMValue;

    void setHight(double _highvalue);
    double Calculate_LQR(const double &angle, const double &angularspeed, const double &position, const double &velocity);
};


#endif //__LQR_HPP
