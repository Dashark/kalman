
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>

#include "Tracking.hpp"
#include "SystemModel.hpp"
#include "PositionMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>


using namespace KalmanTracking;

typedef float T;

// Some type shortcuts
typedef LidarTarget::State<T> State;
typedef LidarTarget::Control<T> Control;
typedef LidarTarget::SystemModel<T> SystemModel;

typedef LidarTarget::PositionMeasurement<T> PositionMeasurement;
typedef LidarTarget::PositionMeasurementModel<T> PositionModel;

int main(int argc, char** argv)
{
    //一些数据结构转换
    SIn pIn;
    pIn.m_obj_num = 1;
    pIn.m_obj_data[0].height = 0.0f;
    pIn.m_obj_data[0].intensity = 0.0f;
    pIn.m_obj_data[0].length = 0.0f;
    pIn.m_obj_data[0].width = 0.0f;
    pIn.m_obj_data[0].x_pos = 0.0f;
    pIn.m_obj_data[0].y_pos = 0.0f;
    pIn.m_obj_data[0].z_pos = 0.0f;
    pIn.m_obj_data[0].x_speed = 0.0f;
    pIn.m_obj_data[0].y_speed = 0.0f;
    pIn.m_obj_data[0].z_speed = 0.0f;
    pIn.m_obj_data[0].index = 0;
    SIn sIn;
    sIn.m_obj_num = 1;
    sIn.m_obj_data[0].height = 1.0f;
    sIn.m_obj_data[0].intensity = 1.0f;
    sIn.m_obj_data[0].length = 1.0f;
    sIn.m_obj_data[0].width = 1.0f;
    sIn.m_obj_data[0].x_pos = 1.0f;
    sIn.m_obj_data[0].y_pos = 1.0f;
    sIn.m_obj_data[0].z_pos = 1.0f;
    sIn.m_obj_data[0].x_speed = 1.0f;
    sIn.m_obj_data[0].y_speed = 1.0f;
    sIn.m_obj_data[0].z_speed = 1.0f;
    sIn.m_obj_data[0].index = 1;
    std::vector<PV_OBJ_DATA> sinSet, pinSet;
    sinSet.insert(sinSet.end(), sIn.m_obj_data, sIn.m_obj_data+sIn.m_obj_num);
    pinSet.insert(pinSet.end(), pIn.m_obj_data, pIn.m_obj_data+pIn.m_obj_num);
    LidarTracking lidarT(pinSet);
    lidarT.bipartite(sinSet);
    
    return 0;
}
