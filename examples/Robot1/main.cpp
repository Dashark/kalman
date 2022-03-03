
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>


#include "SystemModel.hpp"
#include "OrientationMeasurementModel.hpp"
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
    LidarTracking lidarT(first cloud);
    LidarTracking lidarNew = lidarT.bipartite(second cloud);
    std::vector<PV_OBJ_DATA> news = lidarT.getMatch();
    std::vector<PV_OBJ_DATA> losts = lidarT.getLosting();
    std::vector<int> appears = lidarT.getAppears();
    std::vector<PV_OBJ_DATA> appears = lidarT.newKalmanObj();
    //返回点云数据结构
    // Simulated (true) system state
    // 目标进场时的状态(X, Y, Z, L, W, H, I)
    State x;
    x.setZero();
    
    // Control input
    // 控制量在进场时才应该是0
    Control u;
    u.setZero();
    // System
    // 系统是整个场景一个系统
    SystemModel sys;
    
    // Measurement models
    // Set position landmarks at (-10, -10) and (30, 75)
    PositionModel pm(-10, -10, 30, 75);
    // OrientationModel om; 没有方位模型
    
    // Random number generation (for noise simulation)
    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    std::normal_distribution<T> noise(0, 1);
    
    // Some filters for estimation
    // Pure predictor without measurement updates
    // Kalman::ExtendedKalmanFilter<State> predictor;  不会使用
    // Extended Kalman Filter
    // Kalman::ExtendedKalmanFilter<State> ekf;
    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf(1);
    
    // Init filters with true system state
    // predictor.init(x);
    // ekf.init(x);
    ukf.init(x);
    
    // Standard-Deviation of noise added to all state vector components during state transition
    T systemNoise = 0.1;
    // Standard-Deviation of noise added to all measurement vector components in orientation measurements
    // Standard-Deviation of noise added to all measurement vector components in distance measurements
    T distanceNoise = 0.25;
    
    // Simulate for 100 steps
    const size_t N = 100;
    for(size_t i = 1; i <= N; i++)
    {
        
        // Simulate system
        x = sys.f(x, u);
        
        // Add noise: Our robot move is affected by noise (due to actuator failures)
        // 已经是下一帧的状态了
        x.x() += systemNoise*noise(generator);
        x.y() += systemNoise*noise(generator);
        
        // Predict state for current time-step using the filters
        // auto x_pred = predictor.predict(sys, u);
        // auto x_ekf = ekf.predict(sys, u);
        auto x_ukf = ukf.predict(sys, u); //预测了下一帧的状态
        
        // Position measurement
        {
            // Lidar结果就是观测状态，它和目标状态对应的，不需要做进一步转换
            // We can measure the position every 10th step
            // 下一帧的状态转换成观测状态
            // Lidar数据直接转成观测状态，h()还是需要将目标状态转换成观测状态（内部使用）
            PositionMeasurement position = pm.h(x);
            
            // Measurement is affected by noise as well
            position.d1() += distanceNoise * noise(generator);
            position.d2() += distanceNoise * noise(generator);
            
            // Update EKF
            // x_ekf = ekf.update(pm, position);
            
            // Update UKF
            // 观测状态对照的是预测后的状态了
            x_ukf = ukf.update(pm, position);
        }
        
        // Print to stdout as csv format
        /*
        std::cout   << x.x() << "," << x.y() << "," << x.theta() << ","
                    << x_pred.x() << "," << x_pred.y() << "," << x_pred.theta()  << ","
                    << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta()  << ","
                    << x_ukf.x() << "," << x_ukf.y() << "," << x_ukf.theta()
                    << std::endl;
        */
    }
    
    return 0;
}
