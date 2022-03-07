#ifndef KALMAN_TRACKING_LIDAR_HPP_
#define KALMAN_TRACKING_LIDAR_HPP_

#include <set>
#include <vector>
#include <iostream>

#include "SystemModel.hpp"
#include "PositionMeasurementModel.hpp"
#include <kalman/UnscentedKalmanFilter.hpp>

#include "BipartiteHungarian.h"

typedef struct {
    float x;        //精度 10微米
    float y;        //精度 10微米
    float z;        //精度 10微米
    int32_t intensity;    //能量值
}PV_POINT_XYZI;

//目标检测结果数据
typedef struct {
    uint index;             // 目标临时编号
    float intensity;        // 目标激光强度总和
    float width;            // 精度：0.01m
    float length;           // 精度：0.01m
    float height;           // 精度：0.01m

    float x_pos;            // 精度：0.01m
    float y_pos;            // 精度：0.01m
    float z_pos;            // 精度：0.01m

    float x_speed;            // 精度：0.01m/s
    float y_speed;            // 精度：0.01m/s
    float z_speed;            // 精度：0.01m/s

} PV_OBJ_DATA;

struct SIn {    // 输入的目标点云
    int64_t m_time_ms;             //消息时间戳
    int32_t m_obj_num;             //有效目标数量
    PV_OBJ_DATA m_obj_data[300];  //目标参数
    // int m_obj_point_count[300];   //第x个目标的点云的数量
    // PV_POINT_XYZI points[1331200];  //目标点云数据10400*8*16
};
namespace KalmanTracking
{

typedef float T;

// Some type shortcuts
typedef LidarTarget::State<T> State;
typedef LidarTarget::Control<T> Control;
typedef LidarTarget::SystemModel<T> SystemModel;

typedef LidarTarget::PositionMeasurement<T> PositionMeasurement;
typedef LidarTarget::PositionMeasurementModel<T> PositionModel;

/**
 * @brief 数据格式转换
 * 
 * @param data 自定义数据
 * @return Kalman::Vector<float, 10> Eigen的向量
 */
Kalman::Vector<float, 10> toVector(const PV_OBJ_DATA &data) {
    Kalman::Vector<float, 10> target;
    target << data.width, data.length, data.height,
              data.x_pos, data.y_pos, data.z_pos,
              data.x_speed, data.y_speed, data.z_speed,
              data.intensity;
    return target;
}

/**
 * @brief Create a Edges object
 * 
 * @param prevSet 前一次观测目标
 * @param nextSet 后一次观测目标
 * @return std::vector<WeightedBipartiteEdge> 目标之间匹配的距离
 */
std::vector<WeightedBipartiteEdge> createEdges(const std::vector<PV_OBJ_DATA> &prevSet, const std::vector<PV_OBJ_DATA> &nextSet)
{
    // 当前目标 next 与上一次目标 prev 的特征距离
    std::vector<WeightedBipartiteEdge> edges;
    for (size_t i = 0; i < prevSet.size(); ++i) {
        Kalman::Vector<float, 10> prevTarget = toVector(prevSet[i]);
        for (size_t j = 0; j < nextSet.size(); ++j) {
            PV_OBJ_DATA temp = nextSet[j];
            temp.x_speed = temp.x_pos - prevSet[i].x_pos;
            temp.y_speed = temp.y_pos - prevSet[i].y_pos;
            temp.z_speed = temp.z_pos - prevSet[i].z_pos;
            Kalman::Vector<float, 10> nextTarget = toVector(temp);
            Kalman::Vector<float, 10> delta = nextTarget - prevTarget;
            float d1 = std::sqrt( delta.dot(delta) ); //计算向量距离
            std::cout << "target distance: " << d1 << std::endl;
            // 构造所有边的权重
            if (d1 < threshold_)
                edges.push_back( WeightedBipartiteEdge(prevSet[i].index, j, d1) );
        }
    }
    return edges;
}

#define N 300
/**
 * @brief 基于Lidar的跟踪。
 * 两帧点云按照特征距离建立先后关系
 * 点云目标的ID逻辑也在其中
 * 
 */
class LidarTracking {
public:
    LidarTracking(const std::vector<PV_OBJ_DATA> &in) : prevTargets_(in) {
        // 构造里目标ID固定了，新目标要顺序编号
        threshold_ = 1.0f;
        for (int i = 0; i < N; ++i) {
            x_[i].setZero();
            u_[i].setZero();
        }
    }
    /**
     * @brief 二分图算法匹配，第三方算法。
     * 
     * @param in 当前帧的点云目标
     * @return LidarTracking 新对象包含所有的点云目标
     * 
     */
    void tracking(const std::vector<PV_OBJ_DATA> &in) {
        // 匹配最优的目标组合
        std::vector<WeightedBipartiteEdge> edges = createEdges(prevTargets_, in);
        // 二分最优匹配
        int nodes = prevTargets_.size() < in.size() ? prevTargets_.size() : in.size();
        std::vector<int> matching = bruteForce(nodes, edges);
        // 先做Kalman
        int max_id = 0;
        for (PV_OBJ_DATA &obj : prevTargets_) {
            int right = matching[obj.index];
            if (right >= 0)
                kalmanProcess(obj, in[right]);
            else
                kalmanProcess(obj);
            max_id = max_id < obj.index ? obj.index : max_id;
        }
        int right[N] = {-1};
        for (int &id : matching) {
            right[id] = id;
        }
        for (auto &obj : in) {
            if (right[obj.index] == -1 ) { //新目标
                PV_OBJ_DATA temp = obj;
                temp.index = max_id;
                prevTargets_.push_back(temp);
                max_id += 1;

                //新目标，还需要其它信息计算变化量
                x_[temp.index] << obj.x_pos, obj.y_pos, obj.z_pos,
                                obj.length, obj.width, obj.height,
                                obj.intensity;
                u_[temp.index].setZero(); // Kalman对象还没有控制变量
                ukf_[temp.index].init(x_[temp.index]);
            }
        }
    }
    /**
     * @brief 通过Kalman预测更新目标，同时预测+1
     * 
     * @param obj 目标
     */
    void kalmanProcess(PV_OBJ_DATA &obj)
    {
        x_[obj.index] = ukf_[obj.index].predict(sys_, u_[obj.index]);
        obj.x_speed = u_[obj.index].dx();
        obj.y_speed = u_[obj.index].dy();
        obj.z_speed = u_[obj.index].dz();
        obj.x_pos = x_[obj.index].x();
        obj.y_pos = x_[obj.index].y();
        obj.z_pos = x_[obj.index].z();
        predict_[obj.index] += 1;
    }
    /**
     * @brief 通过Lidar更新目标与控制，同时更新Kalman目标
     * 
     * @param left 上一次目标
     * @param right 当前目标
     */
    void kalmanProcess(PV_OBJ_DATA &left, const PV_OBJ_DATA &right)
    {
        //按照Lidar更新目标
        left.x_speed= right.x_pos - left.x_pos;
        left.y_speed= right.y_pos - left.y_pos;
        left.z_speed= right.z_pos - left.z_pos;
        left.x_pos = right.x_pos;
        left.y_pos = right.y_pos;
        left.z_pos = right.z_pos;
        left.length = right.length;
        left.width = right.width;
        left.height = right.height;
        left.intensity = right.intensity;

        //更新目标的控制
        u_[left.index].dx() = right.x_pos - left.x_pos;
        u_[left.index].dy() = right.y_pos - left.y_pos;
        u_[left.index].dz() = right.z_pos - left.z_pos;
        u_[left.index].dl() = right.length - left.length;
        u_[left.index].dw() = right.width - left.width;
        u_[left.index].dh() = right.height - left.height;
        u_[left.index].di() = right.intensity - left.intensity;

        //对应Kalman的预测与更新
        x_[left.index] = ukf_[left.index].predict(sys_, u_[left.index]);
        PositionMeasurement measure;
        measure << left.x_pos, left.y_pos, left.z_pos,
                    left.length, left.width, left.height,
                    left.intensity;
        x_[left.index] = ukf_[left.index].update(pmm_, measure);
        predict_[left.index] = 0;
    }
private:
    std::vector<PV_OBJ_DATA> prevTargets_;  //前一次观测的集合
    float threshold_;

private:
    int predict_[N];
    State x_[N];
    Control u_[N];
    Kalman::UnscentedKalmanFilter<State> ukf_[N];
    // 系统是整个场景一个系统
    SystemModel sys_;
    // Measurement models
    PositionModel pmm_;
};

} // namespace KalmanTracking

#endif
