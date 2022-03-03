#ifndef KALMAN_TRACKING_LIDAR_HPP_
#define KALMAN_TRACKING_LIDAR_HPP_

#include <set>
#include <vector>
#include <kalman/UnscentedKalmanFilter.hpp>

#include "BipartiteHungarian.h"

namespace KalmanTracking
{
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
    int m_obj_point_count[300];   //第x个目标的点云的数量
    PV_POINT_XYZI points[1331200];  //目标点云数据10400*8*16
};

using namespace KalmanTracking;

typedef float T;

// Some type shortcuts
typedef LidarTarget::State<T> State;
typedef LidarTarget::Control<T> Control;
typedef LidarTarget::SystemModel<T> SystemModel;

typedef LidarTarget::PositionMeasurement<T> PositionMeasurement;
typedef LidarTarget::PositionMeasurementModel<T> PositionModel;

struct KalmanObj
{
    KalmanObj():ukf(1) {
        predict_num = 0;
        x.setZero();
        u.setZero();
    }
    int predict_num;
    State x;
    Control u;
    Kalman::UnscentedKalmanFilter<State> ukf;
};

struct searchEdge {
    int left, right;
    searchEdge(int l, int r) {left = l; right = r;}
    bool operator() (WeightedBipartiteEdge &edge) {
        return (edge.left == left && edge.right == right);
    }
};

Kalman::Vector<float, 10> targetVector(const PV_OBJ_DATA &data) {
    Kalman::Vector<float, 10> target;
    target << data.width, data.length, data.height,
              data.x_pos, data.y_pos, data.z_pos,
              data.x_speed, data.y_speed, data.z_speed,
              data.intensity;
    return target;
}
std::vector<WeightedBipartiteEdge> createEdges(std::vector<PV_OBJ_DATA> prevSet, std::vector<PV_OBJ_DATA> nextSet) {
    // 当前目标 next 与上一次目标 prev 匹配关系
    std::vector<WeightedBipartiteEdge> edges;
    for (auto &prev : prevSet) {
        Kalman::Vector<float, 10> prevTarget = targetVector(prev);
        for (auto &next : nextSet) {
            Kalman::Vector<float, 10> nextTarget = targetVector(next);
            Kalman::Vector<float, 10> delta = nextTarget - prevTarget;
            float d1 = std::sqrt( delta.dot(delta) ); //计算向量距离
            // 构造所有边的权重
            edges.push_back( WeightedBipartiteEdge(prev.index, next.index, d1) );
        }
    }
    return edges;
}

bool ProcessData() //const QByteArray &in, QByteArray &out)
{
    SIn* pIn; // = (SIn*)(in.data());
    std::vector<PV_OBJ_DATA> inSet;
    // 加入Set并按照index排序
    inSet.insert(inSet.end(), pIn->m_obj_data, pIn->m_obj_data+pIn->m_obj_num);
    std::vector<PV_OBJ_DATA> prevSet;
    std::vector<WeightedBipartiteEdge> edges = createEdges(prevSet, inSet);
    std::vector<int> matching = hungarianMinimumWeightPerfectMatching(prevSet.size(), edges);
    // TODO 左边与右边数量不一致会怎样？
    // TODO 左边多个节点会连接到右边一个节点吗？
    // 还要剔除距离明显过大的匹配，从而得到未成功匹配的目标
    int i = 0;
    float threshold = 0.0f; // 多少合适呢？
    std::vector<int> left_match, left_unmatch, right_match, right_unmatch;
    std::vector<std::pair<int, int>> lr_match;
    for (int &id : matching) {
        std::vector<WeightedBipartiteEdge>::iterator it = std::find_if(edges.begin(), edges.end(), searchEdge(i, id)); 
        if (it != edges.end() ) {
            // 匹配的边权重不应该太大，但是现在也不清楚具体是多少
            if (it->cost < threshold) {
                lr_match.push_back(std::make_pair(it->left, it->right));
            }
            else {
                left_unmatch.push_back(it->left);
                right_unmatch.push_back(it->right);
            }
            // 这儿match和unmatch有可能掉了成员
        }
        i += 1;
    }
    /*
    std::map<int, KalmanObj> kalman_map;
    // 处理完成得到3个集合，left, right, 和上面的matching
    // 对matching做Kalman预测与更新
    for (std::pair &pa : lr_match) {
        KalmanObj &obj = kalman_map.find(pa.first);
        obj.x = obj.ukf.predict(sys, obj.u);
        PV_OBJ_DATA pvData = inSet.find(pa.second);  //新输入点云数据
        // 转换观测状态
        PositionMeasurement px = convert(pvData);
        obj.x = obj.ukf.update(pm, px);
        // TODO 返回上层结果
    }
    // 对left做Kalman预测
    for (int &id : left_unmatch) {
        KalmanObj &obj = kalman_map.find(pa.first);
        obj.x = obj.ukf.predict(sys, obj.u);
        obj.predict_num += 1;
        // TODO 返回上层结果
    }
    // 对right做新建目标
    for (int &id : right_unmatch) {
        kalman_map.insert(std::make_pair(id, KalmanObj));
    }
    State x;
    x << prev.width...;
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
    Kalman::UnscentedKalmanFilter<State> ukf(1);
    ukf.init(x);
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
*/
	return true;
}

} // namespace KalmanTracking

#endif
