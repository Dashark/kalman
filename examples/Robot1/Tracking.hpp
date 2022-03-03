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

/**
 * @brief 基于Lidar的跟踪。
 * 两帧点云按照特征距离建立先后关系
 * 点云目标的ID逻辑也在其中
 * 
 */
class LidarTracking {
public:
    LidarTracking(const std::vector<PV_OBJ_DATA> &in) : prevTargets_(in), matching_(0), left_unmatch_(0), right_unmatch_(0) {}
    /**
     * @brief 二分图算法匹配，第三方算法
     * 
     * @param in 当前帧的点云目标
     * @return LidarTracking 新对象包含所有的点云目标
     * 匹配后产生新对象，同时不破坏老对象
     */
    LidarTracking bipartite(const std::vector<PV_OBJ_DATA> &in) {
        std::vector<WeightedBipartiteEdge> edges = createEdges(prevTargets_, in);
        std::vector<int> matching = hungarianMinimumWeightPerfectMatching(prevTargets_.size(), edges);
    // 还要剔除距离明显过大的匹配，从而得到未成功匹配的目标
    int i = 0;
    float threshold = 0.0f; // 多少合适呢？
    std::vector<std::pair<int, int>> lr_match;
    for (int &id : matching) {
        std::vector<WeightedBipartiteEdge>::iterator it = std::find_if(edges.begin(), edges.end(), searchEdge(i, id)); 
        if (it != edges.end() ) {
            // 匹配的边权重不应该太大，但是现在也不清楚具体是多少
            if (it->cost < threshold) {
                lr_match.push_back(std::make_pair(it->left, it->right));
            }
            else {
                left_unmatch_.push_back(it->left);
                right_unmatch_.push_back(it->right);
            }
            // 这儿match和unmatch有可能掉了成员
        }
        i += 1;
    }
        std::vector<PV_OBJ_DATA> temp;
        for (int &id : left_unmatch_) {
            std::vector<PV_OBJ_DATA>::iterator it = std::find_if(prevTargets_.begin(), prevTargets_.end(), searchPV);
            if (it != prevTargets_.end() ) {
                temp.push_back(*it);
            }
            else
                assert(false);
        }
        for (int &id : right_unmatch_) {
            std::vector<PV_OBJ_DATA>::iterator it = std::find_if(in.begin(), in.end(), searchPV);
            if (it != in.end() ) {
                temp.push_back(*it);
            }
            else
                assert(false);
        }
        for (std::pair &pid : lr_match)
    }
    std::vector<int> getMatching() const { return matching_; }
    std::vector<int> getLosting() const { return left_unmatch_; }
    std::vector<int> getAppears() const { return right_unmatch_; }
private:
    std::vector<PV_OBJ_DATA> prevTargets_;  //前一次观测的集合
    std::vector<int> matching_, left_unmatch_, right_unmatch_;
};
/**
 * @brief 管理容器，所有Kalman对象都预测一次，集合的代理对象
 * Kalman是预测和修正后的结果，与Lidar观测还不是一个东西
 * 
 */
class TrackingApp {
public:
    TrackingApp() {}
    bool bipartite() {} // 二分图匹配
    bool predict() {}   //所有目标可以先行预测
    bool update() {
        观测对象的bipartite();
    }
    bool kalmanObj() {}
private:
private:
    // Kalman只是概念对象，映射到实际目标
    // 通过ID号
    std::map<int, KalmanObj> kalman_map;  //Kalman对象集合
    // System
    // 系统是整个场景一个系统
    SystemModel sys;
    // Measurement models
    // Set position landmarks at (-10, -10) and (30, 75)
    PositionModel pm(-10, -10, 30, 75);
};
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
    inSet.insert(inSet.end(), pIn->m_obj_data, pIn->m_obj_data+pIn->m_obj_num);
    // TODO 左边与右边数量不一致会怎样？
    // TODO 左边多个节点会连接到右边一个节点吗？
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
