#ifndef KALMAN_TRACKING_LIDAR_HPP_
#define KALMAN_TRACKING_LIDAR_HPP_

#include <set>
#include <vector>
#include <iostream>

#include "SystemModel.hpp"
#include "PositionMeasurementModel.hpp"
#include <kalman/UnscentedKalmanFilter.hpp>

#include "BipartiteHungarian.h"
#include <QDebug>
typedef struct {
    float x;        //精度 10微米
    float y;        //精度 10微米
    float z;        //精度 10微米
    int32_t intensity;    //能量值
}PV_POINT_XYZI;

//目标检测结果数据
typedef struct {
    uint index;             // 目标临时编号
    char redis_key[40];     // UUID
    int intensity;        // 目标激光强度总和
    float width;            // 精度：0.01m
    float length;           // 精度：0.01m
    float height;           // 精度：0.01m

    float x_pos;            // 精度：0.01m
    float y_pos;            // 精度：0.01m
    float z_pos;            // 精度：0.01m

    float x_speed;            // 精度：0.01m/s
    float y_speed;            // 精度：0.01m/s
    float z_speed;            // 精度：0.01m/s

    int track_times{0};           // 目标的预测次数
} PV_OBJ_DATA;

struct SIn {    // 输入的目标点云
    int64_t m_time_ms;             //消息时间戳
    int32_t m_obj_num;             //有效目标数量
    PV_OBJ_DATA m_obj_data[300];  //目标参数
    // int m_obj_point_count[300];   //第x个目标的点云的数量
    // PV_POINT_XYZI points[1331200];  //目标点云数据10400*8*16
};
struct SOut {
    int64_t m_time_ms;             //消息时间戳
    int32_t m_obj_num;             //有效目标数量
    PV_OBJ_DATA m_obj_data[300];  //目标参数
//    int m_obj_point_count[300];   //第x个目标的点云的数量
//    PV_POINT_XYZI points[1331200];  //目标点云数据10400*8*16
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
 * @brief 基于Lidar的跟踪。
 * 两帧点云按照特征距离建立先后关系
 * 点云目标的ID逻辑也在其中
 *
 */
class LidarTracking {
    const static int N = 300;
    static int frames;
public:
    LidarTracking(const std::vector<PV_OBJ_DATA> &in, float threshold) : prevTargets_(in), predicts_(N, 0) {
        frames = 0;
        // 构造里目标ID固定了，新目标要顺序编号
        threshold_ = threshold;
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
    virtual void tracking(const std::vector<PV_OBJ_DATA> &in) {
        frames += 1;
        // 匹配最优的目标组合
        std::vector<WeightedBipartiteEdge> edges = createEdges(prevTargets_, in);
        // 二分最优匹配
        // TODO 当右少左多时表示有目标丢失，左边的哪个该丢弃？？？
        int nodes = prevTargets_.size() < in.size() ? prevTargets_.size() : in.size();
        std::vector<int> matching = bruteForce(nodes, edges);
        // 先做Kalman
        for (PV_OBJ_DATA &obj : prevTargets_) {
            int right = matching[obj.index];
            if (right >= 0) {
                kalmanProcess(obj, in[right]);
                memcpy(obj.redis_key, in[right].redis_key, 40);
                dumpObj(obj, "Old-track");
                dumpObj(in[right], "New-track");
            }
            else {
                kalmanProcess(obj);
                dumpObj(obj, "Old-pred");
            }
        }
        std::vector<int> right(N, -1);
        for (int &id : matching) {
            if (id != -1)
                right[id] = id;
        }
        // for (auto &obj : in) {
        for (size_t i = 0; i < in.size(); ++i) {
            // if (right[obj.index] == -1 ) { //新目标
            if (right[i] == -1 ) { //新目标
                PV_OBJ_DATA temp = in[i]; //obj;
                temp.index = slotForNewKalman();
                if (temp.index < 0) continue;   //没有空位则丢弃目标
                temp.track_times = 0;  // 新目标没有追踪
                prevTargets_.push_back(temp);
                dumpObj(temp, "New-first");

                //新目标，还需要其它信息计算变化量
                x_[temp.index] << temp.x_pos, temp.y_pos, temp.z_pos,
                                temp.length, temp.width, temp.height,
                                temp.intensity;
                u_[temp.index].setZero(); // Kalman对象还没有控制变量
                ukf_[temp.index].init(x_[temp.index]);
            }
        }
        prevTargets_.erase(std::remove_if(prevTargets_.begin(), prevTargets_.end(), removeKalman(&predicts_)), prevTargets_.end());
        std::sort(prevTargets_.begin(), prevTargets_.end(), [](const PV_OBJ_DATA &a, const PV_OBJ_DATA &b) { return a.index < b.index; });
    }
    void output(PV_OBJ_DATA pOut[], uint &size)
    {
        size = 0;
        for (PV_OBJ_DATA &data : prevTargets_) {
            pOut[size] = data;
            size += 1;
            if (size > N) {
                break;
            }
        }
    }
private:
    void dumpObj(const PV_OBJ_DATA &obj, char type[]) 
    {
        qDebug(",%d,%s,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d",frames,type, obj.index,obj.x_pos,obj.y_pos,obj.z_pos,obj.x_speed,obj.y_speed,obj.z_speed,obj.length,obj.width,obj.height,obj.intensity);
    }
    /**
     * @brief 在0~300的序号选择一个没有使用的
     * 
     * @return int 选中一个。-1 表示没有空位
     */
    int slotForNewKalman()
    {
        std::vector<int> slots(N, 0);  // N 个位子
        // 初始化位子
        for (PV_OBJ_DATA &obj : prevTargets_) {
            assert(obj.index < N);   // 绝对不会超过 N 个目标
            slots[obj.index] = obj.index;  // 占位子
        }
        // 挑选空位子
        for (int i = 0; i < N; ++i) {
            if (slots[i] == 0)
                return i;
        }
        return -1; // 没有空位
    }
    /**
     * @brief Function Object for 清除队列中无效的Kalman对象
     * 
     */
    struct removeKalman {
        removeKalman(std::vector<int> *pred) {
            preds = pred;
        }
        bool operator() (const PV_OBJ_DATA &data)
        {
            if ((*preds)[data.index] > 10) {
                (*preds)[data.index] = 0;
                return true;
            }
            return false;
        }
        std::vector<int> *preds;
    };
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
        predicts_[obj.index] += 1;
        obj.track_times -= 1;  // 雷达目标丢失，重新才跟踪
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
        left.x_speed = (left.x_speed + right.x_pos - left.x_pos) / 2;
        left.y_speed = (left.y_speed + right.y_pos - left.y_pos) / 2;
        left.z_speed = (left.z_speed + right.z_pos - left.z_pos) / 2;
        left.x_pos = right.x_pos;
        left.y_pos = right.y_pos;
        left.z_pos = right.z_pos;
        left.length = (left.length + right.length) / 2;
        left.width = (left.width + right.width) / 2;
        left.height = (left.height + right.height) / 2;
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
        predicts_[left.index] = 0;
        left.track_times += 1;
    }
private:
    std::vector<PV_OBJ_DATA> prevTargets_;  //前一次观测的集合
    float threshold_;

private:
    std::vector<int> predicts_;  // 纯粹的Kalman预测
    State x_[N];
    Control u_[N];
    Kalman::UnscentedKalmanFilter<State> ukf_[N];
    // 系统是整个场景一个系统
    SystemModel sys_;
    // Measurement models
    PositionModel pmm_;

private:
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
        for (size_t j = 0; j < nextSet.size(); ++j) {
            float d1 = eucDistance(prevSet[i], nextSet[j]); //std::sqrt( delta.dot(delta) ); //计算向量距离
            //qDebug(",%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f",j,nextSet[j].x_pos,nextSet[j].y_pos,nextSet[j].z_pos,nextSet[j].x_pos-prevSet[i].x_pos,nextSet[j].y_pos-prevSet[i].y_pos,nextSet[j].z_pos-prevSet[i].z_pos,nextSet[j].length,nextSet[j].width,nextSet[j].height,nextSet[j].intensity, d1);
            //qInfo("%f",d1);
            dumpObj(nextSet[j], "New-edges");
            // 构造所有边的权重
            if (d1 < threshold_)  // 阈值过滤
                edges.push_back( WeightedBipartiteEdge(prevSet[i].index, j, d1) );
        }
    }
    return edges;
}
#define DIMS 3
/**
 * @brief 数据格式转换
 *
 * @param data 自定义数据
 * @return Kalman::Vector<float, 10> Eigen的向量
 */
Kalman::Vector<float, DIMS> toVector(const PV_OBJ_DATA &data) {
    Kalman::Vector<float, DIMS> target;
    target << //data.width, data.length, data.height,
              data.x_pos, data.y_pos, data.z_pos;
              //data.x_speed, data.y_speed, data.z_speed,
              //data.intensity;
    return target;
}

float eucDistance(const PV_OBJ_DATA &left, const PV_OBJ_DATA &right)
{
    // 欧氏距离
    Kalman::Vector<float, DIMS> prevTarget = toVector(left);
    PV_OBJ_DATA temp = right;
    temp.x_speed = temp.x_pos - left.x_pos;
    temp.y_speed = temp.y_pos - left.y_pos;
    temp.z_speed = temp.z_pos - left.z_pos;
    Kalman::Vector<float, DIMS> nextTarget = toVector(temp);
    Kalman::Vector<float, DIMS> delta = nextTarget - prevTarget;
    float d1 = std::sqrt( delta.dot(delta) ); //计算向量距离
    return d1;
}

float mahDistance(const PV_OBJ_DATA &left, const PV_OBJ_DATA &right)
{
    Kalman::Covariance<Kalman::Vector<float, 10>> sigma;
    sigma.setIdentity();
    // 马氏距离
    // 有N个观测样本了，求样本向量的协方差矩阵
    // 最新样本与当前样本的目标求马氏距离
    // 最新样本加入样本集重新计算协方差矩阵？有简单方法吗？
    // 每个目标都加入样本矩阵（不可能无限制）
    return 0.0f;
}
};

class NoneTracking : public LidarTracking
{
    const static int N = 300;
  public:
    NoneTracking(const std::vector<PV_OBJ_DATA> &in, float threshold) : LidarTracking(in, threshold), prevTargets_(in)
    {
        (void)threshold;
    }
    void tracking(const std::vector<PV_OBJ_DATA> &in)
    {
        prevTargets_ = in;
    }
    void output(PV_OBJ_DATA pOut[], uint &size)
    {
        size = 0;
        for (PV_OBJ_DATA &data : prevTargets_) {
            data.track_times = 20;
            pOut[size] = data;
            size += 1;
            if (size > N) {
                break;
            }
        }
    }
  private:
    std::vector<PV_OBJ_DATA> prevTargets_;  //前一次观测的集合

};
} // namespace KalmanTracking

#endif
