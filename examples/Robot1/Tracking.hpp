#ifndef KALMAN_TRACKING_LIDAR_HPP_
#define KALMAN_TRACKING_LIDAR_HPP_

#include <set>
#include <vector>
#include <iostream>
#include <cmath>
#include <random>

#include "SystemModel.hpp"
#include "PositionMeasurementModel.hpp"
#include <kalman/UnscentedKalmanFilter.hpp>

#include "BipartiteHungarian.h"
#include <QDebug>
#include <QDateTime>

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

    float pt1_x;
    float pt1_y;
    float pt2_x;
    float pt2_y;
    float pt3_x;
    float pt3_y;
    float pt4_x;
    float pt4_y;

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
    float max_speed{0};       //最大速度
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

// 方差列表
typedef struct _VAR_PARAMS {
    float first_distance, sec_distance, cos_distance;
    float var_pos_x;
    float var_pos_y;
    float var_pos_z;
    float var_vel_x;
    float var_vel_y;
    float var_vel_z;
    float var_acc_x;
    float var_acc_y;
    float var_acc_z;
    float var_heading;
    float var_heading_rate;
    _VAR_PARAMS() {
        first_distance = 5.0f;
        sec_distance = 5.0f;
        cos_distance = 3.3f;
        var_pos_x = sqrt(0.1f);
        var_pos_y = sqrt(0.1f);
        var_pos_z = 0.0f;
        var_vel_x = sqrt(10.0f);
        var_vel_y = sqrt(10.0f);
        var_vel_z = 0.0f;
        var_acc_x = sqrt(0.1f);
        var_acc_y = sqrt(0.1f);
        var_acc_z = 0.0f;
        var_heading = sqrt(2.46f);
        var_heading_rate = sqrt(0.1f);
    }
    _VAR_PARAMS(const _VAR_PARAMS &pa) {
        first_distance = pa.first_distance;
        sec_distance = pa.sec_distance;
        cos_distance = pa.cos_distance;
        var_pos_x = sqrt(pa.var_pos_x);
        var_pos_y = sqrt(pa.var_pos_y);
        var_pos_z = 0.0f;
        var_vel_x = sqrt(pa.var_vel_x);
        var_vel_y = sqrt(pa.var_vel_y);
        var_vel_z = 0.0f;
        var_acc_x = sqrt(pa.var_acc_x);
        var_acc_y = sqrt(pa.var_acc_y);
        var_acc_z = 0.0f;
        var_heading = sqrt(pa.var_heading);
        var_heading_rate = sqrt(pa.var_heading_rate);
    }
    void modify(const _VAR_PARAMS &pa) {
        first_distance = pa.first_distance;
        sec_distance = pa.sec_distance;
        cos_distance = pa.cos_distance;
        var_pos_x = sqrt(pa.var_pos_x);
        var_pos_y = sqrt(pa.var_pos_y);
        var_pos_z = 0.0f;
        var_vel_x = sqrt(pa.var_vel_x);
        var_vel_y = sqrt(pa.var_vel_y);
        var_vel_z = 0.0f;
        var_acc_x = sqrt(pa.var_acc_x);
        var_acc_y = sqrt(pa.var_acc_y);
        var_acc_z = 0.0f;
        var_heading = sqrt(pa.var_heading);
        var_heading_rate = sqrt(pa.var_heading_rate);
    }
} VAR_PARAMS;

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
    LidarTracking(const std::vector<PV_OBJ_DATA> &in, float threshold)
        : prevTargets_(in), predicts_(N, 0), spots_(N, 0),
         variance_(-1.0, 1.0), var_params_() {
        assert(!prevTargets_.empty());
        // 构造里目标ID固定了，新目标要顺序编号
        int i = 0;
        for (PV_OBJ_DATA &data : prevTargets_) {
            data.index = i ++;
            spots_[data.index] = data.index;
        }
        threshold_ = threshold;
        for (int i = 0; i < N; ++i) {
            x_[i].setZero();
            u_[i].setZero();
        }

        generator_.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    }
    LidarTracking(const std::vector<PV_OBJ_DATA> &in, VAR_PARAMS &pa)
        : prevTargets_(in), predicts_(N, 0), spots_(N, 0),
         variance_(-1.0, 1.0), var_params_(pa) {
        assert(!prevTargets_.empty());
        // 构造里目标ID固定了，新目标要顺序编号
        int i = 0;
        for (PV_OBJ_DATA &data : prevTargets_) {
            data.index = i ++;
            spots_[data.index] = data.index;
        }
        threshold_ = pa.first_distance;
        for (int i = 0; i < N; ++i) {
            x_[i].setZero();
            u_[i].setZero();
        }

        generator_.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    }
    virtual ~LidarTracking(){}
    void modifyParams(const VAR_PARAMS &pa) {
        var_params_.modify(pa);
    }
    /**
     * @brief 二分图算法匹配，第三方算法。
     *
     * @param in 当前帧的点云目标
     * @return LidarTracking 新对象包含所有的点云目标
     *
     */
    virtual void tracking(const std::vector<PV_OBJ_DATA> &in) {
        assert(!in.empty());
        frames += 1;
        qint64 edges_t1 = QDateTime::currentMSecsSinceEpoch(); //TEST***
        std::vector<PV_OBJ_DATA> predictTargets;
        for(PV_OBJ_DATA &obj : prevTargets_) {
            predictTargets.push_back(kalmanPredict(obj));
        }
        // 匹配最优的目标组合
        std::vector<WeightedBipartiteEdge> edges = createEdges(predictTargets, in);
        // 二分最优匹配
        int nodes = prevTargets_.size() < in.size() ? in.size() : prevTargets_.size();
        std::vector<int> matching = hungarianMinimumWeightPerfectMatching(nodes, edges);
        qint64 edges_t2 = QDateTime::currentMSecsSinceEpoch(); //TEST***
        qDebug() << "nodes : " << nodes;
        qDebug() << "edges elapsed time:" << edges_t2 - edges_t1; //TEST***

        //通过共享内存传递中间数据。
        //明确输入输出
        //二分输入：聚类结果
        //
        //二分输出：

        qint64 kalman_t1 = QDateTime::currentMSecsSinceEpoch(); //TEST***
        size_t left_size = prevTargets_.size();
        if (matching.size() < left_size) return; // add for crash
        for (size_t i = 0; i < left_size; ++i) {
            int right = matching[i];
            auto eit = std::find_if(edges.begin(), edges.end(), findEdges(i, right));
            assert(eit != edges.end());
            if (predictTargets[i].track_times == 0 
                && eit->cost < var_params_.first_distance
                && eit->cost > 0.2) {
                // 新目标的跟踪
                kalmanFirst(prevTargets_[i], in[right]);
                memcpy(prevTargets_[i].redis_key, in[right].redis_key, 40);
                dumpObj(prevTargets_[i], "Old-track");
                dumpObj(in[right], "New-track");
                kalman_elapsed_update_ += 1;
            }
            else if (predictTargets[i].track_times != 0 && eit->cost < var_params_.sec_distance) {
                // 旧目标经过预测后配上观测目标
                PV_OBJ_DATA temp = in[right];
                temp.x_speed = in[right].x_pos - prevTargets_[i].x_pos + 0.0001;
                temp.y_speed = in[right].y_pos - prevTargets_[i].y_pos + 0.0001;
                float dist = cosDistance(predictTargets[i], temp);  //观测 vs. 预测 = 余弦距离
                if (dist < var_params_.cos_distance) {
                    // Kalman update
                    updateControl(prevTargets_[i], in[right]);  //观测 vs. 目标
                    kalmanUpdate(prevTargets_[i], in[right]);
                }
                else {
                    // 会奇怪不能满足条件的
                    kalmanProcess(prevTargets_[i], predictTargets[i]);
                    dumpObj(prevTargets_[i], "Old-pred");
                    if (right < in.size()) {  // 不符合条件，成为新目标
                        newTarget(in[right]);
                    }
                    kalman_elapsed_predict_ += 1;
                }
            }
            else {
                kalmanProcess(prevTargets_[i], predictTargets[i]);
                dumpObj(prevTargets_[i], "Old-pred");
                if (right < in.size()) {  // 不符合条件，成为新目标
                    newTarget(in[right]);
                }
                kalman_elapsed_predict_ += 1;
            }
        }
        qint64 kalman_t2 = QDateTime::currentMSecsSinceEpoch(); //TEST***
        qDebug() << "kalman elapsed time:" << kalman_t2 - kalman_t1; //TEST***
        //TEST***
        kalmanElapsedTime();
        qint64 erase_t1 = QDateTime::currentMSecsSinceEpoch(); //TEST***
        prevTargets_.erase(std::remove_if(prevTargets_.begin(), prevTargets_.end(), removeKalman(&predicts_)), prevTargets_.end());
        std::sort(prevTargets_.begin(), prevTargets_.end(), [](const PV_OBJ_DATA &a, const PV_OBJ_DATA &b) { return a.index < b.index; });
        for (auto &tar : prevTargets_)
            dumpObj(tar, "Results");
        qint64 erase_t2 = QDateTime::currentMSecsSinceEpoch(); //TEST***
        qDebug() << "erase elapsed time:" << kalman_t2 - kalman_t1; //TEST***
    }
    void output(PV_OBJ_DATA pOut[], uint &size)
    {
        size = 0;
        for (PV_OBJ_DATA &data : prevTargets_) {
            // data.index 要递增，递增的结果要保留，然后要替换
            // spots_中记录和查找最大值，*std::max_element()
            // 预测的结果不返回了
            if (data.track_times < 2 || predicts_[data.index] != 0) continue;
            pOut[size] = data;
            pOut[size].index = spots_[data.index];
            size += 1;
            if (size > N) {
                break;
            }
        }
    }
private:
    struct findEdges
    {
        int left, right;
        findEdges(int i, int j) { left = i; right = j; }
        bool operator () (const WeightedBipartiteEdge &e) {
            return e.left == left && e.right == right;
        }
    };
    void dumpObj(const PV_OBJ_DATA &obj, char type[]) 
    {
        //qDebug(",%d,%s,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d",frames,type, obj.index,obj.x_pos,obj.y_pos,obj.z_pos,obj.x_speed,obj.y_speed,obj.z_speed,obj.length,obj.width,obj.height,obj.intensity);
    }
    bool newTarget(const PV_OBJ_DATA &data)
    {
        PV_OBJ_DATA temp = data;
        temp.index = slotForNewKalman();
        if (temp.index < 0) false;   //没有空位则丢弃目标
        temp.track_times = 0;  // 新目标没有追踪
        spots_[temp.index] = 1 + *std::max_element(spots_.begin(), spots_.end());
        temp.x_speed = temp.y_speed = temp.z_speed = 0.0f;
        prevTargets_.push_back(temp);
        dumpObj(temp, "New-first");

        //新目标，还需要其它信息计算变化量
        x_[temp.index] << temp.x_pos, temp.y_pos, temp.z_pos,
                        temp.length, temp.width, temp.height,
                        temp.intensity;
        u_[temp.index].setZero(); // Kalman对象还没有控制变量
        ukf_[temp.index].init(x_[temp.index]);
        return true;
    }
    /**
     * @brief 在0~300的序号选择一个没有使用的
     * 
     * @return int 选中一个。-1 表示没有空位
     */
    int slotForNewKalman()
    {
        std::vector<int> spots(N, -1);  // N 个位子
        // 初始化位子
        for (PV_OBJ_DATA &obj : prevTargets_) {
            assert(obj.index < N);   // 绝对不会超过 N 个目标
            spots[obj.index] = obj.index;  // 占位子
        }
        // 挑选空位子
        for (int i = 0; i < N; ++i) {
            if (spots[i] == -1)
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
    void kalmanFirst(PV_OBJ_DATA &left, const PV_OBJ_DATA &right) {
        left.x_speed = right.x_pos - left.x_pos;
        left.y_speed = right.y_pos - left.y_pos;
        left.z_speed = right.z_pos - left.z_pos;
        left.x_pos = right.x_pos;
        left.y_pos = right.y_pos;
        left.z_pos = right.z_pos;

        u_[left.index].dx() = left.x_speed;
        u_[left.index].dy() = left.y_speed;
        u_[left.index].dz() = left.z_speed;

        left.track_times += 1;
    }
    /**
     * @brief 预测目标的新对象，原目标不变，其它都不改变
     * 
     * @param obj 原目标
     * @return PV_OBJ_DATA 预测的目标新状态
     */
    PV_OBJ_DATA kalmanPredict(PV_OBJ_DATA &obj) {
        assert(obj.index < N);
        x_[obj.index] = ukf_[obj.index].predict(sys_, u_[obj.index]);

        PV_OBJ_DATA newObj = obj;
        newObj.x_pos = x_[obj.index].x();
        newObj.y_pos = x_[obj.index].y();
        newObj.z_pos = x_[obj.index].z();
        newObj.x_speed = x_[obj.index].x() - obj.x_pos;
        newObj.y_speed = x_[obj.index].y() - obj.y_pos;
        newObj.z_speed = x_[obj.index].z() - obj.z_pos;

        return newObj;
    }
    /**
     * @brief right作为观测对象更新Kalman模型
     * 
     * @param left 上一帧目标
     * @param right 下一帧对应的目标，也是观测结果
     */
    void kalmanUpdate(PV_OBJ_DATA &left, const PV_OBJ_DATA &right) {
        // 观测更新
        PositionMeasurement measure;
        measure << right.x_pos, right.y_pos, right.z_pos,
                    right.length, right.width, right.height,
                    right.intensity;
        x_[left.index] = ukf_[left.index].update(pmm_, measure);
        // 目标更新，观测结果right作为left的结果
        left.x_speed = x_[left.index].x()- left.x_pos; //[left.index].dx() + u_[left.index].ddx();
        left.y_speed = x_[left.index].y()- left.y_pos; //u_[left.index].dy() + u_[left.index].ddy();
        left.z_speed = x_[left.index].z()- left.z_pos; //u_[left.index].dz() + u_[left.index].ddz();
        left.x_pos = x_[left.index].x();
        left.y_pos = x_[left.index].y();
        left.z_pos = x_[left.index].z();
        left.pt1_x = right.pt1_x;
        left.pt2_x = right.pt2_x;
        left.pt3_x = right.pt3_x;
        left.pt4_x = right.pt4_x;
        left.pt1_y = right.pt1_y;
        left.pt2_y = right.pt2_y;
        left.pt3_y = right.pt3_y;
        left.pt4_y = right.pt4_y;
        left.length = (left.length + x_[left.index].length()) / 2;
        left.width = (left.width + x_[left.index].width()) / 2;
        left.height = (left.height + x_[left.index].height()) / 2;
        left.intensity = x_[left.index].intensity();
        predicts_[left.index] = 0;
        left.track_times += 1;
    }

    /**
     * @brief 将Kalman预测结果更新目标，同时预测+1
     *
     * @param obj 目标
     */
    void kalmanProcess(PV_OBJ_DATA &obj, PV_OBJ_DATA &next)
    {
        assert(obj.index < N);
        // 使用控制参数改变目标
        obj.x_speed = u_[obj.index].dx() + u_[obj.index].ddx();
        obj.y_speed = u_[obj.index].dy() + u_[obj.index].ddy();
        obj.z_speed = u_[obj.index].dz() + u_[obj.index].ddz();
        // BBox也要移动
        obj.pt1_x += next.x_pos - obj.x_pos;
        obj.pt2_x += next.x_pos - obj.x_pos;
        obj.pt3_x += next.x_pos - obj.x_pos;
        obj.pt4_x += next.x_pos - obj.x_pos;
        obj.pt1_y += next.y_pos - obj.y_pos;
        obj.pt2_y += next.y_pos - obj.y_pos;
        obj.pt3_y += next.y_pos - obj.y_pos;
        obj.pt4_y += next.y_pos - obj.y_pos;
        obj.x_pos = next.x_pos;
        obj.y_pos = next.y_pos;
        obj.z_pos = next.z_pos;
        float eu_speed = std::sqrt(obj.x_speed * obj.x_speed + obj.y_speed * obj.y_speed);
        if (eu_speed > obj.max_speed) {
            obj.max_speed = eu_speed;
        }
        predicts_[obj.index] += 1;
        obj.track_times -= 1;  // 雷达目标丢失，重新才跟踪
        obj.track_times = obj.track_times < 0 ? 0 : obj.track_times;
    }
    //TEST***
    qint64 kalman_elapsed_predict_ = 0;
    qint64 kalman_elapsed_update_ = 0;
    virtual void kalmanElapsedTime() {
        qDebug() << "kalman elapsed time: predict:" << kalman_elapsed_predict_ << " ,update:" << kalman_elapsed_update_;
        kalman_elapsed_predict_ = 0;
        kalman_elapsed_update_ = 0;
    }

private:
    std::vector<PV_OBJ_DATA> prevTargets_;  //前一次观测的集合
    float threshold_;   //距离阈值
    std::vector<int> spots_;   // 资源映射表

private:
    std::vector<int> predicts_;  // 纯粹的Kalman预测
    State x_[N];
    Control u_[N];
    Kalman::UnscentedKalmanFilter<State> ukf_[N];
    // 系统是整个场景一个系统
    SystemModel sys_;
    // Measurement models
    PositionModel pmm_;
    std::default_random_engine generator_;
    std::uniform_real_distribution<T> variance_;
    VAR_PARAMS var_params_;

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
            // qDebug(",%d,New-edges,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f",frames,j,nextSet[j].x_pos,nextSet[j].y_pos,nextSet[j].z_pos,nextSet[j].x_pos-prevSet[i].x_pos,nextSet[j].y_pos-prevSet[i].y_pos,nextSet[j].z_pos-prevSet[i].z_pos,nextSet[j].length,nextSet[j].width,nextSet[j].height,nextSet[j].intensity, d1);
            //qInfo("%f",d1);
            //dumpObj(nextSet[j], "New-edges");
            // 构造所有边的权重
            // if (d1 < threshold_)  // 阈值过滤
            edges.push_back( WeightedBipartiteEdge(i, j, d1) );
        }
        for (size_t j = nextSet.size(); j < prevSet.size(); ++j) {
            edges.push_back( WeightedBipartiteEdge(i, j, 1000.0f) );
        }
    }
    for (size_t i = prevSet.size(); i < nextSet.size(); ++i) {
        for (size_t j = 0; j < nextSet.size(); ++j) {
            edges.push_back(WeightedBipartiteEdge(i, j, 2000.0f));
        }
    }
    return edges;
}
std::vector<WeightedBipartiteEdge> createCosEdges(const std::vector<PV_OBJ_DATA> &prevSet, const std::vector<PV_OBJ_DATA> &nextSet)
{
    // 当前目标 next 与上一次目标 prev 的特征距离
    std::vector<WeightedBipartiteEdge> edges;
    for (size_t i = 0; i < prevSet.size(); ++i) {
        for (size_t j = 0; j < nextSet.size(); ++j) {
            float d1 = cosDistance(prevSet[i], nextSet[j]); //std::sqrt( delta.dot(delta) ); //计算向量距离
            // qDebug(",%d,New-edges,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f",frames,j,nextSet[j].x_pos,nextSet[j].y_pos,nextSet[j].z_pos,nextSet[j].x_pos-prevSet[i].x_pos,nextSet[j].y_pos-prevSet[i].y_pos,nextSet[j].z_pos-prevSet[i].z_pos,nextSet[j].length,nextSet[j].width,nextSet[j].height,nextSet[j].intensity, d1);
            //qInfo("%f",d1);
            //dumpObj(nextSet[j], "New-edges");
            // 构造所有边的权重
            // if (d1 < threshold_)  // 阈值过滤
            edges.push_back( WeightedBipartiteEdge(i, j, d1) );
        }
        for (size_t j = nextSet.size(); j < prevSet.size(); ++j) {
            edges.push_back( WeightedBipartiteEdge(i, j, 1000.0f) );
        }
    }
    for (size_t i = prevSet.size(); i < nextSet.size(); ++i) {
        for (size_t j = 0; j < nextSet.size(); ++j) {
            edges.push_back(WeightedBipartiteEdge(i, j, 2000.0f));
        }
    }
    return edges;
}
#define DIMS 2
/**
 * @brief 数据格式转换
 *
 * @param data 自定义数据
 * @return Kalman::Vector<float, 10> Eigen的向量
 */
Kalman::Vector<float, DIMS> toVector(const PV_OBJ_DATA &data) {
    Kalman::Vector<float, DIMS> target;
    target << //data.width, data.length, data.height,
              data.x_pos, data.y_pos;//, data.z_pos;
              //data.x_speed, data.y_speed; //, data.z_speed,
              //data.intensity;
    return target;
}

float eucDistance(const PV_OBJ_DATA &left, const PV_OBJ_DATA &right)
{
    // 欧氏距离
    Kalman::Vector<float, DIMS> prevTarget = toVector(left);
    PV_OBJ_DATA temp = right;
    temp.x_speed = (temp.x_pos - left.x_pos + left.x_speed) / 2;
    temp.y_speed = (temp.y_pos - left.y_pos + left.y_speed) / 2;
    temp.z_speed = (temp.z_pos - left.z_pos + left.z_speed) / 2;
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

    /**
     * @brief 速度的余弦距离
     * 
     * @return float 余弦距离[0, 2]
     */
    float cosDistance(const PV_OBJ_DATA &left, const PV_OBJ_DATA &right) {
        Kalman::Vector<float, 2> leftv;
        leftv << left.x_speed, left.y_speed;
        Kalman::Vector<float, 2> rightv;
        rightv << right.x_speed, right.y_speed;
        leftv.normalize();
        rightv.normalize();
        return 1 - leftv.dot(rightv) / (leftv.norm() * rightv.norm());
    }
void updateControl(int index)
{
    // 修正控制变量 u_
    u_[index].ddx() = u_[index].ddx() + var_params_.var_acc_x * variance_(generator_);
    u_[index].ddy() = u_[index].ddy() + var_params_.var_acc_y * variance_(generator_);
    u_[index].ddz() = u_[index].ddz() + var_params_.var_acc_z * variance_(generator_);
    u_[index].dx() = u_[index].dx() + var_params_.var_vel_x * variance_(generator_);
    u_[index].dy() = u_[index].dy() + var_params_.var_vel_y * variance_(generator_);
    u_[index].dz() = u_[index].dz() + var_params_.var_vel_z * variance_(generator_);
}

void updateControl(PV_OBJ_DATA &left, const PV_OBJ_DATA &right)
{
    //更新目标的控制
    u_[left.index].ddx() = right.x_pos - left.x_pos - left.x_speed; //[left.index].dx() + var_params_.var_acc_x * variance_(generator_);; // + 加速度的偏差
    u_[left.index].ddy() = right.y_pos - left.y_pos - left.y_speed; // u_[left.index].dy() + var_params_.var_acc_y * variance_(generator_);;
    u_[left.index].ddz() = right.z_pos - left.z_pos - left.z_speed; //[left.index].dz() + var_params_.var_acc_z * variance_(generator_);;
    u_[left.index].dx() = right.x_pos - left.x_pos; // + var_params_.var_vel_x * variance_(generator_);
    u_[left.index].dy() = right.y_pos - left.y_pos; // + var_params_.var_vel_y * variance_(generator_);
    u_[left.index].dz() = right.z_pos - left.z_pos; // + var_params_.var_vel_z * variance_(generator_);
    u_[left.index].di() = right.intensity - left.intensity;
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
