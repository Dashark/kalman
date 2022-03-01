#include <set>
#include <kalman/Types.hpp>

typedef struct {
    float x;        //精度 10微米
    float y;        //精度 10微米
    float z;        //精度 10微米
    qint32 intensity;    //能量值
}PV_POINT_XYZI;

//目标检测结果数据
typedef struct {
    uint index;             // 目标临时编号
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
    qint64 m_time_ms;             //消息时间戳
    qint32 m_obj_num;             //有效目标数量
    PV_OBJ_DATA m_obj_data[300];  //目标参数
    int m_obj_point_count[300];   //第x个目标的点云的数量
    PV_POINT_XYZI points[1331200];  //目标点云数据10400*8*16
};

bool CDataProcessor::ProcessData(const QByteArray &in, QByteArray &out)
{
    SIn* pIn = (SIn*)(in.data());
    std::set<PV_OBJ_DATA> inSet;
    for (int i = 0; i < pIn->m_obj_num; i++) {
        // 加入Set并按照index排序
        inSet.add(pIn->m_obj_data, pIn->m_obj_data+pIn->m_obj_num);
    }
    std::set<PV_OBJ_DATA> prevSet;
    // 当前目标 pIn 与上一次目标 pPrev 匹配关系
    std::vector<WeightedBipartiteEdge> edges;
    for (auto &prev : prevSet) {
        Kalman::Vector<float, 9> prevTarget;
        prevTarget << prev.width, prev.length, prev.height, prev.x_pos, prev.y_pos;
        for (auto &next : inSet) {
            Kalman::Vector<float, 9> nextTarget;
            nextTarget << next.width;
            Kalman::Vector<float, 9> delta = nextTarget - prevTarget;
            float d1 = std::sqrt( delta.dot(delta) );
            edges.push_back( WeightedBipartiteEdge(prev.index, next.index, d1) );
        }
    }
    std::vector<int> matching = hungarianMinimumWeightPerfectMatching(prev.size(), edges);
    // TODO 左边与右边数量不一致会怎样？
    // TODO 左边多个节点会连接到右边一个节点吗？
    // 还要剔除距离明显过大的匹配，从而得到未成功匹配的目标
    for (int &id : matching) {
        d 
    }
    // 处理完成得到3个集合，left, right, 和上面的matching
    // 对matching做Kalman预测与更新
    // 对left做Kalman预测
    // 对right做新建目标
}