#include "CDataProcessor.h"
#include "CPluginContext.h"
#include <QJsonObject>
#include <QDateTime>
#include <QJsonValue>
#include "IDataDispatcher.h"


CDataProcessor::CDataProcessor(CPluginContext *pContext, IInterfaceManager *pInterfaceManager)
    : m_pContext(pContext)
    , m_pInterfaceManager(pInterfaceManager)
{
    // TOOD: 算法本身，需要在这里，从插件的配置文件中，将需要的参数解析出来，
    //       存成成员变量，以便在 ProcessData 函数中使用
    m_param = m_pContext->GetConfigValue("TargetThreshold").toFloat();
    lidar_ = nullptr;
}

CDataProcessor::~CDataProcessor()
{
    if (lidar_ != nullptr)
        delete lidar_;
}

bool CDataProcessor::Init()
{
    bool ret = false;
    do {

        ret = true;
    } while (false);
    return ret;
}

bool CDataProcessor::Uninit()
{
    bool ret = false;
    do {
        ret = true;
    } while (false);
    return ret;
}

#include <QMap>
bool CDataProcessor::ProcessData(const QByteArray &in)
{
    bool ret = false;

    do {
        if (m_pContext == nullptr)
        {
            break;
        }

        // TODO: 将 in 转换成输入的数据结构， 计算之后，生成输出的数据结构，存储到 out 中
        // 注意，这里的数据结构中，不能有指针，否则不能与 QByteArray 相互转换
        SIn* pIn = (SIn*)in.data();
        std::vector<PV_OBJ_DATA> inSet;
        QByteArray out(sizeof(SOut), '\0');
        SOut* pOut = (SOut*)out.data();
        // 加入Set并按照index排序
        inSet.insert(inSet.end(), pIn->m_obj_data, pIn->m_obj_data+pIn->m_obj_num);
        if (lidar_ == nullptr) {
            lidar_ = new KalmanTracking::LidarTracking(inSet,m_param);
        }
        else {
            lidar_->tracking(inSet);
            uint size;
            lidar_->output(pOut->m_obj_data, size);
            pOut->m_obj_num = size;
            qint64 timestamp = QDateTime::currentMSecsSinceEpoch();
            pOut->m_time_ms = timestamp;
        }

        // 多个输出，不通过out将数据输出
        /*
        QByteArray out1(sizeof(SOut1), '\0');
        SOut1* pOut1 = (SOut1*)out1.data();
        pOut1->p1 = pIn->p1;

        QByteArray out2(sizeof(SOut2), '\0');
        SOut2* pOut2 = (SOut2*)out2.data();
        pOut2->p1 = pIn->p1;
        */
        // 获取 数据分发接口，
        IDataDispatcher* pDataDispatcher = nullptr;
        if (m_pInterfaceManager == nullptr) break;

        if (!m_pInterfaceManager->QueryInterface("IDataDispatcher", (void**)&pDataDispatcher)) break;

        // 这里，是进程内，插件间的数据分离，直接使用数据分发器，不通过共享内存
        pDataDispatcher->Dispatch("SOut", out);
        //pDataDispatcher->Dispatch("SOut2", out2);


        // 如果需要处理 SOut1或者 SOut2 的数据, 需要向数据分发器订阅数据，方法为:
//        pDataDispatcher->Register(p, "SOut1");
        // 需要在 CPluginDemo::RegistInterfaces 中注册 对应类型的数据

        ret = true;
    } while (false);
    return ret;
}
