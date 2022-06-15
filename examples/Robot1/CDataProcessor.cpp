#include "CDataProcessor.h"
#include "CPluginContext.h"
#include <QJsonObject>
#include <QDateTime>
#include <QJsonValue>
#include "IDataDispatcher.h"

int KalmanTracking::LidarTracking::frames = 0;

#include "CommonDefine.h"
CDataProcessor::CDataProcessor(CPluginContext *pContext, IInterfaceManager *pInterfaceManager)
    : m_pContext(pContext)
    , m_pInterfaceManager(pInterfaceManager)
{
    // TOOD: 算法本身，需要在这里，从插件的配置文件中，将需要的参数解析出来，
    //       存成成员变量，以便在 ProcessData 函数中使用
    m_param = m_pContext->GetConfigValue("TargetThreshold").toDouble();
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
        m_pRedisClient = new CRedisClient();
        if (!m_pRedisClient->Init())
        {
            qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis客户端初始化失败");
            m_pRedisClient = nullptr;
        }
        ret = true;
    } while (false);
    return ret;
}

bool CDataProcessor::Uninit()
{
    bool ret = false;
    do {
        if (m_pRedisClient != nullptr)
        {
            m_pRedisClient->Uninit();
            m_pRedisClient->deleteLater();
            m_pRedisClient = nullptr;
        }
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

        // 注意，这里的数据结构中，不能有指针，否则不能与 QByteArray 相互转换
        SIn* pIn = (SIn*)in.data();
        if (pIn->m_obj_num <= 0) {
            // 没有目标或异常条件下返回false时
            break;
        }
        // TODO 没有聚类目标，但是旧目标的Kalman状态应该更新的

        m_ba_objectinfo.fill('\0',sizeof(SOut));
        SOut* pOut = (SOut*)m_ba_objectinfo.data();
        // 加入Set并按照index排序
        inSet.insert(inSet.end(), pIn->m_obj_data, pIn->m_obj_data+pIn->m_obj_num);
        if (lidar_ == nullptr) {
            lidar_ = new KalmanTracking::LidarTracking(inSet,m_param);
        }
        else {
            lidar_->tracking(inSet);
            uint size=0;
            lidar_->output(pOut->m_obj_data, size);
            pOut->m_obj_num = size;
            qint64 timestamp = QDateTime::currentMSecsSinceEpoch();
            pOut->m_time_ms = timestamp;
#if 0
            for(int i = 0;i<pOut->m_obj_num;i++)
            {
                QStringList redisInfo;
                redisInfo.append(QString::number(pOut->m_obj_data[i].index));
                QString strRedisKey = QString::fromLatin1(pOut->m_obj_data[i].redis_key,40);
                redisInfo.append(strRedisKey);
                if(m_pRedisClient!=nullptr)
                {
                    if(!m_pRedisClient->ListLPush("Redis_Key_PCD_For_Recognize_Info", redisInfo.join(",")))
                    {
                        m_pRedisClient->Reconnect();
                    }
                }
            }
#endif

        }

        inSet.clear();


        // 多个输出，不通过out将数据输出
        /*
        QByteArray out1(sizeof(SOut1), '\0');
        SOut1* pOut1 = (SOut1*)out1.data();
        pOut1->p1 = pIn->p1;

        QByteArray out2(sizeof(SOut2), '\0');
        SOut2* pOut2 = (SOut2*)out2.data();
        pOut2->p1 = pIn->p1;
        */
#if 1
        // 获取 数据分发接口，
        IDataDispatcher* pDataDispatcher = nullptr;
        if (m_pInterfaceManager == nullptr) break;

        if (!m_pInterfaceManager->QueryInterface("IDataDispatcher", (void**)&pDataDispatcher)) break;

        // 这里，是进程内，插件间的数据分离，直接使用数据分发器，不通过共享内存
        pDataDispatcher->Dispatch(INKey_AfterTrackingInfo, m_ba_objectinfo);
        //pDataDispatcher->Dispatch("SOut2", out2);

#endif
        // 如果需要处理 SOut1或者 SOut2 的数据, 需要向数据分发器订阅数据，方法为:
//        pDataDispatcher->Register(p, "SOut1");
        // 需要在 CPluginDemo::RegistInterfaces 中注册 对应类型的数据

        ret = true;
    } while (false);
    return ret;
}
