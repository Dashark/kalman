#include "CDataProcessor.h"
#include "CPluginContext.h"
#include <QJsonObject>
#include <QDateTime>
#include <QJsonValue>
#include <math.h>
#include "IDataDispatcher.h"
#include "IGlobalVariableManager.h"

int KalmanTracking::LidarTracking::frames = 0;

#include "CommonDefine.h"
CDataProcessor::CDataProcessor(CPluginContext *pContext, IInterfaceManager *pInterfaceManager)
    : QObject(nullptr)
    , m_pContext(pContext)
    , m_pInterfaceManager(pInterfaceManager)
    , m_pGlobalVariableHelper(nullptr)
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
        if (m_pInterfaceManager == nullptr) break;

        m_pRedisClient = new CRedisClient();
        if (!m_pRedisClient->Init())
        {
            qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis客户端初始化失败");
            m_pRedisClient = nullptr;
        }

        // 初始化全局变量监听
        {
            m_pGlobalVariableHelper = new CGlobalVariableHelper(m_pContext, m_pInterfaceManager);
            if (!m_pGlobalVariableHelper->Init())
            {
                break;
            }
            connect(m_pGlobalVariableHelper, &CGlobalVariableHelper::signalForGlobalVariablesChanged,
                    this, &CDataProcessor::SlotForGlobalVariablesChanged, Qt::QueuedConnection);

            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:CosDistance", &m_algParamCosDistance);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:DistanceVector", &m_algParamDistanceVector);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:DistanceLimit", &m_algParamDistanceLimit);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVariancePosX", &m_algParamInitVariancePosX);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVariancePosY", &m_algParamInitVariancePosY);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVariancePosZ", &m_algParamInitVariancePosZ);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVarianceVelX", &m_algParamInitVarianceVelX);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVarianceVelY", &m_algParamInitVarianceVelY);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVarianceVelZ", &m_algParamInitVarianceVelZ);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVarianceACCX", &m_algParamInitVarianceAccX);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVarianceACCY", &m_algParamInitVarianceAccY);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVarianceACCZ", &m_algParamInitVarianceAccZ);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVarianceHeading", &m_algParamInitVarianceHeading);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:InitVarianceHeadingRate", &m_algParamInitVarianceHeadingRate);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:MinTrackNum", &m_algParamMinTrackNum);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:MinPredictNum", &m_algParamMinPredictNum);
            m_pGlobalVariableHelper->BindGlobalVariable("TrackParams:MinPredictShow", &m_algParamMinPredictShow);

            m_pGlobalVariableHelper->BindGlobalVariable("KalmanEffectiveParams:PosX", &m_usingPosXInKalman);
            m_pGlobalVariableHelper->BindGlobalVariable("KalmanEffectiveParams:PosY", &m_usingPosYInKalman);
            m_pGlobalVariableHelper->BindGlobalVariable("KalmanEffectiveParams:PosZ", &m_usingPosZInKalman);
            m_pGlobalVariableHelper->BindGlobalVariable("KalmanEffectiveParams:VelX", &m_usingVelXInKalman);
            m_pGlobalVariableHelper->BindGlobalVariable("KalmanEffectiveParams:VelY", &m_usingVelYInKalman);
            m_pGlobalVariableHelper->BindGlobalVariable("KalmanEffectiveParams:VelZ", &m_usingVelZInKalman);
            m_pGlobalVariableHelper->BindGlobalVariable("KalmanEffectiveParams:Length", &m_usingLenghtInKalman);
            m_pGlobalVariableHelper->BindGlobalVariable("KalmanEffectiveParams:Width", &m_usingWidthInKalman);
            m_pGlobalVariableHelper->BindGlobalVariable("KalmanEffectiveParams:Height", &m_usingHeightInKalman);

            m_pGlobalVariableHelper->BindGlobalVariable("TrackVisible", &m_trackVisible);
        }

        // 轨迹不可见时，不启动跟踪服务
//        {
//            QByteArray ba_in;
//            if (!m_pRedisClient->Get("TrackVisible", ba_in))
//            {
//                qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis字符串TrackVisible非true，不允许启动跟踪服务");
//                break;
//            }

//            QString strIn = QString::fromUtf8(ba_in);
//            if (strIn != "true")
//            {
//                qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis字符串TrackVisible非true，不允许启动跟踪服务");
//                break;
//            }
//        }

        m_pTimer = new QTimer();
        connect(m_pTimer, &QTimer::timeout, this, &CDataProcessor::slotForTimeout);
        m_pTimer->start(1000);

        ret = true;
    } while (false);
    return ret;
}

bool CDataProcessor::Uninit()
{
    bool ret = false;
    do {
        if (m_pTimer != nullptr)
        {
            m_pTimer->stop();
            m_pTimer->deleteLater();
            m_pTimer = nullptr;
        }
        if (m_pRedisClient != nullptr)
        {
            m_pRedisClient->Uninit();
            m_pRedisClient->deleteLater();
            m_pRedisClient = nullptr;
        }
        if (m_pGlobalVariableHelper != nullptr)
        {
            m_pGlobalVariableHelper->Uninit();
            m_pGlobalVariableHelper->deleteLater();
            m_pGlobalVariableHelper = nullptr;
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

        // 更新所有绑定了的全局变量
        {
            if (m_pGlobalVariableHelper == nullptr)
            {
                break;
            }
            IGlobalVariableManager* pGlobalVariableManager = nullptr;
            if (!m_pInterfaceManager->QueryInterface("IGlobalVariableManager", (void**)&pGlobalVariableManager))
            {
                break;
            }

            m_pGlobalVariableHelper->UpdateAllBindGlobalVariables();
        }

        if (!m_trackVisible)
        {
            // 不执行跟踪计算
            break;
        }

        // 注意，这里的数据结构中，不能有指针，否则不能与 QByteArray 相互转换
        SIn* pIn = (SIn*)in.data();
        // 根据卡尔曼参数，对输入数据做变更
        changeInputData(pIn);
        if (!verifyData(pIn)) {
            // 没有目标或异常条件下返回false时
            break;
        }
        // TODO 没有聚类目标，但是旧目标的Kalman状态应该更新的

        m_ba_objectinfo.fill('\0',sizeof(SOut));
        SOut* pOut = (SOut*)m_ba_objectinfo.data();
        // 加入Set并按照index排序
        inSet.insert(inSet.end(), pIn->m_obj_data, pIn->m_obj_data+pIn->m_obj_num);
        if (lidar_ == nullptr) {
#if 1
            VAR_PARAMS pa;
            pa.first_distance = m_algParamDistanceLimit;
            pa.sec_distance = m_param;
            pa.cos_distance = m_algParamCosDistance;
            pa.var_pos_x = m_algParamInitVariancePosX;
            pa.var_pos_y = m_algParamInitVariancePosY;
            pa.var_pos_z = m_algParamInitVariancePosZ;
            pa.var_acc_x = m_algParamInitVarianceAccX;
            pa.var_acc_y = m_algParamInitVarianceAccY;
            pa.var_acc_z = m_algParamInitVarianceAccZ;
            pa.var_vel_x = m_algParamInitVarianceVelX;
            pa.var_vel_y = m_algParamInitVarianceVelY;
            pa.var_vel_z = m_algParamInitVarianceVelZ;
            pa.var_heading = m_algParamInitVarianceHeading;
            pa.var_heading_rate = m_algParamInitVarianceHeadingRate;
            lidar_ = new KalmanTracking::LidarTracking(inSet,pa);
#else

            lidar_ = new KalmanTracking::LidarTracking(inSet,m_param);

#endif
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
        if (pOut->m_obj_num > 0)
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

void CDataProcessor::slotForTimeout()
{
    do {
        // 获取是否执行跟踪计算功能
        {
            QByteArray ba_in;
            if (!m_pRedisClient->Get("TrackVisible", ba_in))
            {
                qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis字符串找不到TrackVisible");
                break;
            }

            QString strTrackVisible = QString::fromUtf8(ba_in);

            IGlobalVariableManager* pGlobalVariableManager = nullptr;
            if (!m_pInterfaceManager->QueryInterface("IGlobalVariableManager", (void**)&pGlobalVariableManager))
            {
                break;
            }

            pGlobalVariableManager->SetGlobalVariable("TrackVisible", "bool", strTrackVisible);

        }
    } while (false);

    do {
        // 获取跟踪计算算法参数
        if (!UpdateTrackParams())
        {
            break;
        }
    } while (false);

    do {
        if (!UpDateKalmanEffectiveParams())
        {
            break;
        }
    } while (false);

}

void CDataProcessor::SlotForGlobalVariablesChanged()
{
    m_pGlobalVariableHelper->UpdateAllBindGlobalVariables();
    qDebug() << Q_FUNC_INFO
             << "m_algParamMinTrackNum = " << m_algParamMinTrackNum
             << ", m_algParamMinPredictNum = " << m_algParamMinPredictNum
             << ", m_algParamMinPredictShow = " << m_algParamMinPredictShow;
    VAR_PARAMS pa;
    pa.first_distance = m_algParamDistanceLimit;
    pa.sec_distance = m_param;
    pa.cos_distance = m_algParamCosDistance;
    pa.var_pos_x = m_algParamInitVariancePosX;
    pa.var_pos_y = m_algParamInitVariancePosY;
    pa.var_pos_z = m_algParamInitVariancePosZ;
    pa.var_acc_x = m_algParamInitVarianceAccX;
    pa.var_acc_y = m_algParamInitVarianceAccY;
    pa.var_acc_z = m_algParamInitVarianceAccZ;
    pa.var_vel_x = m_algParamInitVarianceVelX;
    pa.var_vel_y = m_algParamInitVarianceVelY;
    pa.var_vel_z = m_algParamInitVarianceVelZ;
    pa.var_heading = m_algParamInitVarianceHeading;
    pa.var_heading_rate = m_algParamInitVarianceHeadingRate;
    lidar_->modifyParams(pa);
}

bool CDataProcessor::UpdateTrackParams()
{
    bool ret = false;
    do {

        QByteArray baTrackParams;
        if (!m_pRedisClient->Get("TrackParams", baTrackParams))
        {
            qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis字符串找不到TrackParams");
            break;
        }
        if (baTrackParams.at(baTrackParams.size()-1) == 0)
        {
            baTrackParams = baTrackParams.mid(0, baTrackParams.size()-1);
        }
        QJsonParseError error;
        QJsonDocument jdTrackParams = QJsonDocument::fromJson(baTrackParams, &error);
        if (jdTrackParams.isNull())
        {
            qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis字符串TrackParams不是合法的json");
            qWarning() << Q_FUNC_INFO << error.errorString();
            break;
        }
        QJsonObject joTraceParam = jdTrackParams.object();
        if (joTraceParam.isEmpty())
        {
            qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis字符串TrackParams不是合法的json");
            break;
        }

        IGlobalVariableManager* pGlobalVariableManager = nullptr;
        if (!m_pInterfaceManager->QueryInterface("IGlobalVariableManager", (void**)&pGlobalVariableManager))
        {
            break;
        }

        //TODO: 修正pview端，每个变量使用一个redis字符串，简化逻辑
        pGlobalVariableManager->SetGlobalVariable("TrackParams:MinTrackNum", "int", joTraceParam.value("MinTrackNum").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:MinPredictNum", "int", joTraceParam.value("MinPredictNum").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:MinPredictShow", "int", joTraceParam.value("MinPredictShow").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:CosDistance", "double", joTraceParam.value("CosDistance").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:DistanceVector", "string", joTraceParam.value("DistanceVector").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:DistanceLimit", "double", joTraceParam.value("DistanceLimit").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVariancePosX", "double", joTraceParam.value("InitVariancePosX").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVariancePosY", "double", joTraceParam.value("InitVariancePosY").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVariancePosZ", "double", joTraceParam.value("InitVariancePosZ").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVarianceVelX", "double", joTraceParam.value("InitVarianceVelX").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVarianceVelY", "double", joTraceParam.value("InitVarianceVelY").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVarianceVelZ", "double", joTraceParam.value("InitVarianceVelZ").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVarianceACCX", "double", joTraceParam.value("InitVarianceACCX").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVarianceACCY", "double", joTraceParam.value("InitVarianceACCY").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVarianceACCZ", "double", joTraceParam.value("InitVarianceACCZ").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVarianceHeading", "double", joTraceParam.value("InitVarianceHeading").toString());
        pGlobalVariableManager->SetGlobalVariable("TrackParams:InitVarianceHeadingRate", "double", joTraceParam.value("InitVarianceHeadingRate").toString());

        ret = true;
    } while (false);
    return ret;
}

bool CDataProcessor::UpDateKalmanEffectiveParams()
{
    bool ret = false;
    do {

        QByteArray baKalmanEffectiveParams;
        if (!m_pRedisClient->Get("KalmanEffectiveParams", baKalmanEffectiveParams))
        {
            qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis字符串找不到KalmanEffectiveParams");
            break;
        }
        if (baKalmanEffectiveParams.at(baKalmanEffectiveParams.size()-1) == 0)
        {
            baKalmanEffectiveParams = baKalmanEffectiveParams.mid(0, baKalmanEffectiveParams.size()-1);
        }
        QString strKalmanEffectiveParams = QString::fromUtf8(baKalmanEffectiveParams);
        qDebug() << Q_FUNC_INFO << strKalmanEffectiveParams;

        QJsonDocument jdKalmanEffectiveParams = QJsonDocument::fromJson(strKalmanEffectiveParams.toUtf8());
        if (jdKalmanEffectiveParams.isNull())
        {
            qWarning() << Q_FUNC_INFO << QString::fromUtf8("redis字符串KalmanEffectiveParams不是合法的json");
            break;
        }

        IGlobalVariableManager* pGlobalVariableManager = nullptr;
        if (!m_pInterfaceManager->QueryInterface("IGlobalVariableManager", (void**)&pGlobalVariableManager))
        {
            break;
        }

        QJsonObject joKalmanEffectiveParams = jdKalmanEffectiveParams.object();

        //TODO: 修正pview端，每个变量使用一个redis字符串，简化逻辑
        pGlobalVariableManager->SetGlobalVariable("KalmanEffectiveParams:PosX", "bool", joKalmanEffectiveParams.value("PosX").toString());
        pGlobalVariableManager->SetGlobalVariable("KalmanEffectiveParams:PosY", "bool", joKalmanEffectiveParams.value("PosY").toString());
        pGlobalVariableManager->SetGlobalVariable("KalmanEffectiveParams:PosZ", "bool", joKalmanEffectiveParams.value("PosZ").toString());
        pGlobalVariableManager->SetGlobalVariable("KalmanEffectiveParams:VelX", "bool", joKalmanEffectiveParams.value("VelX").toString());
        pGlobalVariableManager->SetGlobalVariable("KalmanEffectiveParams:VelY", "bool", joKalmanEffectiveParams.value("VelY").toString());
        pGlobalVariableManager->SetGlobalVariable("KalmanEffectiveParams:VelZ", "bool", joKalmanEffectiveParams.value("VelZ").toString());
        pGlobalVariableManager->SetGlobalVariable("KalmanEffectiveParams:Length", "bool", joKalmanEffectiveParams.value("Length").toString());
        pGlobalVariableManager->SetGlobalVariable("KalmanEffectiveParams:Width", "bool", joKalmanEffectiveParams.value("Width").toString());
        pGlobalVariableManager->SetGlobalVariable("KalmanEffectiveParams:Height", "bool", joKalmanEffectiveParams.value("Height").toString());

        ret = true;
    } while (false);
    return ret;
}

void CDataProcessor::changeInputData(SIn *pin)
{
    if (pin != nullptr)
    {
        for (int i = 0; i < pin->m_obj_num; ++i)
        {
            PV_OBJ_DATA *tp = pin->m_obj_data + i;

            //!< 不在 卡尔曼过程中使用 X 坐标
            if (!m_usingPosXInKalman)
            {
                tp->x_pos = -10000.0f;
            }

            //!< 不在 卡尔曼过程中使用 Y 坐标
            if (!m_usingPosYInKalman)
            {
                tp->y_pos = -10000.0f;
            }

            //!< 不在 卡尔曼过程中使用 Z 坐标
            if (!m_usingPosZInKalman)
            {
                tp->z_pos = -10000.0f;
            }

            //!< 不在 卡尔曼过程中使用 X 速度
            if (!m_usingVelXInKalman)
            {
                tp->x_speed = -10000.0f;
            }

            //!< 不在 卡尔曼过程中使用 Y 速度
            if (!m_usingVelYInKalman)
            {
                tp->y_speed = -10000.0f;
            }

            //!< 不在 卡尔曼过程中使用 Z 速度
            if (!m_usingVelZInKalman)
            {
                tp->z_speed = -10000.0f;
            }

            //!< 不在 卡尔曼过程中使用 长度
            if (!m_usingLenghtInKalman)
            {
                tp->length = -10000.0f;
            }

            //!< 不在 卡尔曼过程中使用 宽度
            if (!m_usingWidthInKalman)
            {
                tp->width = -10000.0f;
            }

            //!< 不在 卡尔曼过程中使用 高度
            if (!m_usingHeightInKalman)
            {
                tp->height = -10000.0f;
            }
        }
    }
}

bool CDataProcessor::verifyData(SIn *pin)
{
    bool ret = false;
    if (pin->m_obj_num <= 0)
        return ret;

    for (int i = 0; i < pin->m_obj_num; ++i) {
        PV_OBJ_DATA *tp = pin->m_obj_data + i;
        // just one attr is NAN or INF
        if (!isnormal(tp->width))
            tp->width = 0.1f;
        if (!isnormal(tp->height))
            tp->height = 0.1f;
        if (!isnormal(tp->length))
            tp->length = 0.1f;
        if (!isnormal(tp->x_pos))
            tp->x_pos = 0.1f;
        if (!isnormal(tp->y_pos))
            tp->y_pos = 0.1f;
        if (!isnormal(tp->z_pos))
            tp->z_pos = 0.1f;
        if (!isnormal(tp->x_speed))
            tp->x_speed = 0.1f;
        if (!isnormal(tp->y_speed))
            tp->y_speed = 0.1f;
        if (!isnormal(tp->z_speed))
            tp->z_speed = 0.1f;
    }
    ret = true;
    return ret;
}
