#ifndef CDATAPROCESSOR_H
#define CDATAPROCESSOR_H

#include "IDataProcessor.h"
#include "CPluginContext.h"
#include "IInterfaceManager.h"
#include <QJsonValue>
#include <QTimer>
#include <QObject>
#include "CRedisClient.h"
#include "Tracking.hpp"

class CDataProcessor
        : public QObject
        , public IDataProcessor
{
    Q_OBJECT
public:
    explicit CDataProcessor(CPluginContext* pContext=nullptr, IInterfaceManager* pInterfaceManager=nullptr);
    virtual ~CDataProcessor();

public:
    bool Init();
    bool Uninit();

public:
    bool ProcessData(const QByteArray& in) override;

private slots:
    void slotForTimeout();

private:
    bool UpdateTrackParams();

private:
    CPluginContext* m_pContext{nullptr};
    IInterfaceManager* m_pInterfaceManager{nullptr};

    int m_param{0};
    KalmanTracking::LidarTracking *lidar_;
    QByteArray m_ba_objectinfo;

    std::vector<PV_OBJ_DATA> inSet;

    CRedisClient* m_pRedisClient{nullptr};
    QTimer*         m_pTimer{nullptr};

private:
    bool        m_trackVisible{false};                  //!< 跟踪是否可见，是否执行跟踪计算
    QString     m_algParamDistanceVector{""};           //!< 距离向量 缺省是X，Y值，可选 X轴速度，Y轴速度，强度。
    double      m_algParamDistanceLimit{0.0};           //!<
    double      m_algParamInitVariancePosX{0.1};        //!< 初始方差-X轴<initVariancePosX>：0.1
    double      m_algParamInitVariancePosY{0.1};        //!< 初始方差-Y轴<initVariancePosY>：0.1
    double      m_algParamInitVariancePosZ{0.0};        //!< 初始方差-Z轴<initVariancePosZ>：0.0
    double      m_algParamInitVarianceVelX{10.0};       //!< 初始方差-X轴速度<initVarianceVelX>：10
    double      m_algParamInitVarianceVelY{10.0};       //!< 初始方差-Y轴速度<initVarianceVelY>：10
    double      m_algParamInitVarianceVelZ{0.0};        //!< 初始方差-Z轴速度<initVarianceVelZ>：0.0
    double      m_algParamInitVarianceAccX{0.1};        //!< 初始方差-X轴加速度<initVarianceAccX>：0.1
    double      m_algParamInitVarianceAccY{0.1};        //!< 初始方差-Y轴加速度<initVarianceAccY>：0.1
    double      m_algParamInitVarianceAccZ{0.0};        //!< 初始方差-Z轴加速度<initVarianceAccZ>：0.0
    double      m_algParamInitVarianceHeading{2.46};    //!< 初始方差-方向<initVarianceHeading>：2.46
    double      m_algParamInitVarianceHeadingRate{0.1}; //!< 初始方差-方向变化率<initVarianceHeadingRate>：0.1
};

#endif // CDATAPROCESSOR_H
