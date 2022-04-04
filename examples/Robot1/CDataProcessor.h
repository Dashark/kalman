#ifndef CDATAPROCESSOR_H
#define CDATAPROCESSOR_H

#include "IDataProcessor.h"
#include "CPluginContext.h"
#include "IInterfaceManager.h"
#include <QJsonValue>

#include "CRedisClient.h"
#include "Tracking.hpp"

class CDataProcessor : public IDataProcessor
{
public:
    explicit CDataProcessor(CPluginContext* pContext=nullptr, IInterfaceManager* pInterfaceManager=nullptr);
    virtual ~CDataProcessor();

public:
    bool Init();
    bool Uninit();

public:
    bool ProcessData(const QByteArray& in) override;

private:
    CPluginContext* m_pContext{nullptr};
    IInterfaceManager* m_pInterfaceManager{nullptr};

    int m_param{0};
    KalmanTracking::LidarTracking *lidar_;
    QByteArray m_ba_objectinfo;

    std::vector<PV_OBJ_DATA> inSet;

    CRedisClient* m_pRedisClient{nullptr};
};

#endif // CDATAPROCESSOR_H
