#ifndef CDATAPROCESSOR_H
#define CDATAPROCESSOR_H

#include "IDataProcessor.h"
#include "CPluginContext.h"
#include "IInterfaceManager.h"
#include <QJsonValue>

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
};

#endif // CDATAPROCESSOR_H
