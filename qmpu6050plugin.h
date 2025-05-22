#ifndef QMPU6050PLUGIN_H
#define QMPU6050PLUGIN_H

#include <QObject>
#include <QSensorBackendFactory>
#include <QSensorPluginInterface>
#include <QSensorChangesInterface>
#include <QAccelerometer>
#include <QGyroscope>

#include "qmpu6050_global.h"
#include "qmpu6050backend.h"
#include "qmpu6050accelerometerbackend.h"
#include "qmpu6050gyroscopebackend.h"
#include "qmpu6050.h"

QT_BEGIN_NAMESPACE

class QMPU6_5__EXPORT QMPU6050Plugin : public QObject, public QSensorPluginInterface, public QSensorChangesInterface, public QSensorBackendFactory
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.qt-project.Qt.QSensorPluginInterface/1.0")
    Q_INTERFACES(QSensorPluginInterface QSensorChangesInterface)
public:
    void registerSensors() override
    {
        QSensor::defaultSensorForType(QMPU6050::sensorType);
        QSensorManager::registerBackend(QMPU6050::sensorType, QMPU6050Backend::id, this);
        QSensorManager::registerBackend(QAccelerometer::sensorType, QMPU6050AccelerometerBackend::id, this);
        QSensorManager::registerBackend(QGyroscope::sensorType, QMPU6050GyroscopeBackend::id, this);
    }

    void sensorsChanged() override
    {
        //register backend on initial load
        if(!QSensorManager::isBackendRegistered(QMPU6050::sensorType, QMPU6050Backend::id))
            QSensorManager::registerBackend(QMPU6050::sensorType, QMPU6050Backend::id, this);

        if(!QSensorManager::isBackendRegistered(QAccelerometer::sensorType, QMPU6050AccelerometerBackend::id))
            QSensorManager::registerBackend(QAccelerometer::sensorType, QMPU6050AccelerometerBackend::id, this);

        if(!QSensorManager::isBackendRegistered(QGyroscope::sensorType, QMPU6050GyroscopeBackend::id))
            QSensorManager::registerBackend(QGyroscope::sensorType, QMPU6050GyroscopeBackend::id, this);

        //TODO: Create backends for the gyroscope, compass and temperature
    }

    QSensorBackend *createBackend(QSensor *sensor) override
    {
        if (sensor->identifier() == QMPU6050Backend::id)
            return new QMPU6050Backend(sensor);
        else if (sensor->identifier() == QMPU6050AccelerometerBackend::id)
            return new QMPU6050AccelerometerBackend(sensor);
        else if (sensor->identifier() == QMPU6050GyroscopeBackend::id)
            return new QMPU6050GyroscopeBackend(sensor);

        return 0;
    }
};

QT_END_NAMESPACE

#endif
