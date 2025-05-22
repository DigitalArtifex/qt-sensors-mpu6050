#ifndef QMPU6_5_ACCELEROMETERBACKEND_H
#define QMPU6_5_ACCELEROMETERBACKEND_H

#include <QObject>
#include <QString>
#include <QTextStream>
#include <QSensorBackend>
#include <QTimer>
#include <QThread>
#include <QDateTime>
#include <QAccelerometerReading>

#include <QObject>

#include "qmpu6050backend.h"
#include "qi2cdevice.h"
#include "qmpu6050_p.h"

class QMPU6050AccelerometerBackend : public QSensorBackend
{
    Q_OBJECT
public:
    static inline const char* id = "baremetal.mpu6050.accelerometer";
    static inline const quint8 m_chipId = 0x58; //i2c chip id

    explicit QMPU6050AccelerometerBackend(QSensor *sensor = nullptr);
    ~QMPU6050AccelerometerBackend();

    virtual void start() override;
    virtual void stop() override;

private slots:
    void poll();
    void handleFault();
    void reportEvent(QString message);
    void reportError(QString message);
    void onSensorDataRateChanged();

private:
    QTimer *m_pollTimer = nullptr;
    QAccelerometerReading m_reading;
    QMPU6050Backend *m_backend = nullptr;
    QI2CDevice *m_i2c = nullptr;
    bool m_backendDebug = false;
};

#endif // QMPU6_5_ACCELEROMETERBACKEND_H
