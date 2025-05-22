#ifndef QMPU6_5_GYROSCOPEBACKEND_H
#define QMPU6_5_GYROSCOPEBACKEND_H
#include <QObject>
#include <QString>
#include <QTextStream>
#include <QSensorBackend>
#include <QTimer>
#include <QThread>
#include <QDateTime>
#include <QGyroscopeReading>

#include <QObject>

#include "qmpu6050backend.h"
#include "qmpu6050_p.h"

class QMPU6050GyroscopeBackend : public QSensorBackend
{
    Q_OBJECT
public:
    static inline const char* id = "baremetal.mpu6050.gyroscope";
    static inline const quint8 m_chipId = 0x58; //i2c chip id

    explicit QMPU6050GyroscopeBackend(QSensor *sensor = nullptr);
    ~QMPU6050GyroscopeBackend();

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
    QGyroscopeReading m_reading;
    QI2CDevice *m_i2c = nullptr;

    bool m_backendDebug = false;
};

#endif // QMPU6_5_GYROSCOPEBACKEND_H
