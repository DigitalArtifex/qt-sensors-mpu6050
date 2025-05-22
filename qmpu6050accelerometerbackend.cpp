#include "qmpu6050accelerometerbackend.h"

QMPU6050AccelerometerBackend::QMPU6050AccelerometerBackend(QSensor *sensor)
    : QSensorBackend{sensor}
{
    m_pollTimer = new QTimer(this);
    QObject::connect(m_pollTimer, SIGNAL(timeout()), this, SLOT(poll()));


    QAccelerometer *child = qobject_cast<QAccelerometer*>(sensor);

    if(child)
    {
        QObject::connect(child, &QAccelerometer::dataRateChanged, this, &QMPU6050AccelerometerBackend::onSensorDataRateChanged);
        setReading<QAccelerometerReading>(&m_reading);
        reading();

        //setup i2c device
        m_i2c = new QI2CDevice;

        if(child->property("i2c-bus").isValid())
            m_i2c->setBus(child->property("i2c-bus").toString());
        else
            m_i2c->setBus("/dev/i2c-1");

        if(child->property("i2c-address").isValid())
            m_i2c->setAddress(static_cast<quint8>(child->property("i2c-address").toUInt()));
        else
            m_i2c->setAddress(static_cast<quint8>(0x68));
    }
}

QMPU6050AccelerometerBackend::~QMPU6050AccelerometerBackend()
{
    if(m_pollTimer)
    {
        if(m_pollTimer->isActive())
            m_pollTimer->stop();

        delete m_pollTimer;
    }

    QSensorBackend::~QSensorBackend();
}

void QMPU6050AccelerometerBackend::start()
{
    if(m_pollTimer && m_pollTimer->isActive())
        return;

    m_pollTimer->setInterval(1000 / sensor()->dataRate());
    m_pollTimer->start();
}

void QMPU6050AccelerometerBackend::stop()
{
    if(!m_pollTimer || !m_pollTimer->isActive())
        return;

    m_pollTimer->stop();
}

void QMPU6050AccelerometerBackend::poll()
{
    if(!m_i2c->start())
    {
        reportError("COULD NOT START I2C");
        handleFault();
        return;
    }

    quint8 *buffer = new quint8[6];

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_ACCEL_XOUT_H), buffer, 6))
    {
        m_i2c->end();
        delete [] buffer;
        handleFault();
        return;
    }

    qreal x = ((((qint16)buffer[0]) << 8) | buffer[1]);
    qreal y = ((((qint16)buffer[2]) << 8) | buffer[3]);
    qreal z = ((((qint16)buffer[4]) << 8) | buffer[5]);

    if(x > 32768)
        x = (x - 65536) / 16384.0;
    else
        x /= 16384.0;

    if(y > 32768)
        y = (y - 65536) / 16384.0;
    else
        y /= 16384.0;

    if(z > 32768)
        z = (z - 65536) / 16384.0;
    else
        z /= 16384.0;

    delete [] buffer;

    if(!m_i2c->end())
        handleFault();

    m_reading.setX(x);
    m_reading.setY(y);
    m_reading.setZ(z);

    newReadingAvailable();
}

void QMPU6050AccelerometerBackend::handleFault()
{
    //TODO
}

void QMPU6050AccelerometerBackend::reportEvent(QString message)
{
    //report event if backendDebug is true
    if(m_backendDebug)
        qDebug() << QString("** %1 - (QMPU6050@%2:0x%3)").arg(message, m_i2c->bus()).arg(m_i2c->address(), 2, 16);
}

void QMPU6050AccelerometerBackend::reportError(QString message)
{
    qDebug() << QString("!! ERROR: %1 - (QMPU6050@%2:0x%3)").arg(message, m_i2c->bus()).arg(m_i2c->address(), 2, 16);
}

void QMPU6050AccelerometerBackend::onSensorDataRateChanged()
{
    if(m_pollTimer->isActive())
    {
        stop();
        start();
    }
}
