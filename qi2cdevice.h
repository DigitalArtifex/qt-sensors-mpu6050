#ifndef QI2CDEVICE_H
#define QI2CDEVICE_H

#include <QObject>
#include <QDebug>
#include "qmpu6050_global.h"

#ifdef Q_OS_LINUX
#include "fcntl.h"
#include "i2c/smbus.h"
#include "linux/i2c-dev.h"
#include "linux/errno.h"
#include "sys/ioctl.h"
#endif

QT_BEGIN_NAMESPACE

class QMPU6_5__EXPORT QI2CDevice
{
public:
    QI2CDevice() = default;
    QI2CDevice(const QString &bus, const quint8 address);
    QI2CDevice(const QString &bus, const quint16 address);

    bool read(quint8 registerAddress, quint8 *buffer, quint16 length);
    bool read(quint16 registerAddress, quint8 *buffer, quint16 length);
    bool readBit(quint8 registerAddress, quint8 *buffer, quint8 bit);
    bool readBits(quint8 registerAddress, quint8 *buffer, quint8 startBit, quint8 bitWidth = 1);

    bool write(quint8 registerAddress, quint8 *buffer, quint16 length);
    bool write(quint16 registerAddress, quint8 *buffer, quint16 length);
    bool writeBit(quint8 registerAddress, quint8 bit, bool enabled);
    bool writeBits(quint8 registerAddress, quint8 buffer, quint8 startBit, quint8 bitWidth = 1);

    bool start();
    bool end();

    quint16 address() const;
    void setAddress(quint16 address);
    void setAddress(quint8 address);

    QString bus() const;
    void setBus(const QString &bus);

private:
    bool m_10BitAddress = false;
    quint16 m_address = 0x00;
    QString m_bus;
    int m_i2c = -1;
    int m_errno = 0;
};

QT_END_NAMESPACE
#endif // QI2CDEVICE_H
