#include "qi2cdevice.h"

QI2CDevice::QI2CDevice(const QString &bus, const quint8 address)
{
    m_10BitAddress = false;
    m_address = static_cast<quint16>(address);
    m_bus = bus;
}

QI2CDevice::QI2CDevice(const QString &bus, const quint16 address)
{
    m_10BitAddress = true;
    m_address = address;
    m_bus = bus;
}

bool QI2CDevice::read(quint8 registerAddress, quint8 *buffer, quint16 length)
{
#ifdef Q_OS_LINUX
    struct i2c_msg messages[]
    {
        {
            .addr = m_address,
            .flags = 0,
            .len = 1,
            .buf = &registerAddress
        },
        {
            .addr = m_address,
            .flags = I2C_M_RD,
            .len = length,
            .buf = buffer
        }
    };

    struct i2c_rdwr_ioctl_data payload =
    {
        .msgs = messages,
        .nmsgs = 2
    };

    if(ioctl(m_i2c, I2C_RDWR, &payload) < 0)
    {
        qDebug() << QString("COULD NOT READ REGISTER 0x%1").arg(registerAddress, 2, 16, '0');
        m_errno = errno;
        return false;
    }

    return true;
#else
    return false;
#endif
}

bool QI2CDevice::read(quint16 registerAddress, quint8 *buffer, quint16 length)
{
#ifdef Q_OS_LINUX
    quint8 *registerBuffer = new quint8[2]
    {
        static_cast<quint8>(registerAddress),
        static_cast<quint8>(registerAddress >> 8)
    };
    struct i2c_msg messages[]
    {
        {
            .addr = m_address,
            .flags = 0,
            .len = 2,
            .buf = registerBuffer
        },
        {
            .addr = m_address,
            .flags = I2C_M_RD,
            .len = length,
            .buf = buffer
        }
    };

    struct i2c_rdwr_ioctl_data payload =
    {
        .msgs = messages,
        .nmsgs = 2
    };

    if(ioctl(m_i2c, I2C_RDWR, &payload) < 0)
    {
        delete [] registerBuffer;
        qDebug() << QString("COULD NOT READ 16bit REGISTER 0x%1").arg(registerAddress, 2, 16, '0');
        m_errno = errno;
        return false;
    }

    delete [] registerBuffer;
    return true;
#else
    return false;
#endif
}

bool QI2CDevice::readBit(quint8 registerAddress, quint8 *buffer, quint8 bit)
{
    quint8 b;

    if(!read(registerAddress, &b, 1))
        return false;

    *buffer = b & (1 << bit);
    return true;
}

bool QI2CDevice::readBits(quint8 registerAddress, quint8 *buffer, quint8 startBit, quint8 bitWidth)
{
    if(!read(registerAddress, buffer, 1))
        return false;

    quint8 mask = ((1 << bitWidth) - 1) << (startBit - bitWidth + 1);
    *buffer &= mask;
    *buffer >>= (startBit - bitWidth + 1);

    return true;
}

bool QI2CDevice::write(quint8 registerAddress, quint8 *buffer, quint16 length)
{
#ifdef Q_OS_LINUX
    quint16 dataLength = strlen(reinterpret_cast<char*>(buffer));
    quint8 *data = new quint8[dataLength + 1];
    data[0] = registerAddress;
    memcpy(&data[1], buffer, dataLength);

    struct i2c_msg messages[]
    {
        {
            .addr = m_address,
            .flags = 0,
            .len = length + 1,
            .buf = data
        }
    };

    struct i2c_rdwr_ioctl_data payload =
    {
        .msgs = messages,
        .nmsgs = 1
    };

    if(ioctl(m_i2c, I2C_RDWR, &payload) < 0)
    {
        delete [] data;
        qDebug() << QString("COULD NOT WRITE REGISTER 0x%1").arg(registerAddress, 2, 16, '0');
        return false;
    }

    qDebug() << QString("WROTE 0x%1 TO REGISTER 0x%2").arg(*buffer, 2, 16, '0').arg(registerAddress, 2, 16, '0');
    delete [] data;
    return true;
#else
    return false;
#endif
}

bool QI2CDevice::write(quint16 registerAddress, quint8 *buffer, quint16 length)
{
#ifdef Q_OS_LINUX
    quint8 *registerBuffer = new quint8[2]
    {
        static_cast<quint8>(registerAddress),
        static_cast<quint8>(registerAddress >> 8)
    };

    struct i2c_msg messages[]
    {
        {
            .addr = m_address,
            .flags = 0,
            .len = 2,
            .buf = registerBuffer
        },
        {
            .addr = m_address,
            .flags = 0,
            .len = length,
            .buf = buffer
        }
    };

    struct i2c_rdwr_ioctl_data payload =
    {
        .msgs = messages,
        .nmsgs = 2
    };

    if(ioctl(m_i2c, I2C_RDWR, &payload) < 0)
    {
        delete [] registerBuffer;
        qDebug() << QString("COULD NOT WRITE REGISTER 0x%1").arg(registerAddress, 2, 16, '0');
        return false;
    }

    delete [] registerBuffer;
    return true;
#else
    return false;
#endif
}

bool QI2CDevice::writeBit(quint8 registerAddress, quint8 bit, bool enabled)
{
    quint8 b;

    if(!read(registerAddress, &b, 1))
        return false;

    b = enabled ? (b | (1 << bit)) : (b & ~(1 << bit));
    return write(registerAddress, &b, 1);
}

bool QI2CDevice::writeBits(quint8 registerAddress, quint8 buffer, quint8 startBit, quint8 bitWidth)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b = 0;
    if(!read(registerAddress, &b, 1))
        return false;

    uint8_t mask = ((1 << bitWidth) - 1) << (startBit - bitWidth + 1);
    buffer <<= (startBit - bitWidth + 1); // shift data into correct position
    buffer &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= buffer; // combine data with existing byte
    return write(registerAddress, &b, 1);
}

bool QI2CDevice::start()
{
#ifdef Q_OS_LINUX
    if(m_i2c < 0)
    {
        if((m_i2c = open(m_bus.toStdString().c_str(), O_RDWR)) < 0)
        {
            errno = EBADF;
            return false;
        }
    }

    if(ioctl(m_i2c, I2C_SLAVE, m_address) < 0)
    {
        errno = ENODEV;
        end();
        return false;
    }

    errno = 0; //errno = 2 after successful start. reset it for error handling

    return true;
#else
    return false;
#endif
}

bool QI2CDevice::end()
{
#ifdef Q_OS_LINUX
    if(m_i2c < 0)
        return true;

    if(close(m_i2c) < 0)
    {
        m_errno = errno;
        m_i2c = -1;
        return false;
    }

    m_i2c = -1;

    return true;
#else
    return false;
#endif
}

quint16 QI2CDevice::address() const
{
    return m_address;
}

void QI2CDevice::setAddress(quint16 address)
{
    m_10BitAddress = true;
    m_address = address;
}

void QI2CDevice::setAddress(quint8 address)
{
    m_10BitAddress = false;
    m_address = static_cast<quint16>(address);
}

QString QI2CDevice::bus() const
{
    return m_bus;
}

void QI2CDevice::setBus(const QString &bus)
{
    m_bus = bus;
}
