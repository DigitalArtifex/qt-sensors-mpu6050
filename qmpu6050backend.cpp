#include "qmpu6050backend.h"

QMPU6050Backend::QMPU6050Backend(QSensor *sensor)
    : QSensorBackend{sensor}
{
    m_pollTimer = new QTimer(this);
    QObject::connect(m_pollTimer, SIGNAL(timeout()), this, SLOT(poll()));

    QMPU6050 *child = qobject_cast<QMPU6050*>(sensor);

    if(child)
    {
        m_sensor = child;
        m_i2c = new QI2CDevice(child->bus(), child->address());

        reportEvent("QMPU6050 BACKEND CREATED");

        setReading<QAccelerometerReading>(&m_reading);
        reading();

        child->m_controller = this;

        addDataRate(1, 157);
    }
}

QMPU6050Backend::~QMPU6050Backend()
{
    if(m_pollTimer)
    {
        if(m_pollTimer->isActive())
            m_pollTimer->stop();

        delete m_pollTimer;
    }

    QSensorBackend::~QSensorBackend();
}

void QMPU6050Backend::start()
{
    if(m_pollTimer && m_pollTimer->isActive())
        return;

    m_pollTimer->setInterval(1000 / sensor()->dataRate());
    m_pollTimer->start();
}

void QMPU6050Backend::stop()
{
    if(!m_pollTimer || !m_pollTimer->isActive())
        return;

    m_pollTimer->stop();
}

bool QMPU6050Backend::isFeatureSupported(QSensor::Feature feature) const
{
    return false;
}

void QMPU6050Backend::poll()
{
    //start i2c
    quint16 count = 0;
    if(!m_i2c->start() || !get6AxisMotion() || !m_i2c->end())
    {
        handleFault();
        return;
    }

    m_reading.setX(m_ax);
    m_reading.setY(m_ay);
    m_reading.setZ(m_az);

    newReadingAvailable();
}

void QMPU6050Backend::handleFault()
{
    //TODO
}

void QMPU6050Backend::reportEvent(QString message)
{
    //report event if backendDebug is true
    if(m_backendDebug)
        qDebug() << QString("** %1 - (QMPU6050@%2:0x%3)").arg(message, m_bus).arg(m_address, 2, 16);
}

void QMPU6050Backend::reportError(QString message)
{
    qDebug() << QString("!! ERROR: %1 - (QBMP280@%2:0x%3)").arg(message, m_bus).arg(m_address, 2, 16);
}

void QMPU6050Backend::newLine()
{
    //add newline if backendDebug is true
    if(m_backendDebug)
        qDebug() << "";
}

void QMPU6050Backend::onSensorBusChanged()
{
    QMPU6050 *sensor = qobject_cast<QMPU6050*>(this->sensor());

    if(!sensor)
        return;

    m_i2c->setBus(sensor->bus());
}

void QMPU6050Backend::onSensorAddressChanged()
{
    QMPU6050 *sensor = qobject_cast<QMPU6050*>(this->sensor());

    if(!sensor)
        return;

    m_i2c->setAddress(sensor->address());
}

void QMPU6050Backend::onSesnorDataRateChanged()
{
    stop();
    start();
}

void QMPU6050Backend::setAddress(quint8 address)
{
    m_i2c->setAddress(address);
}

void QMPU6050Backend::setBus(const QString &bus)
{
    m_i2c->setBus(bus);
}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
bool QMPU6050Backend::initialize()
{
    QMPU6050 *sensor = qobject_cast<QMPU6050*>(this->sensor());

    if(!sensor)
        return false;

    m_i2c->setBus(sensor->bus());
    m_i2c->setAddress(sensor->address());

    if(!m_i2c->start())
        return false;

    if(!testConnection())
    {
        reportError("COULD NOT FIND DEVICE");
        return false;
    }

    quint8 gyro = 7;
    if(!m_i2c->write(static_cast<quint8>(0x19), &gyro, 1))
    {
        reportError("COULD SET GYRO RANGE");
        return false;
    }

    gyro = 1;
    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), &gyro, 1))
    {
        reportError("COULD SET GYRO RANGE");
        return false;
    }

    gyro = 0;
    if(!m_i2c->write(static_cast<quint8>(0x1A), &gyro, 1))
    {
        reportError("COULD SET CONFIG");
        return false;
    }

    gyro = 24;
    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_GYRO_CONFIG), &gyro, 1))
    {
        reportError("COULD SET GYRO RANGE");
        return false;
    }

    gyro = 1;
    if(!m_i2c->write(static_cast<quint8>(0x38), &gyro, 1))
    {
        reportError("COULD SET GYRO RANGE");
        return false;
    }

    // if(!setFullScaleAccelRange(MPU6050_ACCEL_FS_2))
    // {
    //     reportError("COULD SET ACCEL RANGE");
    //     return false;
    // }

    // setIntDataReadyEnabled(true);

    if(!m_i2c->end())
        return false;

    m_initialized = true;

    QObject::connect(sensor, &QMPU6050::busChanged, this, &QMPU6050Backend::onSensorBusChanged);
    QObject::connect(sensor, &QMPU6050::addressChanged, this, &QMPU6050Backend::onSensorAddressChanged);
    QObject::connect(sensor, &QMPU6050::dataRateChanged, this, &QMPU6050Backend::onSesnorDataRateChanged);

    return true;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool QMPU6050Backend::testConnection() {
    quint8 id = 0;

    if(!getDeviceID(&id))
        return false;

    return id == 0b110100;
}

// AUX_VDDIO register (InvenSense demo code calls this RA_*G_OFFS_TC)

/** Get the auxiliary I2C supply voltage level.
 * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
 * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
 * the MPU-6000, which does not have a VLOGIC pin.
 * @return I2C supply voltage level (0=VLOGIC, 1=VDD)
 */
bool QMPU6050Backend::getAuxVDDIOLevel()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_YG_OFFS_TC), &buffer, static_cast<quint8>(MPU6050_TC_PWR_MODE_BIT)))
        return false;

    bool enabled = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isAccelerometerSelfTestYEnabled != enabled)
    {
        m_sensor->m_isAccelerometerSelfTestXEnabled = enabled;
        emit m_sensor->accelerometerSelfTestXEnabledChanged();
    }

    return true;
}
/** Set the auxiliary I2C supply voltage level.
 * When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
 * 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
 * the MPU-6000, which does not have a VLOGIC pin.
 * @param level I2C supply voltage level (0=VLOGIC, 1=VDD)
 */
bool QMPU6050Backend::setAuxVDDIOLevel(quint8 level)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_YG_OFFS_TC), static_cast<quint8>(MPU6050_TC_PWR_MODE_BIT), static_cast<bool>(level));
}

// SMPLRT_DIV register

/** Get gyroscope output rate divider.
 * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
 * Motion detection, and Free Fall detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current sample rate
 * @see MPU6050_RA_SMPLRT_DIV
 */
bool QMPU6050Backend::getRate()
{
    quint8 buffer;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_SMPLRT_DIV), &buffer, 1))
        return false;

    if(m_sensor && m_sensor->m_gyroscopeRateDivider != buffer)
    {
        m_sensor->m_gyroscopeRateDivider = buffer;
        emit m_sensor->gyroscopeRateDividerChanged();
    }

    return false;
}
/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
bool QMPU6050Backend::setRate(quint8 rate)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_SMPLRT_DIV), &rate, 1);
}

// CONFIG register

/** Get external FSYNC configuration.
 * Configures the external Frame Synchronization (FSYNC) pin sampling. An
 * external signal connected to the FSYNC pin can be sampled by configuring
 * EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
 * strobes may be captured. The latched FSYNC signal will be sampled at the
 * Sampling Rate, as defined in register 25. After sampling, the latch will
 * reset to the current FSYNC signal state.
 *
 * The sampled value will be reported in place of the least significant bit in
 * a sensor data register determined by the value of EXT_SYNC_SET according to
 * the following table.
 *
 * <pre>
 * EXT_SYNC_SET | FSYNC Bit Location
 * -------------+-------------------
 * 0            | Input disabled
 * 1            | TEMP_OUT_L[0]
 * 2            | GYRO_XOUT_L[0]
 * 3            | GYRO_YOUT_L[0]
 * 4            | GYRO_ZOUT_L[0]
 * 5            | ACCEL_XOUT_L[0]
 * 6            | ACCEL_YOUT_L[0]
 * 7            | ACCEL_ZOUT_L[0]
 * </pre>
 *
 * @return FSYNC configuration value
 */
bool QMPU6050Backend::getExternalFrameSync()
{
    quint8 buffer;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_CONFIG), &buffer, static_cast<quint8>(MPU6050_CFG_EXT_SYNC_SET_BIT),static_cast<quint8>(MPU6050_CFG_EXT_SYNC_SET_LENGTH)))
        return false;

    if(m_sensor && m_sensor->m_externalFrameSync != static_cast<QMPU6050::ExternalFrameSync>(buffer))
    {
        m_sensor->m_externalFrameSync = static_cast<QMPU6050::ExternalFrameSync>(buffer);
        emit m_sensor->externalFrameSyncChanged();
    }

    return true;
}
/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
bool QMPU6050Backend::setExternalFrameSync(quint8 sync)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_CONFIG), sync, static_cast<quint8>(MPU6050_CFG_EXT_SYNC_SET_BIT), static_cast<quint8>(MPU6050_CFG_EXT_SYNC_SET_LENGTH));
}
/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
bool QMPU6050Backend::getDLPFMode()
{
    quint8 buffer;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_CONFIG), &buffer, static_cast<quint8>(MPU6050_CFG_DLPF_CFG_BIT),static_cast<quint8>(MPU6050_CFG_DLPF_CFG_LENGTH)))
        return false;

    if(m_sensor && m_sensor->m_dlpFilterMode != static_cast<QMPU6050::DLPFilterMode>(buffer))
    {
        m_sensor->m_dlpFilterMode = static_cast<QMPU6050::DLPFilterMode>(buffer);
        emit m_sensor->dlpFilterModeChanged();
    }

    return true;
}
/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
bool QMPU6050Backend::setDLPFMode(quint8 mode)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_CONFIG), mode, static_cast<quint8>(MPU6050_CFG_DLPF_CFG_BIT), static_cast<quint8>(MPU6050_CFG_DLPF_CFG_LENGTH));
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
bool QMPU6050Backend::getFullScaleGyroRange()
{
    quint8 buffer;
    quint16 result = 0;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_GYRO_CONFIG), &buffer, static_cast<quint8>(MPU6050_GCONFIG_FS_SEL_BIT),static_cast<quint8>(MPU6050_GCONFIG_FS_SEL_LENGTH)))
        return false;

    switch (buffer)
    {
    case 0:
        result = 250;
        break;
    case 1:
        result = 500;
        break;
    case 2:
        result = 1000;
        break;
    case 3:
        result = 2000;
        break;
    default:
        break;
    }

    if(m_sensor && m_sensor->m_fullScaleGyroscopeRange != result)
    {
        m_sensor->m_fullScaleGyroscopeRange = result;
        emit m_sensor->fullScaleGyroscopeRangeChanged();
    }

    return true;
}
/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
bool QMPU6050Backend::setFullScaleGyroRange(quint8 range)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_GYRO_CONFIG), range, static_cast<quint8>(MPU6050_GCONFIG_FS_SEL_BIT), static_cast<quint8>(MPU6050_GCONFIG_FS_SEL_LENGTH));
}

// ACCEL_CONFIG register

/** Get self-test enabled setting for accelerometer X axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool QMPU6050Backend::getAccelXSelfTest()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), &buffer, static_cast<quint8>(MPU6050_ACONFIG_XA_ST_BIT)))
        return false;

    bool enabled = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isAccelerometerSelfTestYEnabled != enabled)
    {
        m_sensor->m_isAccelerometerSelfTestXEnabled = enabled;
        emit m_sensor->accelerometerSelfTestXEnabledChanged();
    }

    return true;
}
/** Get self-test enabled setting for accelerometer X axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool QMPU6050Backend::setAccelXSelfTest(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), static_cast<quint8>(MPU6050_ACONFIG_XA_ST_BIT), enabled);
}
/** Get self-test enabled value for accelerometer Y axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool QMPU6050Backend::getAccelYSelfTest()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), &buffer, static_cast<quint8>(MPU6050_ACONFIG_YA_ST_BIT)))
        return false;

    bool enabled = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isAccelerometerSelfTestYEnabled != enabled)
    {
        m_sensor->m_isAccelerometerSelfTestYEnabled = enabled;
        emit m_sensor->accelerometerSelfTestYEnabledChanged();
    }

    return true;
}
/** Get self-test enabled value for accelerometer Y axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool QMPU6050Backend::setAccelYSelfTest(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), static_cast<quint8>(MPU6050_ACONFIG_YA_ST_BIT), enabled);
}
/** Get self-test enabled value for accelerometer Z axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool QMPU6050Backend::getAccelZSelfTest()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), &buffer, static_cast<quint8>(MPU6050_ACONFIG_ZA_ST_BIT)))
        return false;

    bool enabled = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isAccelerometerSelfTestZEnabled != enabled)
    {
        m_sensor->m_isAccelerometerSelfTestZEnabled = enabled;
        emit m_sensor->accelerometerSelfTestZEnabledChanged();
    }

    return true;
}

/** Set self-test enabled value for accelerometer Z axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool QMPU6050Backend::setAccelZSelfTest(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), static_cast<quint8>(MPU6050_ACONFIG_ZA_ST_BIT), enabled);
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
bool QMPU6050Backend::getFullScaleAccelRange()
{
    quint8 buffer;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), &buffer, static_cast<quint8>(MPU6050_ACONFIG_AFS_SEL_BIT),static_cast<quint8>(MPU6050_ACONFIG_AFS_SEL_LENGTH)))
        return false;

    switch (buffer)
    {
    case 0:
        buffer = 2;
        break;
    case 1:
        buffer = 4;
        break;
    case 2:
        buffer = 8;
        break;
    case 3:
        buffer = 16;
        break;
    default:
        break;
    }

    if(m_sensor && m_sensor->m_fullScaleAccerometerRange != buffer)
    {
        m_sensor->m_fullScaleAccerometerRange = buffer;
        emit m_sensor->fullScaleAccerometerRangeChanged();
    }

    return true;
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
bool QMPU6050Backend::setFullScaleAccelRange(quint8 range)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), range, static_cast<quint8>(MPU6050_ACONFIG_AFS_SEL_BIT), static_cast<quint8>(MPU6050_ACONFIG_AFS_SEL_LENGTH));
}
/** Get the high-pass filter configuration.
 * The DHPF is a filter module in the path leading to motion detectors (Free
 * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
 * available to the data registers (see Figure in Section 8 of the MPU-6000/
 * MPU-6050 Product Specification document).
 *
 * The high pass filter has three modes:
 *
 * <pre>
 *    Reset: The filter output settles to zero within one sample. This
 *           effectively disables the high pass filter. This mode may be toggled
 *           to quickly settle the filter.
 *
 *    On:    The high pass filter will pass signals above the cut off frequency.
 *
 *    Hold:  When triggered, the filter holds the present sample. The filter
 *           output will be the difference between the input sample and the held
 *           sample.
 * </pre>
 *
 * <pre>
 * ACCEL_HPF | Filter Mode | Cut-off Frequency
 * ----------+-------------+------------------
 * 0         | Reset       | None
 * 1         | On          | 5Hz
 * 2         | On          | 2.5Hz
 * 3         | On          | 1.25Hz
 * 4         | On          | 0.63Hz
 * 7         | Hold        | None
 * </pre>
 *
 * @return Current high-pass filter configuration
 * @see MPU6050_DHPF_RESET
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool QMPU6050Backend::getDHPFMode()
{
    quint8 buffer;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), &buffer, static_cast<quint8>(MPU6050_ACONFIG_ACCEL_HPF_BIT),static_cast<quint8>(MPU6050_ACONFIG_ACCEL_HPF_LENGTH)))
        return false;

    QMPU6050::DHPFilterMode result = static_cast<QMPU6050::DHPFilterMode>(buffer);

    if(m_sensor && m_sensor->m_dhpFilterMode != result)
    {
        m_sensor->m_dhpFilterMode = result;
        emit m_sensor->dhpFilterModeChanged();
    }

    return true;
}
/** Set the high-pass filter configuration.
 * @param bandwidth New high-pass filter configuration
 * @see setDHPFMode()
 * @see MPU6050_DHPF_RESET
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool QMPU6050Backend::setDHPFMode(quint8 bandwidth)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_ACCEL_CONFIG), bandwidth, static_cast<quint8>(MPU6050_ACONFIG_ACCEL_HPF_BIT), static_cast<quint8>(MPU6050_ACONFIG_ACCEL_HPF_LENGTH));
}

// FF_THR register

/** Get free-fall event acceleration threshold.
 * This register configures the detection threshold for Free Fall event
 * detection. The unit of FF_THR is 1LSB = 2mg. Free Fall is detected when the
 * absolute value of the accelerometer measurements for the three axes are each
 * less than the detection threshold. This condition increments the Free Fall
 * duration counter (Register 30). The Free Fall interrupt is triggered when the
 * Free Fall duration counter reaches the time specified in FF_DUR.
 *
 * For more details on the Free Fall detection interrupt, see Section 8.2 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @return Current free-fall acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_FF_THR
 */
bool QMPU6050Backend::getFreefallDetectionThreshold()
{
    quint8 buffer;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_FF_THR), &buffer, 1))
        return false;

    qint8 result = static_cast<qint8>(buffer);

    if(m_sensor && m_sensor->m_freefallDetectionThreshold != result)
    {
        m_sensor->m_freefallDetectionThreshold = result;
        emit m_sensor->freefallDetectionThresholdChanged();
    }

    return true;
}
/** Get free-fall event acceleration threshold.
 * @param threshold New free-fall acceleration threshold value (LSB = 2mg)
 * @see getFreefallDetectionThreshold()
 * @see MPU6050_RA_FF_THR
 */
bool QMPU6050Backend::setFreefallDetectionThreshold(quint8 threshold)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_FF_THR), &threshold, 1);
}

// FF_DUR register

/** Get free-fall event duration threshold.
 * This register configures the duration counter threshold for Free Fall event
 * detection. The duration counter ticks at 1kHz, therefore FF_DUR has a unit
 * of 1 LSB = 1 ms.
 *
 * The Free Fall duration counter increments while the absolute value of the
 * accelerometer measurements are each less than the detection threshold
 * (Register 29). The Free Fall interrupt is triggered when the Free Fall
 * duration counter reaches the time specified in this register.
 *
 * For more details on the Free Fall detection interrupt, see Section 8.2 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @return Current free-fall duration threshold value (LSB = 1ms)
 * @see MPU6050_RA_FF_DUR
 */
bool QMPU6050Backend::getFreefallDetectionDuration()
{
    quint8 buffer;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_FF_DUR), &buffer, 1))
        return false;

    qint8 result = static_cast<qint8>(buffer);

    if(m_sensor && m_sensor->m_freefallDetectionDuration != result)
    {
        m_sensor->m_freefallDetectionDuration = result;
        emit m_sensor->freefallDetectionDurationChanged();
    }

    return true;
}
/** Get free-fall event duration threshold.
 * @param duration New free-fall duration threshold value (LSB = 1ms)
 * @see getFreefallDetectionDuration()
 * @see MPU6050_RA_FF_DUR
 */
bool QMPU6050Backend::setFreefallDetectionDuration(quint8 duration)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_FF_DUR), &duration, 1);
}

// MOT_THR register

/** Get motion detection event acceleration threshold.
 * This register configures the detection threshold for Motion interrupt
 * generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
 * absolute value of any of the accelerometer measurements exceeds this Motion
 * detection threshold. This condition increments the Motion detection duration
 * counter (Register 32). The Motion detection interrupt is triggered when the
 * Motion Detection counter reaches the time count specified in MOT_DUR
 * (Register 32).
 *
 * The Motion interrupt will indicate the axis and polarity of detected motion
 * in MOT_DETECT_STATUS (Register 97).
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @return Current motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_MOT_THR
 */
bool QMPU6050Backend::getMotionDetectionThreshold()
{
    quint8 buffer;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_MOT_THR), &buffer, 1))
        return false;

    qint8 result = static_cast<qint8>(buffer);

    if(m_sensor && m_sensor->m_motionDetectionThreshold != result)
    {
        m_sensor->m_motionDetectionThreshold = result;
        emit m_sensor->motionDetectionThresholdChanged();
    }

    return true;
}
/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
bool QMPU6050Backend::setMotionDetectionThreshold(quint8 threshold)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_MOT_THR), &threshold, 1);
}

// MOT_DUR register

/** Get motion detection event duration threshold.
 * This register configures the duration counter threshold for Motion interrupt
 * generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
 * of 1LSB = 1ms. The Motion detection duration counter increments when the
 * absolute value of any of the accelerometer measurements exceeds the Motion
 * detection threshold (Register 31). The Motion detection interrupt is
 * triggered when the Motion detection counter reaches the time count specified
 * in this register.
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current motion detection duration threshold value (LSB = 1ms)
 * @see MPU6050_RA_MOT_DUR
 */
bool QMPU6050Backend::getMotionDetectionDuration()
{
    quint8 buffer;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_MOT_DUR), &buffer, 1))
        return false;

    qint8 result = static_cast<qint8>(buffer);

    if(m_sensor && m_sensor->m_motionDetectionDuration != result)
    {
        m_sensor->m_motionDetectionDuration = result;
        emit m_sensor->motionDetectionDurationChanged();
    }

    return true;
}
/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
bool QMPU6050Backend::setMotionDetectionDuration(quint8 duration)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_MOT_DUR), &duration, 1);
}

// ZRMOT_THR register

/** Get zero motion detection event acceleration threshold.
 * This register configures the detection threshold for Zero Motion interrupt
 * generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
 * the absolute value of the accelerometer measurements for the 3 axes are each
 * less than the detection threshold. This condition increments the Zero Motion
 * duration counter (Register 34). The Zero Motion interrupt is triggered when
 * the Zero Motion duration counter reaches the time count specified in
 * ZRMOT_DUR (Register 34).
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an
 * interrupt both when Zero Motion is first detected and when Zero Motion is no
 * longer detected.
 *
 * When a zero motion event is detected, a Zero Motion Status will be indicated
 * in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
 * condition is detected, the status bit is set to 1. When a zero-motion-to-
 * motion condition is detected, the status bit is set to 0.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_ZRMOT_THR
 */
bool QMPU6050Backend::getZeroMotionDetectionThreshold()
{
    quint8 buffer;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_ZRMOT_THR), &buffer, 1))
        return false;

    qint8 result = static_cast<qint8>(buffer);

    if(m_sensor && m_sensor->m_zeroMotionDetectionThreshold != result)
    {
        m_sensor->m_zeroMotionDetectionThreshold = result;
        emit m_sensor->zeroMotionDetectionThresholdChanged();
    }

    return true;
}
/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
bool QMPU6050Backend::setZeroMotionDetectionThreshold(quint8 threshold)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_ZRMOT_THR), &threshold, 1);
}

// ZRMOT_DUR register

/** Get zero motion detection event duration threshold.
 * This register configures the duration counter threshold for Zero Motion
 * interrupt generation. The duration counter ticks at 16 Hz, therefore
 * ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
 * increments while the absolute value of the accelerometer measurements are
 * each less than the detection threshold (Register 33). The Zero Motion
 * interrupt is triggered when the Zero Motion duration counter reaches the time
 * count specified in this register.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document, as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection duration threshold value (LSB = 64ms)
 * @see MPU6050_RA_ZRMOT_DUR
 */
bool QMPU6050Backend::getZeroMotionDetectionDuration()
{
    quint8 buffer;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_ZRMOT_DUR), &buffer, 1))
        return false;

    qint8 result = static_cast<qint8>(buffer);

    if(m_sensor && m_sensor->m_zeroMotionDetectionDuration != result)
    {
        m_sensor->m_zeroMotionDetectionDuration = result;
        emit m_sensor->zeroMotionDetectionDurationChanged();
    }

    return true;
}
/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
bool QMPU6050Backend::setZeroMotionDetectionDuration(quint8 duration)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_ZRMOT_DUR), &duration, 1);
}

// FIFO_EN register
/** Set temperature FIFO enabled value.
 * @param enabled New temperature FIFO enabled value
 * @see getTempFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
bool QMPU6050Backend::setTempFIFOEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_FIFO_EN), static_cast<quint8>(MPU6050_TEMP_FIFO_EN_BIT), enabled);
}
/** Set gyroscope X-axis FIFO enabled value.
 * @param enabled New gyroscope X-axis FIFO enabled value
 * @see getXGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
bool QMPU6050Backend::setXGyroFIFOEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_FIFO_EN), static_cast<quint8>(MPU6050_XG_FIFO_EN_BIT), enabled);
}
/** Set gyroscope Y-axis FIFO enabled value.
 * @param enabled New gyroscope Y-axis FIFO enabled value
 * @see getYGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
bool QMPU6050Backend::setYGyroFIFOEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_FIFO_EN), static_cast<quint8>(MPU6050_YG_FIFO_EN_BIT), enabled);
}
/** Set gyroscope Z-axis FIFO enabled value.
 * @param enabled New gyroscope Z-axis FIFO enabled value
 * @see getZGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
bool QMPU6050Backend::setZGyroFIFOEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_FIFO_EN), static_cast<quint8>(MPU6050_ZG_FIFO_EN_BIT), enabled);
}
/** Set accelerometer FIFO enabled value.
 * @param enabled New accelerometer FIFO enabled value
 * @see getAccelFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
bool QMPU6050Backend::setAccelFIFOEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_FIFO_EN), static_cast<quint8>(MPU6050_ACCEL_FIFO_EN_BIT), enabled);
}

// INT_ENABLE register
/** Set FIFO Buffer Overflow interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFIFOBufferOverflowEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 **/
bool QMPU6050Backend::setIntFIFOBufferOverflowEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_INT_ENABLE), static_cast<quint8>(MPU6050_INTERRUPT_FIFO_OFLOW_BIT), enabled);
}
/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see MPU6050_RA_INT_CFG
 * @see MPU6050_INTERRUPT_DATA_RDY_BIT
 */
bool QMPU6050Backend::setIntDataReadyEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_INT_ENABLE), static_cast<quint8>(MPU6050_INTERRUPT_DATA_RDY_BIT), enabled);
}

// INT_STATUS register
/** Get FIFO Buffer Overflow interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 */
bool QMPU6050Backend::getIntFIFOBufferOverflowStatus()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)))
        return false;

    m_fifoOverflow = static_cast<bool>(buffer);

    return true;
}
/** Get Data Ready interrupt status.
 * This bit automatically sets to 1 when a Data Ready interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_DATA_RDY_BIT
 */
bool QMPU6050Backend::getIntDataReadyStatus()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_INTERRUPT_DATA_RDY_BIT)))
        return false;

    m_dataReady = static_cast<bool>(buffer);

    return true;
}

// ACCEL_*OUT_* registers

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @param mx 16-bit signed integer container for magnetometer X-axis value
 * @param my 16-bit signed integer container for magnetometer Y-axis value
 * @param mz 16-bit signed integer container for magnetometer Z-axis value
 * @see getMotion6()
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
bool QMPU6050Backend::get9AxisMotion()
{
    return get6AxisMotion();
    // TODO: magnetometer integration
}
/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
bool QMPU6050Backend::get6AxisMotion()
{
    quint8 *buffer = new quint8[14];

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_ACCEL_XOUT_H), buffer, 14))
    {
        delete [] buffer;
        return false;
    }

    m_ax = ((((qint16)buffer[0]) << 8) | buffer[1]);
    m_ay = ((((qint16)buffer[2]) << 8) | buffer[3]);
    m_az = ((((qint16)buffer[4]) << 8) | buffer[5]);

    if(m_ax > 32768)
        m_ax = (m_ax - 65536) / 16384.0;
    else
        m_ax /= 16384.0;

    if(m_ay > 32768)
        m_ay = (m_ay - 65536) / 16384.0;
    else
        m_ay /= 16384.0;

    if(m_az > 32768)
        m_az = (m_az - 65536) / 16384.0;
    else
        m_az /= 16384.0;

    m_gx = ((((qint16)buffer[8]) << 8) | buffer[9]) / 131;
    m_gy = ((((qint16)buffer[10]) << 8) | buffer[11]) / 131;
    m_gz = ((((qint16)buffer[12]) << 8) | buffer[13]) / 131;

    delete [] buffer;

    return true;
}
/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU6050_RA_GYRO_XOUT_H
 */
bool QMPU6050Backend::getAcceleration()
{
    quint8 *buffer = new quint8[6];

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_ACCEL_XOUT_H), buffer, 6))
    {
        delete [] buffer;
        return false;
    }

    qint16 x = (((qint16)buffer[0]) << 8) | buffer[1];
    qint16 y = (((qint16)buffer[2]) << 8) | buffer[3];
    qint16 z = (((qint16)buffer[4]) << 8) | buffer[5];
    delete [] buffer;

    if(m_sensor && m_sensor->m_xAcceleration != x)
    {
        m_sensor->m_xAcceleration = x;
        emit m_sensor->xAccelerationChanged();
    }

    if(m_sensor && m_sensor->m_yAcceleration != y)
    {
        m_sensor->m_yAcceleration = y;
        emit m_sensor->yAccelerationChanged();
    }

    if(m_sensor && m_sensor->m_zAcceleration != z)
    {
        m_sensor->m_zAcceleration = z;
        emit m_sensor->zAccelerationChanged();
    }

    return true;
}

// TEMP_OUT_* registers

/** Get current internal temperature.
 * @return Temperature reading in 16-bit 2's complement format
 * @see MPU6050_RA_TEMP_OUT_H
 */
bool QMPU6050Backend::getTemperature()
{
    quint8 *buffer = new quint8[2];

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_TEMP_OUT_H), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    qint16 temperature = (((qint16)buffer[0]) << 8) | buffer[1];
    delete [] buffer;

    if(m_sensor && m_sensor->m_temperature != temperature)
    {
        m_sensor->m_temperature = temperature;
        emit m_sensor->temperatureChanged();
    }

    return true;
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
bool QMPU6050Backend::getRotation()
{
    quint8 *buffer = new quint8[6];

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_GYRO_XOUT_H), buffer, 6))
    {
        delete [] buffer;
        return false;
    }

    qint16 x = (((qint16)buffer[0]) << 8) | buffer[1];
    qint16 y = (((qint16)buffer[2]) << 8) | buffer[3];
    qint16 z = (((qint16)buffer[4]) << 8) | buffer[5];
    delete [] buffer;

    if(m_sensor && m_sensor->m_xRotation != x)
    {
        m_sensor->m_xRotation = x;
        emit m_sensor->xRotationChanged();
    }

    if(m_sensor && m_sensor->m_yRotation != y)
    {
        m_sensor->m_yRotation = y;
        emit m_sensor->yRotationChanged();
    }

    if(m_sensor && m_sensor->m_zRotation != z)
    {
        m_sensor->m_zRotation = z;
        emit m_sensor->zRotationChanged();
    }

    return true;
}

// MOT_DETECT_STATUS register

/** Get X-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_XNEG_BIT
 */
bool QMPU6050Backend::getXNegMotionDetected()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_MOT_DETECT_STATUS), &buffer, static_cast<quint8>(MPU6050_MOTION_MOT_XNEG_BIT)))
        return false;

    bool detected = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isXPositiveMotionDetected != detected)
    {
        m_sensor->m_isXPositiveMotionDetected = detected;
        emit m_sensor->xPositiveMotionDetectedChanged();
    }

    return true;
}
/** Get X-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_XPOS_BIT
 */
bool QMPU6050Backend::getXPosMotionDetected()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_MOT_DETECT_STATUS), &buffer, static_cast<quint8>(MPU6050_MOTION_MOT_XPOS_BIT)))
        return false;

    bool detected = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isXPositiveMotionDetected != detected)
    {
        m_sensor->m_isXPositiveMotionDetected = detected;
        emit m_sensor->xPositiveMotionDetectedChanged();
    }

    return true;
}
/** Get Y-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_YNEG_BIT
 */
bool QMPU6050Backend::getYNegMotionDetected()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_MOT_DETECT_STATUS), &buffer, static_cast<quint8>(MPU6050_MOTION_MOT_YNEG_BIT)))
        return false;

    bool detected = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isYPositiveMotionDetected != detected)
    {
        m_sensor->m_isYPositiveMotionDetected = detected;
        emit m_sensor->yPositiveMotionDetectedChanged();
    }

    return true;
}
/** Get Y-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_YPOS_BIT
 */
bool QMPU6050Backend::getYPosMotionDetected()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_MOT_DETECT_STATUS), &buffer, static_cast<quint8>(MPU6050_MOTION_MOT_YPOS_BIT)))
        return false;

    bool detected = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isYPositiveMotionDetected != detected)
    {
        m_sensor->m_isYPositiveMotionDetected = detected;
        emit m_sensor->yPositiveMotionDetectedChanged();
    }

    return true;
}
/** Get Z-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZNEG_BIT
 */
bool QMPU6050Backend::getZNegMotionDetected()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_MOT_DETECT_STATUS), &buffer, static_cast<quint8>(MPU6050_MOTION_MOT_ZNEG_BIT)))
        return false;

    bool detected = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isZPositiveMotionDetected != detected)
    {
        m_sensor->m_isZPositiveMotionDetected = detected;
        emit m_sensor->zPositiveMotionDetectedChanged();
    }

    return true;
}
/** Get Z-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZPOS_BIT
 */
bool QMPU6050Backend::getZPosMotionDetected()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_MOT_DETECT_STATUS), &buffer, static_cast<quint8>(MPU6050_MOTION_MOT_ZPOS_BIT)))
        return false;

    bool detected = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isZPositiveMotionDetected != detected)
    {
        m_sensor->m_isZPositiveMotionDetected = detected;
        emit m_sensor->zPositiveMotionDetectedChanged();
    }

    return true;
}
/** Get zero motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZRMOT_BIT
 */
bool QMPU6050Backend::getZeroMotionDetected()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_MOT_DETECT_STATUS), &buffer, static_cast<quint8>(MPU6050_MOTION_MOT_ZRMOT_BIT)))
        return false;

    bool detected = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isZeroMotionDetected != detected)
    {
        m_sensor->m_isZeroMotionDetected = detected;
        emit m_sensor->zeroMotionDetectedChanged();
    }

    return true;
}

// SIGNAL_PATH_RESET register

/** Reset gyroscope signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU6050_RA_SIGNAL_PATH_RESET
 * @see MPU6050_PATHRESET_GYRO_RESET_BIT
 */
bool QMPU6050Backend::resetGyroscopePath()
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_SIGNAL_PATH_RESET), static_cast<quint8>(MPU6050_PATHRESET_GYRO_RESET_BIT), true);
}
/** Reset accelerometer signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU6050_RA_SIGNAL_PATH_RESET
 * @see MPU6050_PATHRESET_ACCEL_RESET_BIT
 */
bool QMPU6050Backend::resetAccelerometerPath()
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_SIGNAL_PATH_RESET), static_cast<quint8>(MPU6050_PATHRESET_ACCEL_RESET_BIT), true);
}
/** Reset temperature sensor signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU6050_RA_SIGNAL_PATH_RESET
 * @see MPU6050_PATHRESET_TEMP_RESET_BIT
 */
bool QMPU6050Backend::resetTemperaturePath()
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_SIGNAL_PATH_RESET), static_cast<quint8>(MPU6050_PATHRESET_TEMP_RESET_BIT), true);
}

// MOT_DETECT_CTRL register

/** Get accelerometer power-on delay.
 * The accelerometer data path provides samples to the sensor registers, Motion
 * detection, Zero Motion detection, and Free Fall detection modules. The
 * signal path contains filters which must be flushed on wake-up with new
 * samples before the detection modules begin operations. The default wake-up
 * delay, of 4ms can be lengthened by up to 3ms. This additional delay is
 * specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
 * any value above zero unless instructed otherwise by InvenSense. Please refer
 * to Section 8 of the MPU-6000/MPU-6050 Product Specification document for
 * further information regarding the detection modules.
 * @return Current accelerometer power-on delay
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_ACCEL_ON_DELAY_BIT
 */
bool QMPU6050Backend::getAccelerometerPowerOnDelay()
{
    quint8 buffer;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_MOT_DETECT_CTRL), &buffer, static_cast<quint8>(MPU6050_DETECT_ACCEL_ON_DELAY_BIT), static_cast<quint8>(MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH)))
        return false;

    if(m_sensor && m_sensor->m_accelerometerPowerOnDelay != buffer)
    {
        m_sensor->m_accelerometerPowerOnDelay = buffer;
        emit m_sensor->freefallDetectionCounterDecrementChanged();
    }

    return true;
}
/** Set accelerometer power-on delay.
 * @param delay New accelerometer power-on delay (0-3)
 * @see getAccelerometerPowerOnDelay()
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_ACCEL_ON_DELAY_BIT
 */
bool QMPU6050Backend::setAccelerometerPowerOnDelay(quint8 delay)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_MOT_DETECT_CTRL), delay, static_cast<quint8>(MPU6050_DETECT_ACCEL_ON_DELAY_BIT), static_cast<quint8>(MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH));
}
/** Get Free Fall detection counter decrement configuration.
 * Detection is registered by the Free Fall detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring FF_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * FF_COUNT | Counter Decrement
 * ---------+------------------
 * 0        | Reset
 * 1        | 1
 * 2        | 2
 * 3        | 4
 * </pre>
 *
 * When FF_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Free Fall detection,
 * please refer to Registers 29 to 32.
 *
 * @return Current decrement configuration
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_FF_COUNT_BIT
 */
bool QMPU6050Backend::getFreefallDetectionCounterDecrement()
{
    quint8 buffer;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_MOT_DETECT_CTRL), &buffer, static_cast<quint8>(MPU6050_DETECT_FF_COUNT_BIT), static_cast<quint8>(MPU6050_DETECT_FF_COUNT_LENGTH)))
        return false;

    if(m_sensor && static_cast<quint8>(m_sensor->m_freefallDetectionCounterDecrement) != buffer)
    {
        m_sensor->m_freefallDetectionCounterDecrement = static_cast<QMPU6050::CounterDecrement>(buffer);
        emit m_sensor->freefallDetectionCounterDecrementChanged();
    }

    return true;
}
/** Set Free Fall detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getFreefallDetectionCounterDecrement()
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_FF_COUNT_BIT
 */
bool QMPU6050Backend::setFreefallDetectionCounterDecrement(quint8 decrement)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_MOT_DETECT_CTRL), decrement, static_cast<quint8>(MPU6050_DETECT_FF_COUNT_BIT), static_cast<quint8>(MPU6050_DETECT_FF_COUNT_LENGTH));
}
/** Get Motion detection counter decrement configuration.
 * Detection is registered by the Motion detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring MOT_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * MOT_COUNT | Counter Decrement
 * ----------+------------------
 * 0         | Reset
 * 1         | 1
 * 2         | 2
 * 3         | 4
 * </pre>
 *
 * When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Motion detection,
 * please refer to Registers 29 to 32.
 *
 */
bool QMPU6050Backend::getMotionDetectionCounterDecrement()
{
    quint8 buffer;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_MOT_DETECT_CTRL), &buffer, static_cast<quint8>(MPU6050_USERCTRL_FIFO_EN_BIT), static_cast<quint8>(MPU6050_DETECT_MOT_COUNT_LENGTH)))
        return false;

    if(m_sensor && static_cast<quint8>(m_sensor->m_motionDetectionCounterDecrement) != buffer)
    {
        m_sensor->m_motionDetectionCounterDecrement = static_cast<QMPU6050::CounterDecrement>(buffer);
        emit m_sensor->motionDetectionCounterDecrementChanged();
    }

    return true;
}
/** Set Motion detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getMotionDetectionCounterDecrement()
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_MOT_COUNT_BIT
 */
bool QMPU6050Backend::setMotionDetectionCounterDecrement(quint8 decrement)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_MOT_DETECT_CTRL), decrement, static_cast<quint8>(MPU6050_DETECT_MOT_COUNT_BIT), static_cast<quint8>(MPU6050_DETECT_MOT_COUNT_LENGTH));
}

// USER_CTRL register

/** Get FIFO enabled status.
 * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
 * cannot be written to or read from while disabled. The FIFO buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 * @return Current FIFO enabled status
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
bool QMPU6050Backend::getFIFOEnabled()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_USER_CTRL), &buffer, static_cast<quint8>(MPU6050_USERCTRL_FIFO_EN_BIT)))
        return false;

    bool enabled = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isFIFOEnabled != enabled)
    {
        m_sensor->m_isFIFOEnabled = enabled;
        emit m_sensor->FIFOEnabledChanged();
    }

    return true;
}

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
bool QMPU6050Backend::setFIFOEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_USER_CTRL), static_cast<quint8>(MPU6050_USERCTRL_FIFO_EN_BIT), enabled);
}
/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
bool QMPU6050Backend::resetFIFO()
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_USER_CTRL), static_cast<quint8>(MPU6050_USERCTRL_FIFO_RESET_BIT), true);
}
/** Reset all sensor registers and signal paths.
 * When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
 * accelerometers, and temperature sensor). This operation will also clear the
 * sensor registers. This bit automatically clears to 0 after the reset has been
 * triggered.
 *
 * When resetting only the signal path (and not the sensor registers), please
 * use Register 104, SIGNAL_PATH_RESET.
 *
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_SIG_COND_RESET_BIT
 */
bool QMPU6050Backend::resetSensors()
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_USER_CTRL), static_cast<quint8>(MPU6050_USERCTRL_SIG_COND_RESET_BIT), true);
}

// PWR_MGMT_1 register

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
bool QMPU6050Backend::reset()
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), static_cast<quint8>(MPU6050_PWR1_DEVICE_RESET_BIT), true);
}
/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool QMPU6050Backend::getSleepEnabled()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), &buffer, static_cast<quint8>(MPU6050_PWR1_SLEEP_BIT)))
        return false;

    bool enabled = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isSleepEnabled != enabled)
    {
        m_sensor->m_isSleepEnabled = enabled;
        emit m_sensor->sleepEnabledChanged();
    }

    return true;
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool QMPU6050Backend::setSleepEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), static_cast<quint8>(MPU6050_PWR1_SLEEP_BIT), enabled);
}
/** Get wake cycle enabled status.
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (register 108).
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CYCLE_BIT
 */
bool QMPU6050Backend::getWakeCycleEnabled()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), &buffer, static_cast<quint8>(MPU6050_PWR1_CYCLE_BIT)))
        return false;

    bool enabled = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isWakeCycleEnabled != enabled)
    {
        m_sensor->m_isWakeCycleEnabled = enabled;
        emit m_sensor->wakeCycleEnabledChanged();
    }

    return true;
}
/** Set wake cycle enabled status.
 * @param enabled New sleep mode enabled status
 * @see getWakeCycleEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CYCLE_BIT
 */
bool QMPU6050Backend::setWakeCycleEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), static_cast<quint8>(MPU6050_PWR1_CYCLE_BIT), enabled);
}
/** Get temperature sensor enabled status.
 * Control the usage of the internal temperature sensor.
 *
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @return Current temperature sensor enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_TEMP_DIS_BIT
 */
bool QMPU6050Backend::getTempSensorEnabled()
{
    quint8 buffer;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), &buffer, static_cast<quint8>(MPU6050_PWR1_TEMP_DIS_BIT)))
        return false;

    bool enabled = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isTemperatureSensorEnabled != enabled)
    {
        m_sensor->m_isTemperatureSensorEnabled = enabled;
        emit m_sensor->temperatureSensorEnabledChanged();
    }

    return true;
}
/** Set temperature sensor enabled status.
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param enabled New temperature sensor enabled status
 * @see getTempSensorEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_TEMP_DIS_BIT
 */
bool QMPU6050Backend::setTempSensorEnabled(bool enabled)
{
    // 1 is actually disabled here
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), static_cast<quint8>(MPU6050_PWR1_TEMP_DIS_BIT), !enabled);
}
/** Get clock source setting.
 * @return Current clock source setting
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
bool QMPU6050Backend::getClockSource()
{
    quint8 buffer = 0;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), &buffer, static_cast<quint8>(MPU6050_PWR1_CLKSEL_BIT), static_cast<quint8>(MPU6050_PWR1_CLKSEL_LENGTH)))
        return false;

    QMPU6050::ClockSource clockSource = static_cast<QMPU6050::ClockSource>(buffer);

    if(m_sensor && m_sensor->m_clockSource != clockSource)
    {
        m_sensor->m_clockSource = clockSource;
        emit m_sensor->clockSourceChanged();
    }

    return true;
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
bool QMPU6050Backend::setClockSource(quint8 source)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_PWR_MGMT_1), source, static_cast<quint8>(MPU6050_PWR1_CLKSEL_BIT), static_cast<quint8>(MPU6050_PWR1_CLKSEL_LENGTH));
}

// PWR_MGMT_2 register

/** Get wake frequency in Accel-Only Low Power Mode.
 * The MPU-60X0 can be put into Accerlerometer Only Low Power Mode by setting
 * PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode,
 * the device will power off all devices except for the primary I2C interface,
 * waking only the accelerometer at fixed intervals to take a single
 * measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL
 * as shown below:
 *
 * <pre>
 * LP_WAKE_CTRL | Wake-up Frequency
 * -------------+------------------
 * 0            | 1.25 Hz
 * 1            | 2.5 Hz
 * 2            | 5 Hz
 * 3            | 10 Hz
 * <pre>
 *
 * For further information regarding the MPU-60X0's power modes, please refer to
 * Register 107.
 *
 * @return Current wake frequency
 * @see MPU6050_RA_PWR_MGMT_2
 */
bool QMPU6050Backend::getWakeFrequency()
{
    quint8 buffer = 0;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), &buffer, static_cast<quint8>(MPU6050_PWR2_LP_WAKE_CTRL_BIT), static_cast<quint8>(MPU6050_PWR2_LP_WAKE_CTRL_LENGTH)))
        return false;

    QMPU6050::WakeFrequency wake = static_cast<QMPU6050::WakeFrequency>(buffer);

    if(m_sensor && m_sensor->m_wakeFrequency != wake)
    {
        m_sensor->m_wakeFrequency = wake;
        emit m_sensor->wakeFrequencyChanged();
    }

    return true;
}

/** Set wake frequency in Accel-Only Low Power Mode.
 * @param frequency New wake frequency
 * @see MPU6050_RA_PWR_MGMT_2
 */
bool QMPU6050Backend::setWakeFrequency(quint8 frequency)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), frequency, static_cast<quint8>(MPU6050_PWR2_LP_WAKE_CTRL_BIT), static_cast<quint8>(MPU6050_PWR2_LP_WAKE_CTRL_LENGTH));
}

/** Get X-axis accelerometer standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XA_BIT
 */
bool QMPU6050Backend::getStandbyXAccelEnabled()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), &buffer, static_cast<quint8>(MPU6050_PWR2_STBY_XA_BIT)))
        return false;

    bool standby = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_xAccelStandby != standby)
    {
        m_sensor->m_xAccelStandby = standby;
        emit m_sensor->xAccelStandbyChanged();
    }

    return true;
}
/** Set X-axis accelerometer standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXAccelEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XA_BIT
 */
bool QMPU6050Backend::setStandbyXAccelEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), static_cast<quint8>(MPU6050_PWR2_STBY_XA_BIT), enabled);
}
/** Get Y-axis accelerometer standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YA_BIT
 */
bool QMPU6050Backend::getStandbyYAccelEnabled()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), &buffer, static_cast<quint8>(MPU6050_PWR2_STBY_YA_BIT)))
        return false;

    bool standby = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_yAccelStandby != standby)
    {
        m_sensor->m_yAccelStandby = standby;
        emit m_sensor->yAccelStandbyChanged();
    }

    return true;
}
/** Set Y-axis accelerometer standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYAccelEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YA_BIT
 */
bool QMPU6050Backend::setStandbyYAccelEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), static_cast<quint8>(MPU6050_PWR2_STBY_YA_BIT), enabled);
}
/** Get Z-axis accelerometer standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZA_BIT
 */
bool QMPU6050Backend::getStandbyZAccelEnabled()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), &buffer, static_cast<quint8>(MPU6050_PWR2_STBY_ZA_BIT)))
        return false;

    bool standby = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_zAccelStandby != standby)
    {
        m_sensor->m_zAccelStandby = standby;
        emit m_sensor->zAccelStandbyChanged();
    }

    return true;
}
/** Set Z-axis accelerometer standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZAccelEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZA_BIT
 */
bool QMPU6050Backend::setStandbyZAccelEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), static_cast<quint8>(MPU6050_PWR2_STBY_ZA_BIT), enabled);
}

/** Get X-axis gyroscope standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XG_BIT
 */
bool QMPU6050Backend::getStandbyXGyroEnabled()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), &buffer, static_cast<quint8>(MPU6050_PWR2_STBY_XG_BIT)))
        return false;

    bool standby = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_xGyroStandby != standby)
    {
        m_sensor->m_xGyroStandby = standby;
        emit m_sensor->xGyroStandbyChanged();
    }

    return true;
}
/** Set X-axis gyroscope standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXGyroEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XG_BIT
 */
bool QMPU6050Backend::setStandbyXGyroEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), static_cast<quint8>(MPU6050_PWR2_STBY_XG_BIT), enabled);
}
/** Get Y-axis gyroscope standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YG_BIT
 */
bool QMPU6050Backend::getStandbyYGyroEnabled()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), &buffer, static_cast<quint8>(MPU6050_PWR2_STBY_YG_BIT)))
        return false;

    bool standby = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_yGyroStandby != standby)
    {
        m_sensor->m_yGyroStandby = standby;
        emit m_sensor->yGyroStandbyChanged();
    }

    return true;
}
/** Set Y-axis gyroscope standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYGyroEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YG_BIT
 */
bool QMPU6050Backend::setStandbyYGyroEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), static_cast<quint8>(MPU6050_PWR2_STBY_YG_BIT), enabled);
}
/** Get Z-axis gyroscope standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZG_BIT
 */
bool QMPU6050Backend::getStandbyZGyroEnabled()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), &buffer, static_cast<quint8>(MPU6050_PWR2_STBY_ZG_BIT)))
        return false;

    bool standby = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_zGyroStandby != standby)
    {
        m_sensor->m_zGyroStandby = standby;
        emit m_sensor->zGyroStandbyChanged();
    }

    return true;
}
/** Set Z-axis gyroscope standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZGyroEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZG_BIT
 */
bool QMPU6050Backend::setStandbyZGyroEnabled(bool enabled)
{
    return m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_PWR_MGMT_2), static_cast<quint8>(MPU6050_PWR2_STBY_ZG_BIT), enabled);
}

// FIFO_COUNT* registers

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
bool QMPU6050Backend::getFIFOCount(quint16 *count)
{
    quint8 *buffer = new quint8[2];

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_FIFO_COUNTH), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    quint16 result = (((quint16)buffer[0]) << 8) | buffer[1];
    delete [] buffer;

    if(m_sensor && m_sensor->m_FIFOCount != result)
    {
        m_sensor->m_FIFOCount = result;
        emit m_sensor->FIFOCountChanged();
    }

    return true;
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
bool QMPU6050Backend::getFIFOByte(quint8 *data)
{
    quint8 buffer = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_FIFO_R_W), &buffer, 1))
        return false;

    if(data)
        *data = buffer;

    return true;
}
/** Write byte to FIFO buffer.
 * @see getFIFOByte()
 * @see MPU6050_RA_FIFO_R_W
 */
bool QMPU6050Backend::setFIFOByte(quint8 data)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_FIFO_R_W), &data, 1);
}

// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @param buffer Buffer to write Device ID into (should be 0x68, 104 dec, 150 oct)
 * @return True/False for successful read
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
bool QMPU6050Backend::getDeviceID(quint8 *buffer)
{
    quint8 id = 0;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_WHO_AM_I), &id, static_cast<quint8>(MPU6050_WHO_AM_I_BIT), static_cast<quint8>(MPU6050_WHO_AM_I_LENGTH)))
        return false;

    if(m_sensor && m_sensor->m_deviceID != id)
    {
        m_sensor->m_deviceID = id;
        emit m_sensor->deviceIDChanged();
    }

    if(buffer)
        *buffer = id;

    return true;
}

/** Set Device ID.
 * Write a new ID into the WHO_AM_I register (no idea why this should ever be
 * necessary though).
 * @param id New device ID to set.
 * @return True/False for successful write
 * @see getDeviceID()
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
bool QMPU6050Backend::setDeviceID(quint8 id)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_WHO_AM_I), id, static_cast<quint8>(MPU6050_WHO_AM_I_BIT), static_cast<quint8>(MPU6050_WHO_AM_I_LENGTH));
}

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register

bool QMPU6050Backend::getXGyroOffset()
{
    quint8 offset = 0;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_XG_OFFS_TC), &offset, static_cast<quint8>(MPU6050_TC_OFFSET_BIT), static_cast<quint8>(MPU6050_TC_OFFSET_LENGTH)))
        return false;

    if(m_sensor && m_sensor->m_xGyroOffset != offset)
    {
        m_sensor->m_xGyroOffset = offset;
        emit m_sensor->xGyroOffsetChanged();
    }

    return true;
}

bool QMPU6050Backend::setXGyroOffset(qint8 offset)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_XG_OFFS_TC), offset, static_cast<quint8>(MPU6050_TC_OFFSET_BIT), static_cast<quint8>(MPU6050_TC_OFFSET_LENGTH));
}

// YG_OFFS_TC register

bool QMPU6050Backend::getYGyroOffset()
{
    quint8 offset = 0;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_YG_OFFS_TC), &offset, static_cast<quint8>(MPU6050_TC_OFFSET_BIT), static_cast<quint8>(MPU6050_TC_OFFSET_LENGTH)))
        return false;

    if(m_sensor && m_sensor->m_yGyroOffset != offset)
    {
        m_sensor->m_yGyroOffset = offset;
        emit m_sensor->yGyroOffsetChanged();
    }

    return true;
}

bool QMPU6050Backend::setYGyroOffset(qint8 offset)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_YG_OFFS_TC), offset, static_cast<quint8>(MPU6050_TC_OFFSET_BIT), static_cast<quint8>(MPU6050_TC_OFFSET_LENGTH));
}

// ZG_OFFS_TC register

bool QMPU6050Backend::getZGyroOffset()
{
    quint8 offset = 0;

    if(!m_i2c->readBits(static_cast<quint8>(MPU6050_RA_ZG_OFFS_TC), &offset, static_cast<quint8>(MPU6050_TC_OFFSET_BIT), static_cast<quint8>(MPU6050_TC_OFFSET_LENGTH)))
        return false;

    if(m_sensor && m_sensor->m_zGyroOffset != offset)
    {
        m_sensor->m_zGyroOffset = offset;
        emit m_sensor->zGyroOffsetChanged();
    }

    return true;
}

bool QMPU6050Backend::setZGyroOffset(qint8 offset)
{
    return m_i2c->writeBits(static_cast<quint8>(MPU6050_RA_ZG_OFFS_TC), offset, static_cast<quint8>(MPU6050_TC_OFFSET_BIT), static_cast<quint8>(MPU6050_TC_OFFSET_LENGTH));
}

// X_FINE_GAIN register
bool QMPU6050Backend::getXFineGain()
{
    quint8 gain = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_X_FINE_GAIN), &gain, 1))
        return false;

    if(m_sensor && m_sensor->m_yFineGrain != gain)
    {
        m_sensor->m_yFineGrain = gain;
        emit m_sensor->yFineGrainChanged();
    }

    return true;
}

bool QMPU6050Backend::setXFineGain(quint8 gain)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_X_FINE_GAIN), &gain, 1);
}

// Y_FINE_GAIN register

bool QMPU6050Backend::getYFineGain()
{
    quint8 gain = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_Y_FINE_GAIN), &gain, 1))
        return false;

    if(m_sensor && m_sensor->m_yFineGrain != gain)
    {
        m_sensor->m_yFineGrain = gain;
        emit m_sensor->yFineGrainChanged();
    }

    return true;
}
bool QMPU6050Backend::setYFineGain(quint8 gain)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_Y_FINE_GAIN), &gain, 1);
}

// Z_FINE_GAIN register

bool QMPU6050Backend::getZFineGain()
{
    quint8 gain = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_Z_FINE_GAIN), &gain, 1))
        return false;

    if(m_sensor && m_sensor->m_zFineGrain != gain)
    {
        m_sensor->m_zFineGrain = gain;
        emit m_sensor->zFineGrainChanged();
    }

    return true;
}
bool QMPU6050Backend::setZFineGain(quint8 gain)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_Z_FINE_GAIN), &gain, 1);
}

// XA_OFFS_* registers

bool QMPU6050Backend::getXAccelOffset()
{
    quint8 *buffer = new quint8[2];
    qint16 offset = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_XA_OFFS_H), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    offset = (qint16)buffer[0] << 8 | buffer[1];
    delete [] buffer;

    if(m_sensor && m_sensor->m_xAccelOffset != offset)
    {
        m_sensor->m_xAccelOffset = offset;
        emit m_sensor->xAccelOffsetChanged();
    }

    return true;
}

bool QMPU6050Backend::setXAccelOffset(qint16 offset)
{
    quint8 *buffer = new quint8[2]
    {
        static_cast<quint8>(offset >> 8),
        static_cast<quint8>(offset)
    };

    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_XA_OFFS_H), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    delete [] buffer;

    return true;
}

// YA_OFFS_* register

bool QMPU6050Backend::getYAccelOffset()
{
    quint8 *buffer = new quint8[2];
    qint16 offset = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_YA_OFFS_H), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    offset = (qint16)buffer[0] << 8 | buffer[1];
    delete [] buffer;

    if(m_sensor && m_sensor->m_yAccelOffset != offset)
    {
        m_sensor->m_yAccelOffset = offset;
        emit m_sensor->yAccelOffsetChanged();
    }

    return true;
}
bool QMPU6050Backend::setYAccelOffset(qint16 offset)
{
    quint8 *buffer = new quint8[2]
    {
        static_cast<quint8>(offset >> 8),
        static_cast<quint8>(offset)
    };

    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_YA_OFFS_H), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    delete [] buffer;

    return true;
}

// ZA_OFFS_* register

bool QMPU6050Backend::getZAccelOffset()
{
    quint8 *buffer = new quint8[2];
    qint16 offset = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_ZA_OFFS_H), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    offset = (qint16)buffer[0] << 8 | buffer[1];
    delete [] buffer;

    if(m_sensor && m_sensor->m_zAccelOffset != offset)
    {
        m_sensor->m_zAccelOffset = offset;
        emit m_sensor->zAccelOffsetChanged();
    }

    return true;
}
bool QMPU6050Backend::setZAccelOffset(qint16 offset)
{
    quint8 *buffer = new quint8[2]
    {
        static_cast<quint8>(offset >> 8),
        static_cast<quint8>(offset)
    };

    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_ZA_OFFS_H), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    delete [] buffer;

    if(m_sensor && m_sensor->m_zAccelOffset != offset)
    {
        m_sensor->m_zAccelOffset = offset;
        emit m_sensor->zAccelOffsetChanged();
    }

    return true;
}

// XG_OFFS_USR* registers

bool QMPU6050Backend::getXGyroOffsetUser()
{
    quint8 *buffer = new quint8[2];
    qint16 offset = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_XG_OFFS_USRH), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    offset = (qint16)buffer[0] << 8 | buffer[1];
    delete [] buffer;

    if(m_sensor && m_sensor->m_xGyroOffsetUser != offset)
    {
        m_sensor->m_xGyroOffsetUser = offset;
        emit m_sensor->xGyroOffsetUserChanged();
    }

    return true;
}
bool QMPU6050Backend::setXGyroOffsetUser(qint16 offset)
{
    quint8 *buffer = new quint8[2]
    {
        static_cast<quint8>(offset >> 8),
        static_cast<quint8>(offset)
    };

    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_XG_OFFS_USRH), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    delete [] buffer;

    return true;
}

// YG_OFFS_USR* register

bool QMPU6050Backend::getYGyroOffsetUser()
{
    quint8 *buffer = new quint8[2];
    qint16 offset = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_YG_OFFS_USRH), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    offset = (qint16)buffer[0] << 8 | buffer[1];
    delete [] buffer;

    if(m_sensor && m_sensor->m_yGyroOffsetUser != offset)
    {
        m_sensor->m_yGyroOffsetUser = offset;
        emit m_sensor->yGyroOffsetUserChanged();
    }

    return true;
}
bool QMPU6050Backend::setYGyroOffsetUser(qint16 offset)
{
    quint8 *buffer = new quint8[2]
    {
        static_cast<quint8>(offset >> 8),
        static_cast<quint8>(offset)
    };

    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_YG_OFFS_USRH), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    delete [] buffer;

    return true;
}

// ZG_OFFS_USR* register

bool QMPU6050Backend::getZGyroOffsetUser()
{
    quint8 *buffer = new quint8[2];
    qint16 offset = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_ZG_OFFS_USRH), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    offset = (qint16)buffer[0] << 8 | buffer[1];
    delete [] buffer;

    if(m_sensor && m_sensor->m_zGyroOffsetUser != offset)
    {
        m_sensor->m_zGyroOffsetUser = offset;
        emit m_sensor->zGyroOffsetUserChanged();
    }

    return true;
}
bool QMPU6050Backend::setZGyroOffsetUser(qint16 offset)
{
    quint8 *buffer = new quint8[2]
    {
        static_cast<quint8>(offset >> 8),
        static_cast<quint8>(offset)
    };

    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_ZG_OFFS_USRH), buffer, 2))
    {
        delete [] buffer;
        return false;
    }

    delete [] buffer;

    return true;
}

// INT_ENABLE register (DMP functions)

bool QMPU6050Backend::getIntPLLReadyEnabled()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_ZG_OFFS_USRH), &buffer, static_cast<quint8>(MPU6050_INTERRUPT_PLL_RDY_INT_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isIntPLLReadyEnabled != status)
    {
        m_sensor->m_intDMPEnabled = status;
        emit m_sensor->intPLLReadyEnabledChanged();
    }

    return buffer;
}

bool QMPU6050Backend::setIntPLLReadyEnabled(bool enabled)
{
    if(!m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_INT_ENABLE), static_cast<quint8>(MPU6050_INTERRUPT_PLL_RDY_INT_BIT), enabled))
        return false;

    return true;
}

bool QMPU6050Backend::getIntDMPEnabled()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_INT_ENABLE), &buffer, static_cast<quint8>(MPU6050_INTERRUPT_DMP_INT_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_intDMPEnabled != status)
    {
        m_sensor->m_intDMPEnabled = status;
        emit m_sensor->intDMPEnabledChanged();
    }

    return buffer;
}

bool QMPU6050Backend::setIntDMPEnabled(bool enabled)
{
    if(!m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_INT_ENABLE), static_cast<quint8>(MPU6050_INTERRUPT_DMP_INT_BIT), enabled))
        return false;

    return true;
}

// DMP_INT_STATUS

bool QMPU6050Backend::getDMPInt5Status()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_DMP_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_DMPINT_5_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_dmpInt5Status != status)
    {
        m_sensor->m_dmpInt5Status = status;
        emit m_sensor->dmpInt5StatusChanged();
    }

    return true;
}

bool QMPU6050Backend::getDMPInt4Status()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_DMP_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_DMPINT_4_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_dmpInt4Status != status)
    {
        m_sensor->m_dmpInt4Status = status;
        emit m_sensor->dmpInt4StatusChanged();
    }

    return true;
}
bool QMPU6050Backend::getDMPInt3Status()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_DMP_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_DMPINT_3_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_dmpInt3Status != status)
    {
        m_sensor->m_dmpInt3Status = status;
        emit m_sensor->dmpInt3StatusChanged();
    }

    return true;
}
bool QMPU6050Backend::getDMPInt2Status()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_DMP_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_DMPINT_2_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_dmpInt2Status != status)
    {
        m_sensor->m_dmpInt2Status = status;
        emit m_sensor->dmpInt2StatusChanged();
    }

    return true;
}
bool QMPU6050Backend::getDMPInt1Status()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_DMP_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_DMPINT_1_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_dmpInt1Status != status)
    {
        m_sensor->m_dmpInt1Status = status;
        emit m_sensor->dmpInt1StatusChanged();
    }

    return true;
}
bool QMPU6050Backend::getDMPInt0Status()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_DMP_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_DMPINT_0_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_dmpInt0Status != status)
    {
        m_sensor->m_dmpInt0Status = status;
        emit m_sensor->dmpInt0StatusChanged();
    }

    return true;
}

// INT_STATUS register (DMP functions)

bool QMPU6050Backend::getIntPLLReadyStatus()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_INTERRUPT_PLL_RDY_INT_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isIntPLLReadyEnabled != status)
    {
        m_sensor->m_isIntPLLReadyEnabled = status;
        emit m_sensor->intPLLReadyEnabledChanged();
    }

    return true;
}

bool QMPU6050Backend::getIntDMPStatus()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_INT_STATUS), &buffer, static_cast<quint8>(MPU6050_INTERRUPT_DMP_INT_BIT)))
        return false;

    bool status = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_intDMPStatus != status)
    {
        m_sensor->m_intDMPStatus = status;
        emit m_sensor->intDMPStatusChanged();
    }

    return true;
}

// USER_CTRL register (DMP functions)

bool QMPU6050Backend::getDMPEnabled()
{
    quint8 buffer = 0;

    if(!m_i2c->readBit(static_cast<quint8>(MPU6050_RA_USER_CTRL), &buffer, static_cast<quint8>(MPU6050_USERCTRL_DMP_EN_BIT)))
        return false;

    bool enabled = static_cast<bool>(buffer);

    if(m_sensor && m_sensor->m_isDMPEnabled != enabled)
    {
        m_sensor->m_isDMPEnabled = enabled;
        emit m_sensor->dmpEnabledChanged();
    }

    return true;
}

bool QMPU6050Backend::setDMPEnabled(bool enabled)
{
    if(!m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_USER_CTRL), static_cast<quint8>(MPU6050_USERCTRL_DMP_EN_BIT), enabled))
        return false;

    return true;
}

bool QMPU6050Backend::resetDMP()
{
    if(!m_i2c->writeBit(static_cast<quint8>(MPU6050_RA_USER_CTRL), static_cast<quint8>(MPU6050_USERCTRL_DMP_RESET_BIT), true))
        return false;

    return true;
}

bool QMPU6050Backend::pollDMPStatus()
{
    if(!getDMPEnabled()
        || !getIntDMPStatus()
        || !getIntPLLReadyEnabled()
        || !getIntPLLReadyStatus()
        || !getDMPEnabled()
        || !getDMPInt0Status()
        || !getDMPInt1Status()
        || !getDMPInt2Status()
        || !getDMPInt3Status()
        || !getDMPInt4Status()
        || !getDMPInt5Status())
        return false;

    return true;
}

// BANK_SEL register

bool QMPU6050Backend::setMemoryBank(quint8 bank, bool prefetchEnabled, bool userBank)
{
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;

    return m_i2c->write(static_cast<quint8>(MPU6050_RA_BANK_SEL), &bank, 1);
}

// MEM_START_ADDR register

bool QMPU6050Backend::setMemoryStartAddress(quint8 address)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_MEM_START_ADDR), &address, 1);
}

// MEM_R_W register

bool QMPU6050Backend::readMemoryByte(quint8 *buffer)
{
    return m_i2c->read(static_cast<quint8>(MPU6050_RA_MEM_R_W), buffer, 1);
}

bool QMPU6050Backend::writeMemoryByte(quint8 data)
{
    return m_i2c->write(static_cast<quint8>(MPU6050_RA_MEM_R_W), &data, 1);
}

bool QMPU6050Backend::readMemoryBlock(quint8 *data, quint16 dataSize, quint8 bank, quint8 address)
{
    if(!setMemoryBank(bank))
        return false;

    if(!setMemoryStartAddress(address))
        return false;

    quint8 chunkSize;
    for (quint16 i = 0; i < dataSize;)
    {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize)
            chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address)
            chunkSize = 256 - address;

        // read the chunk of data as specified
        if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_MEM_R_W), data + i, chunkSize))
            return false;

        // increase byte index by [chunkSize]
        i += chunkSize;

        // quint8 automatically wraps to 0 at 256
        address + chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize)
        {
            if (address == 0)
                bank++;

            if(!setMemoryBank(bank))
                return false;

            if(!setMemoryStartAddress(address))
                return false;
        }
    }

    return true;
}
bool QMPU6050Backend::writeMemoryBlock(quint8 *data, quint16 dataSize, quint8 bank, quint8 address, bool verify)
{
    if(!setMemoryBank(bank))
        return false;

    if(!setMemoryStartAddress(address))
        return false;

    quint8 chunkSize;
    quint8 *verifyBuffer;
    quint16 i;
    quint8 j;

    if (verify)
        verifyBuffer = new quint8[MPU6050_DMP_MEMORY_CHUNK_SIZE];

    for (i = 0; i < dataSize;)
    {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize)
            chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address)
            chunkSize = 256 - address;

        if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_MEM_R_W), data + i, chunkSize))
            return false;

        // verify data if needed
        if (verify && verifyBuffer)
        {
            if(!setMemoryBank(bank))
                return false;

            if(!setMemoryStartAddress(address))
                return false;

            if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_MEM_R_W), verifyBuffer, chunkSize))
                return false;

            if (memcmp(data + i, verifyBuffer, chunkSize) != 0)
            {
                QString message;
                QTextStream stream(&message);

                stream << QString("Block write verification error, bank ");
                stream << QString("%1").arg(bank, 2, 16, '0');
                stream << QString(", address ");
                stream << QString("%1").arg(address, 2, 16, '0');
                stream << QString("!\nExpected:");

                for (j = 0; j < chunkSize; j++)
                    stream << QString("0x%1 ").arg(data[i + j], 2, 16, '0');

                stream << QString("\nReceived:");

                for (quint8 j = 0; j < chunkSize; j++)
                    stream << QString("0x%1 ").arg(verifyBuffer[i + j], 2, 16, '0');

                stream << QString("\n");

                qDebug() << message;

                delete [] verifyBuffer;

                m_errno = EIO;
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // quint8 automatically wraps to 0 at 256
        address + chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize)
        {
            if (address == 0)
                bank++;

            if(!setMemoryBank(bank))
                return false;

            if(!setMemoryStartAddress(address))
                return false;
        }
    }
    if (verify)
        delete [] verifyBuffer;

    return true;
}

// DMP_CFG_1 register

bool QMPU6050Backend::getDMPConfig1()
{
    quint8 buffer = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_DMP_CFG_1), &buffer, 1))
        m_errno = errno;

    if(m_sensor && m_sensor->m_dmpConfig1 != buffer)
    {
        m_sensor->m_dmpConfig1 = buffer;
        emit m_sensor->dmpConfig1Changed();
    }

    return true;
}

bool QMPU6050Backend::setDMPConfig1(quint8 config)
{
    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_DMP_CFG_1), &config, 1))
    {
        handleFault();
        return false;
    }

    return true;
}

// DMP_CFG_2 register

bool QMPU6050Backend::getDMPConfig2()
{
    quint8 buffer = 0;

    if(!m_i2c->read(static_cast<quint8>(MPU6050_RA_DMP_CFG_2), &buffer, 1))
        handleFault();

    if(m_sensor && m_sensor->m_dmpConfig2 != buffer)
    {
        m_sensor->m_dmpConfig2 = buffer;
        emit m_sensor->dmpConfig2Changed();
    }

    return true;
}

bool QMPU6050Backend::setDMPConfig2(quint8 config)
{
    if(!m_i2c->write(static_cast<quint8>(MPU6050_RA_DMP_CFG_2), &config, 1))
    {
        handleFault();
        return false;
    }

    return true;
}
