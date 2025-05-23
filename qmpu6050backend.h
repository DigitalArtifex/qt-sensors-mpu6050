#ifndef QMPU6_5_BACKEND_H
#define QMPU6_5_BACKEND_H

#include <QObject>
#include <QString>
#include <QTextStream>
#include <QSensorBackend>
#include <QTimer>
#include <QThread>
#include <QDateTime>
#include <QStack>

#include "qmpu6050_global.h"
#include "qmpu6050.h"
#include "qmpu6050_p.h"
#include "qi2cdevice.h"

#include "fcntl.h"
#include "i2c/smbus.h"
#include "linux/i2c-dev.h"
#include "linux/errno.h"
#include "sys/ioctl.h"

QT_BEGIN_NAMESPACE

typedef bool (*QMPUFUNC) ();

class QMPU6_5__EXPORT QMPU6050Backend : public QSensorBackend
{
    Q_OBJECT

public:
    static inline const char* id = "QMPU6050-Backend";
    static inline const quint8 m_chipId = 0x58; //i2c chip id

    explicit QMPU6050Backend(QSensor *sensor = nullptr);
    ~QMPU6050Backend();

    virtual void start() override;
    virtual void stop() override;
    virtual bool isFeatureSupported(QSensor::Feature feature) const override;

    bool initialize();
    bool testConnection();

    // AUX_VDDIO register
    bool getAuxVDDIOLevel();
    bool setAuxVDDIOLevel(quint8 level);

    // SMPLRT_DIV register
    bool getRate();
    bool setRate(quint8 rate);

    // CONFIG register
    bool getExternalFrameSync();
    bool setExternalFrameSync(quint8 sync);
    bool getDLPFMode();
    bool setDLPFMode(quint8 bandwidth);

    // GYRO_CONFIG register
    bool getFullScaleGyroRange();
    bool setFullScaleGyroRange(quint8 range);

    // ACCEL_CONFIG register
    bool getAccelXSelfTest();
    bool setAccelXSelfTest(bool enabled);
    bool getAccelYSelfTest();
    bool setAccelYSelfTest(bool enabled);
    bool getAccelZSelfTest();
    bool setAccelZSelfTest(bool enabled);
    bool getFullScaleAccelRange();
    bool setFullScaleAccelRange(quint8 range);
    bool getDHPFMode();
    bool setDHPFMode(quint8 mode);

    // FF_THR register
    bool getFreefallDetectionThreshold();
    bool setFreefallDetectionThreshold(quint8 threshold);

    // FF_DUR register
    bool getFreefallDetectionDuration();
    bool setFreefallDetectionDuration(quint8 duration);

    // MOT_THR register
    bool getMotionDetectionThreshold();
    bool setMotionDetectionThreshold(quint8 threshold);

    // MOT_DUR register
    bool getMotionDetectionDuration();
    bool setMotionDetectionDuration(quint8 duration);

    // ZRMOT_THR register
    bool getZeroMotionDetectionThreshold();
    bool setZeroMotionDetectionThreshold(quint8 threshold);

    // ZRMOT_DUR register
    bool getZeroMotionDetectionDuration();
    bool setZeroMotionDetectionDuration(quint8 duration);

    // FIFO_EN register
    bool setTempFIFOEnabled(bool enabled);
    bool setXGyroFIFOEnabled(bool enabled);
    bool setYGyroFIFOEnabled(bool enabled);
    bool setZGyroFIFOEnabled(bool enabled);
    bool setAccelFIFOEnabled(bool enabled);

    // INT_ENABLE register
    bool setIntFIFOBufferOverflowEnabled(bool enabled);
    bool setIntDataReadyEnabled(bool enabled);

    // INT_STATUS register
    bool getIntFIFOBufferOverflowStatus();
    bool getIntDataReadyStatus();

    // ACCEL_*OUT_* registers
    bool get9AxisMotion();
    bool get6AxisMotion();
    bool getAcceleration();

    // TEMP_OUT_* registers
    bool getTemperature();

    // GYRO_*OUT_* registers
    bool getRotation();

    // MOT_DETECT_STATUS register
    bool getXNegMotionDetected();
    bool getXPosMotionDetected();
    bool getYNegMotionDetected();
    bool getYPosMotionDetected();
    bool getZNegMotionDetected();
    bool getZPosMotionDetected();
    bool getZeroMotionDetected();

    // SIGNAL_PATH_RESET register
    bool resetGyroscopePath();
    bool resetAccelerometerPath();
    bool resetTemperaturePath();

    // MOT_DETECT_CTRL register
    bool getAccelerometerPowerOnDelay();
    bool setAccelerometerPowerOnDelay(quint8 delay);
    bool getFreefallDetectionCounterDecrement();
    bool setFreefallDetectionCounterDecrement(quint8 decrement);
    bool getMotionDetectionCounterDecrement();
    bool setMotionDetectionCounterDecrement(quint8 decrement);

    // USER_CTRL register
    bool getFIFOEnabled();
    bool setFIFOEnabled(bool enabled);
    bool resetFIFO();
    bool resetSensors();

    // PWR_MGMT_1 register
    bool reset();
    bool getSleepEnabled();
    bool setSleepEnabled(bool enabled);
    bool getWakeCycleEnabled();
    bool setWakeCycleEnabled(bool enabled);
    bool getTempSensorEnabled();
    bool setTempSensorEnabled(bool enabled);
    bool getClockSource();
    bool setClockSource(quint8 source);

    // PWR_MGMT_2 register
    bool getWakeFrequency();
    bool setWakeFrequency(quint8 frequency);
    bool getStandbyXAccelEnabled();
    bool setStandbyXAccelEnabled(bool enabled);
    bool getStandbyYAccelEnabled();
    bool setStandbyYAccelEnabled(bool enabled);
    bool getStandbyZAccelEnabled();
    bool setStandbyZAccelEnabled(bool enabled);
    bool getStandbyXGyroEnabled();
    bool setStandbyXGyroEnabled(bool enabled);
    bool getStandbyYGyroEnabled();
    bool setStandbyYGyroEnabled(bool enabled);
    bool getStandbyZGyroEnabled();
    bool setStandbyZGyroEnabled(bool enabled);

    // FIFO_COUNT_* registers
    bool getFIFOCount(quint16 *count);

    // FIFO_R_W register
    bool getFIFOByte(quint8 *data);
    bool setFIFOByte(quint8 data);

    // WHO_AM_I register
    bool getDeviceID(quint8 *buffer = nullptr);
    bool setDeviceID(quint8 id);

    // ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

    // XG_OFFS_TC register
    bool getXGyroOffset();
    bool setXGyroOffset(qint8 offset);

    // YG_OFFS_TC register
    bool getYGyroOffset();
    bool setYGyroOffset(qint8 offset);

    // ZG_OFFS_TC register
    bool getZGyroOffset();
    bool setZGyroOffset(qint8 offset);

    // X_FINE_GAIN register
    bool getXFineGain();
    bool setXFineGain(quint8 gain);

    // Y_FINE_GAIN register
    bool getYFineGain();
    bool setYFineGain(quint8 gain);

    // Z_FINE_GAIN register
    bool getZFineGain();
    bool setZFineGain(quint8 gain);

    // XA_OFFS_* registers
    bool getXAccelOffset();
    bool setXAccelOffset(qint16 offset);

    // YA_OFFS_* register
    bool getYAccelOffset();
    bool setYAccelOffset(qint16 offset);

    // ZA_OFFS_* register
    bool getZAccelOffset();
    bool setZAccelOffset(qint16 offset);

    // XG_OFFS_USR* registers
    bool getXGyroOffsetUser();
    bool setXGyroOffsetUser(qint16 offset);

    // YG_OFFS_USR* register
    bool getYGyroOffsetUser();
    bool setYGyroOffsetUser(qint16 offset);

    // ZG_OFFS_USR* register
    bool getZGyroOffsetUser();
    bool setZGyroOffsetUser(qint16 offset);

    // INT_ENABLE register (DMP functions)
    bool getIntPLLReadyEnabled();
    bool setIntPLLReadyEnabled(bool enabled);
    bool getIntDMPEnabled();
    bool setIntDMPEnabled(bool enabled);

    // DMP_INT_STATUS
    bool getDMPInt5Status();
    bool getDMPInt4Status();
    bool getDMPInt3Status();
    bool getDMPInt2Status();
    bool getDMPInt1Status();
    bool getDMPInt0Status();

    // INT_STATUS register (DMP functions)
    bool getIntPLLReadyStatus();
    bool getIntDMPStatus();

    // USER_CTRL register (DMP functions)
    bool getDMPEnabled();
    bool setDMPEnabled(bool enabled);
    bool resetDMP();

    /**
     * @brief pollDMPStatus
     *
     * Polls DMPEnabled, IntDMPStatus, IntPLLReadyStatus, IntPLLReadyEnabled and DMPInt0-DMPInt5Status
     *
     * @return
     */
    bool pollDMPStatus();

    // BANK_SEL register
    bool setMemoryBank(quint8 bank, bool prefetchEnabled=false, bool userBank=false);

    // MEM_START_ADDR register
    bool setMemoryStartAddress(quint8 address);

    // MEM_R_W register
    bool readMemoryByte(quint8 *buffer);
    bool writeMemoryByte(quint8 data);
    bool readMemoryBlock(quint8 *data, quint16 dataSize, quint8 bank=0, quint8 address=0);
    bool writeMemoryBlock(quint8 *data, quint16 dataSize, quint8 bank=0, quint8 address=0, bool verify=true);

    // DMP_CFG_1 register
    bool getDMPConfig1();
    bool setDMPConfig1(quint8 config);

    // DMP_CFG_2 register
    bool getDMPConfig2();
    bool setDMPConfig2(quint8 config);

public slots:
    void setBus(const QString &bus);

    void setAddress(quint8 address);

signals:
    void accelerometerDataReady(const qreal &x, const qreal &y, const qreal &z);
    void gyroscopeDataReady(const qreal &x, const qreal &y, const qreal &z);

protected slots:
    void poll();

protected:
    void handleFault();
    void reportEvent(QString message);
    void reportError(QString message);
    void newLine();

    void onSensorBusChanged();
    void onSensorAddressChanged();
    void onSesnorDataRateChanged();

private:
    QString m_bus = "/dev/i2c-1";
    quint8 m_address = 0x68;

    QMPU6050 *m_sensor = nullptr;

    QI2CDevice *m_i2c = nullptr;
    int m_errno;

    bool m_initialized = false;
    bool m_backendDebug = true;
    QTimer *m_pollTimer = nullptr;

    qreal m_ax = 0;
    qreal m_ay = 0;
    qreal m_az = 0;

    qreal m_gx = 0;
    qreal m_gy = 0;
    qreal m_gz = 0;

    bool m_dataReady = false;
    bool m_fifoOverflow = false;

    QAccelerometerReading m_reading;

    /* This is only included if you want it, since it eats about 2K of program
     * memory, which is a waste if you aren't using the DMP (or if you aren't
     * using this particular flavor of DMP).
     *
     * Source is from the InvenSense MotionApps v2 demo code. Original source is
     * unavailable, unless you happen to be amazing as decompiling binary by
     * hand (in which case, please contact me, and I'm totally serious).
     *
     * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
     * DMP reverse-engineering he did to help make this bit of wizardry
     * possible.
     */

    // this block of memory gets written to the MPU on start-up, and it seems
    // to be volatile memory, so it has to be done each time (it only takes ~1
    // second though)
    quint8 dmpMemory[MPU6050_DMP_CODE_SIZE] {
        // bank 0, 256 bytes
        0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
        0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
        0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
        0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
        0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
        0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
        0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
        0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
        0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

        // bank 1, 256 bytes
        0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
        0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
        0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
        0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

        // bank 2, 256 bytes
        0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
        0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        // bank 3, 256 bytes
        0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
        0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
        0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
        0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
        0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
        0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
        0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
        0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
        0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
        0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
        0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
        0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
        0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
        0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
        0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
        0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

        // bank 4, 256 bytes
        0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
        0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
        0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
        0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
        0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
        0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
        0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
        0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
        0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
        0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
        0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
        0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
        0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
        0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
        0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
        0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

        // bank 5, 256 bytes
        0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
        0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
        0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
        0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
        0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
        0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
        0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
        0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
        0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
        0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
        0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
        0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
        0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
        0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
        0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
        0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

        // bank 6, 256 bytes
        0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
        0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
        0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
        0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
        0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
        0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
        0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
        0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
        0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
        0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
        0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
        0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
        0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
        0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
        0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
        0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

        // bank 7, 138 bytes (remainder)
        0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
        0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
        0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
        0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
        0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
        0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
        0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
        0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
        0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
    };

    quint8 dmpUpdates[29][9] = {
        { 0x03, 0x7B, 0x03, 0x4C, 0xCD, 0x6C },         // FCFG_1 inv_set_gyro_calibration
        { 0x03, 0xAB, 0x03, 0x36, 0x56, 0x76 },         // FCFG_3 inv_set_gyro_calibration
        { 0x00, 0x68, 0x04, 0x02, 0xCB, 0x47, 0xA2 },   // D_0_104 inv_set_gyro_calibration
        { 0x02, 0x18, 0x04, 0x00, 0x05, 0x8B, 0xC1 },   // D_0_24 inv_set_gyro_calibration
        { 0x01, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x00 },   // D_1_152 inv_set_accel_calibration
        { 0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97 }, // FCFG_2 inv_set_accel_calibration
        { 0x03, 0x89, 0x03, 0x26, 0x46, 0x66 },         // FCFG_7 inv_set_accel_calibration
        { 0x00, 0x6C, 0x02, 0x20, 0x00 },               // D_0_108 inv_set_accel_calibration
        { 0x02, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00 },   // CPASS_MTX_00 inv_set_compass_calibration
        { 0x02, 0x44, 0x04, 0x00, 0x00, 0x00, 0x00 },   // CPASS_MTX_01
        { 0x02, 0x48, 0x04, 0x00, 0x00, 0x00, 0x00 },   // CPASS_MTX_02
        { 0x02, 0x4C, 0x04, 0x00, 0x00, 0x00, 0x00 },   // CPASS_MTX_10
        { 0x02, 0x50, 0x04, 0x00, 0x00, 0x00, 0x00 },   // CPASS_MTX_11
        { 0x02, 0x54, 0x04, 0x00, 0x00, 0x00, 0x00 },   // CPASS_MTX_12
        { 0x02, 0x58, 0x04, 0x00, 0x00, 0x00, 0x00 },   // CPASS_MTX_20
        { 0x02, 0x5C, 0x04, 0x00, 0x00, 0x00, 0x00 },   // CPASS_MTX_21
        { 0x02, 0xBC, 0x04, 0x00, 0x00, 0x00, 0x00 },   // CPASS_MTX_22
        { 0x01, 0xEC, 0x04, 0x00, 0x00, 0x40, 0x00 },   // D_1_236 inv_apply_endian_accel
        { 0x03, 0x7F, 0x06, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97 }, // FCFG_2 inv_set_mpu_sensors
        { 0x04, 0x02, 0x03, 0x0D, 0x35, 0x5D },         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
        { 0x04, 0x09, 0x04, 0x87, 0x2D, 0x35, 0x3D },   // FCFG_5 inv_set_bias_update
        { 0x00, 0xA3, 0x01, 0x00 },                     // D_0_163 inv_set_dead_zone
        // SET INT_ENABLE at i=22
        { 0x07, 0x86, 0x01, 0xFE },                     // CFG_6 inv_set_fifo_interupt
        { 0x07, 0x41, 0x05, 0xF1, 0x20, 0x28, 0x30, 0x38 }, // CFG_8 inv_send_quaternion
        { 0x07, 0x7E, 0x01, 0x30 },                     // CFG_16 inv_set_footer
        { 0x07, 0x46, 0x01, 0x9A },                     // CFG_GYRO_SOURCE inv_send_gyro
        { 0x07, 0x47, 0x04, 0xF1, 0x28, 0x30, 0x38 },   // CFG_9 inv_send_gyro -> inv_construct3_fifo
        { 0x07, 0x6C, 0x04, 0xF1, 0x28, 0x30, 0x38 },   // CFG_12 inv_send_accel -> inv_construct3_fifo
        { 0x02, 0x16, 0x02, 0x00, 0x0A }                // D_0_22 inv_set_fifo_rate
    };
};

QT_END_NAMESPACE

#endif // QMPU6_5_BACKEND_H
