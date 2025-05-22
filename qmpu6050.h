#ifndef QMPU6_5__H
#define QMPU6_5__H


#include <QObject>
#include <QSensor>
#include <QString>
#include <QAccelerometerReading>

#include "qmpu6050_global.h"

QT_BEGIN_NAMESPACE

class QMPU6050Backend;

class QMPU6_5__EXPORT QMPU6050 : public QSensor
{
    Q_OBJECT
    friend class QMPU6050Backend;
public:
    static inline char const * const sensorType = "QMPU6050";

    enum class ClockSource : quint8
    {
        Internal,
        PLLXGyro,
        PLLYGyro,
        PLLZGyro,
        PLL32kHz,
        PLL19MHz,
        Stopped = 7
    };
    Q_ENUM(ClockSource)

    enum class WakeFrequency : quint8
    {
        Hz1,
        Hz2,
        Hz5,
        Hz10
    };
    Q_ENUM(WakeFrequency)

    enum class CounterDecrement : quint8
    {
        Reset,
        Single,
        Double,
        Quad
    };
    Q_ENUM(CounterDecrement)

    enum class DHPFilterMode : quint8
    {
        Reset,
        Cutoff5Hz,
        Cutoff2Hz,
        Cutoff1Hz,
        Cutoff630mHz,
        Hold = 7
    };
    Q_ENUM(DHPFilterMode)

    enum class DLPFilterMode : quint8
    {
        DLPF0,
        DLPF1,
        DLPF2,
        DLPF3,
        DLPF4,
        DLPF5,
        DLPF6,
        DLPF7,
    };
    Q_ENUM(DLPFilterMode)

    enum class ExternalFrameSync : quint8
    {
        Disabled,
        Temperature,
        XGyroscope,
        YGyroscope,
        ZGyroscope,
        XAccelerometer,
        YAccelerometer,
        ZAccelerometer
    };
    Q_ENUM(ExternalFrameSync)

    explicit QMPU6050(QObject *parent = nullptr);

    QAccelerometerReading *reading() const;

    bool initialize();

    QString bus() const;
    void setBus(const QString &bus);

    quint8 address() const;
    void setAddress(quint8 address);

    quint8 dmpConfig2() const;
    void setDmpConfig2(quint8 dmpConfig2);

    quint8 dmpConfig1() const;
    void setDmpConfig1(quint8 dmpConfig1);

    QMPU6050Backend *controller() const;

    bool isIntPLLReadyEnabled() const;
    void setIntPLLReadyEnabled(bool intPLLReadyEnabled);

    bool intPLLReadyStatus() const;

    bool intDMPStatus() const;

    bool isDMPEnabled() const;
    void setDMPEnabled(bool dmpEnabled);

    bool dmpInt0Status() const;

    bool dmpInt1Status() const;

    bool dmpInt2Status() const;

    bool dmpInt3Status() const;
    void setDmpInt3Status(bool dmpInt3Status);

    bool dmpInt4Status() const;
    void setDmpInt4Status(bool dmpInt4Status);

    bool dmpInt5Status() const;

    bool intDMPEnabled() const;
    void setIntDMPEnabled(bool intDMPEnabled);

    qint16 zGyroOffsetUser() const;
    void setZGyroOffsetUser(qint16 zGyroOffsetUser);

    qint16 yGyroOffsetUser() const;
    void setYGyroOffsetUser(qint16 yGyroOffsetUser);

    qint16 xGyroOffsetUser() const;
    void setXGyroOffsetUser(qint16 xGyroOffsetUser);

    qint16 zAccelOffset() const;
    void setZAccelOffset(qint16 zAccelOffset);

    qint16 yAccelOffset() const;
    void setYAccelOffset(qint16 yAccelOffset);

    qint16 xAccelOffset() const;
    void setXAccelOffset(qint16 xAccelOffset);

    qint8 zFineGrain() const;
    void setZFineGrain(qint8 zFineGrain);

    qint8 yFineGrain() const;
    void setYFineGrain(qint8 yFineGrain);

    qint8 xFineGrain() const;
    void setXFineGrain(qint8 xFineGrain);

    qint16 zGyroOffset() const;
    void setZGyroOffset(qint16 zGyroOffset);

    qint16 yGyroOffset() const;
    void setYGyroOffset(qint16 yGyroOffset);

    qint16 xGyroOffset() const;
    void setXGyroOffset(qint16 xGyroOffset);

    qint8 deviceID() const;
    void setDeviceID(qint8 deviceID);
    bool updateDeviceID(qint8 deviceID);

    bool zGyroStandby() const;
    void setZGyroStandby(bool zGyroStandby);

    bool yGyroStandby() const;
    void setYGyroStandby(bool yGyroStandby);

    bool xGyroStandby() const;
    void setXGyroStandby(bool xGyroStandby);

    bool zAccelStandby() const;
    void setZAccelStandby(bool zAccelStandby);

    bool yAccelStandby() const;
    void setYAccelStandby(bool yAccelStandby);

    bool xAccelStandby() const;
    void setXAccelStandby(bool xAccelStandby);

    WakeFrequency wakeFrequency() const;
    void setWakeFrequency(WakeFrequency wakeFrequency);

    ClockSource clockSource() const;
    void setClockSource(ClockSource clockSource);

    bool isWakeCycleEnabled() const;
    void setWakeCycleEnabled(bool wakeCycleEnabled);

    bool isTemperatureSensorEnabled() const;
    void setTemperatureSensorEnabled(bool temperatureSensorEnabled);

    bool isSleepEnabled() const;
    void setSleepEnabled(bool sleepEnabled);

    bool isFIFOEnabled() const;
    void setFIFOEnabled(bool FIFOEnabled);

    quint16 FIFOCount() const;

    CounterDecrement motionDetectionCounterDecrement() const;
    void setMotionDetectionCounterDecrement(CounterDecrement motionDetectionCounterDecrement);

    CounterDecrement freefallDetectionCounterDecrement() const;
    void setFreefallDetectionCounterDecrement(CounterDecrement freefallDetectionCounterDecrement);

    quint8 accelerometerPowerOnDelay() const;
    void setAccelerometerPowerOnDelay(quint8 accelerometerPowerOnDelay);

    bool isZeroMotionDetected() const;

    bool isZPositiveMotionDetected() const;

    bool isZNegativeMotionDetected() const;

    bool isYPositiveMotionDetected() const;

    bool isYNegativeMotionDetected() const;

    bool isXPositiveMotionDetected() const;

    bool isXNegativeMotionDetected() const;

    qint16 zRotation() const;

    qint16 yRotation() const;

    qint16 xRotation() const;

    qint16 temperature() const;

    qint16 zAcceleration() const;

    qint16 yAcceleration() const;

    qint16 xAcceleration() const;

    qint8 zeroMotionDetectionDuration() const;
    void setZeroMotionDetectionDuration(qint8 zeroMotionDetectionDuration);

    qint8 zeroMotionDetectionThreshold() const;
    void setZeroMotionDetectionThreshold(qint8 zeroMotionDetectionThreshold);

    qint8 motionDetectionDuration() const;
    void setMotionDetectionDuration(qint8 motionDetectionDuration);

    qint8 motionDetectionThreshold() const;
    void setMotionDetectionThreshold(qint8 motionDetectionThreshold);

    qint8 freefallDetectionDuration() const;
    void setFreefallDetectionDuration(qint8 freefallDetectionDuration);

    qint8 freefallDetectionThreshold() const;
    void setFreefallDetectionThreshold(qint8 freefallDetectionThreshold);

    DHPFilterMode dhpFilterMode() const;
    void setDhpFilterMode(DHPFilterMode dhpFilterMode);

    quint8 fullScaleAccerometerRange() const;
    void setFullScaleAccerometerRange(quint8 fullScaleAccerometerRange);

    bool isAccelerometerSelfTestZEnabled() const;
    void setAccelerometerSelfTestZEnabled(bool accelerometerSelfTestZEnabled);

    bool isAccelerometerSelfTestYEnabled() const;
    void setAccelerometerSelfTestYEnabled(bool accelerometerSelfTestYEnabled);

    bool isAccelerometerSelfTestXEnabled() const;
    void setAccelerometerSelfTestXEnabled(bool accelerometerSelfTestXEnabled);

    quint16 fullScaleGyroscopeRange() const;
    void setFullScaleGyroscopeRange(quint16 fullScaleGyroscopeRange);

    DLPFilterMode dlpFilterMode() const;
    void setDlpFilterMode(DLPFilterMode dlpFilterMode);

    ExternalFrameSync externalFrameSync() const;
    void setExternalFrameSync(ExternalFrameSync externalFrameSync);

    quint8 gyroscopeRateDivider() const;
    void setGyroscopeRateDivider(quint8 gyroscopeRateDivider);

signals:
    void busChanged();
    void addressChanged();

    void dmpConfig2Changed();

    void dmpConfig1Changed();

    void intPLLReadyEnabledChanged();

    void intPLLReadyStatusChanged();

    void intDMPStatusChanged();

    void dmpEnabledChanged();

    void dmpInt0StatusChanged();

    void dmpInt1StatusChanged();

    void dmpInt2StatusChanged();

    void dmpInt3StatusChanged();

    void dmpInt4StatusChanged();

    void dmpInt5StatusChanged();

    void intDMPEnabledChanged();

    void zGyroOffsetUserChanged();

    void yGyroOffsetUserChanged();

    void xGyroOffsetUserChanged();

    void zAccelOffsetChanged();

    void yAccelOffsetChanged();

    void xAccelOffsetChanged();

    void zFineGrainChanged();

    void yFineGrainChanged();

    void xFineGrainChanged();

    void zGyroOffsetChanged();

    void yGyroOffsetChanged();

    void xGyroOffsetChanged();

    void deviceIDChanged();

    void zGyroStandbyChanged();

    void yGyroStandbyChanged();

    void xGyroStandbyChanged();

    void zAccelStandbyChanged();

    void yAccelStandbyChanged();

    void xAccelStandbyChanged();

    void wakeFrequencyChanged();

    void clockSourceChanged();

    void wakeCycleEnabledChanged();

    void temperatureSensorEnabledChanged();

    void sleepEnabledChanged();

    void FIFOEnabledChanged();

    void FIFOCountChanged();

    void motionDetectionCounterDecrementChanged();

    void freefallDetectionCounterDecrementChanged();

    void accelerometerPowerOnDelayChanged();

    void zeroMotionDetectedChanged();

    void zPositiveMotionDetectedChanged();

    void zNegativeMotionDetectedChanged();

    void yPositiveMotionDetectedChanged();

    void yNegativeMotionDetectedChanged();

    void xPositiveMotionDetectedChanged();

    void xNegativeMotionDetectedChanged();

    void zRotationChanged();

    void yRotationChanged();

    void xRotationChanged();

    void temperatureChanged();

    void zAccelerationChanged();

    void yAccelerationChanged();

    void xAccelerationChanged();

    void zeroMotionDetectionDurationChanged();

    void zeroMotionDetectionThresholdChanged();

    void motionDetectionDurationChanged();

    void motionDetectionThresholdChanged();

    void freefallDetectionDurationChanged();

    void freefallDetectionThresholdChanged();

    void dhpFilterModeChanged();

    void fullScaleAccerometerRangeChanged();

    void accelerometerSelfTestZEnabledChanged();

    void accelerometerSelfTestYEnabledChanged();

    void accelerometerSelfTestXEnabledChanged();

    void fullScaleGyroscopeRangeChanged();

    void dlpFilterModeChanged();

    void externalFrameSyncChanged();

    void gyroscopeRateDividerChanged();

private:
    QMPU6050Backend *m_controller = nullptr;

    QString m_bus = "/dev/i2c-1"; //i2c bus path
    quint8 m_address = 0x68; //i2c device address

    quint8 m_dmpConfig1 = 0;
    quint8 m_dmpConfig2 = 0;

    bool m_isIntPLLReadyEnabled = false;
    bool m_intPLLReadyStatus = false;
    bool m_intDMPStatus = false;
    bool m_intDMPEnabled = false;
    bool m_isDMPEnabled = false;

    //dmp status
    bool m_dmpInt0Status = false;
    bool m_dmpInt1Status = false;
    bool m_dmpInt2Status = false;
    bool m_dmpInt3Status = false;
    bool m_dmpInt4Status = false;
    bool m_dmpInt5Status = false;

    qint16 m_zGyroOffsetUser = 0;
    qint16 m_yGyroOffsetUser = 0;
    qint16 m_xGyroOffsetUser = 0;

    qint16 m_zGyroOffset = 0;
    qint16 m_yGyroOffset = 0;
    qint16 m_xGyroOffset = 0;

    bool m_zGyroStandby = false;
    bool m_yGyroStandby = false;
    bool m_xGyroStandby = false;

    bool m_zAccelStandby = false;
    bool m_yAccelStandby = false;
    bool m_xAccelStandby = false;

    qint16 m_zAccelOffset = 0;
    qint16 m_yAccelOffset = 0;
    qint16 m_xAccelOffset = 0;

    qint8 m_zFineGrain = 0;
    qint8 m_yFineGrain = 0;
    qint8 m_xFineGrain = 0;

    WakeFrequency m_wakeFrequency = WakeFrequency::Hz1;
    bool m_isWakeCycleEnabled = true;

    ClockSource m_clockSource = ClockSource::Internal;

    qint8 m_deviceID = 0;

    bool m_isTemperatureSensorEnabled = true;
    bool m_isSleepEnabled = false;

    quint16 m_FIFOCount = 0;
    bool m_isFIFOEnabled = false;

    CounterDecrement m_motionDetectionCounterDecrement = CounterDecrement::Reset;
    CounterDecrement m_freefallDetectionCounterDecrement = CounterDecrement::Reset;

    quint8 m_accelerometerPowerOnDelay = 0;

    qint16 m_temperature = 0;

    bool m_isZeroMotionDetected = false;
    bool m_isZPositiveMotionDetected = false;
    bool m_isZNegativeMotionDetected = false;
    bool m_isYPositiveMotionDetected = false;
    bool m_isYNegativeMotionDetected = false;
    bool m_isXPositiveMotionDetected = false;
    bool m_isXNegativeMotionDetected = false;

    qint16 m_zRotation = 0;
    qint16 m_yRotation = 0;
    qint16 m_xRotation = 0;

    qint16 m_zAcceleration = 0;
    qint16 m_yAcceleration = 0;
    qint16 m_xAcceleration = 0;

    qint8 m_zeroMotionDetectionDuration = 0;
    qint8 m_zeroMotionDetectionThreshold = 0;

    qint8 m_motionDetectionDuration = 0;
    qint8 m_motionDetectionThreshold = 0;

    qint8 m_freefallDetectionDuration = 0;
    qint8 m_freefallDetectionThreshold = 0;

    DHPFilterMode m_dhpFilterMode = DHPFilterMode::Reset;
    DLPFilterMode m_dlpFilterMode = DLPFilterMode::DLPF0;
    ExternalFrameSync m_externalFrameSync = ExternalFrameSync::Disabled;

    quint8 m_fullScaleAccerometerRange = 16;

    bool m_isAccelerometerSelfTestZEnabled = true;
    bool m_isAccelerometerSelfTestYEnabled = true;
    bool m_isAccelerometerSelfTestXEnabled = true;

    quint16 m_fullScaleGyroscopeRange = 2000;

    quint8 m_gyroscopeRateDivider = 1;

    Q_PROPERTY(QString bus READ bus WRITE setBus NOTIFY busChanged FINAL)
    Q_PROPERTY(quint8 address READ address WRITE setAddress NOTIFY addressChanged FINAL)
    Q_PROPERTY(quint8 dmpConfig1 READ dmpConfig1 WRITE setDmpConfig1 NOTIFY dmpConfig1Changed FINAL)
    Q_PROPERTY(quint8 dmpConfig2 READ dmpConfig2 WRITE setDmpConfig2 NOTIFY dmpConfig2Changed FINAL)
    Q_PROPERTY(bool isIntPLLReadyEnabled READ isIntPLLReadyEnabled WRITE setIntPLLReadyEnabled NOTIFY intPLLReadyEnabledChanged FINAL)
    Q_PROPERTY(bool intPLLReadyStatus READ intPLLReadyStatus NOTIFY intPLLReadyStatusChanged FINAL)
    Q_PROPERTY(bool intDMPStatus READ intDMPStatus NOTIFY intDMPStatusChanged FINAL)
    Q_PROPERTY(bool isDMPEnabled READ isDMPEnabled WRITE setDMPEnabled NOTIFY dmpEnabledChanged FINAL)
    Q_PROPERTY(bool dmpInt0Status READ dmpInt0Status NOTIFY dmpInt0StatusChanged FINAL)
    Q_PROPERTY(bool dmpInt1Status READ dmpInt1Status NOTIFY dmpInt1StatusChanged FINAL)
    Q_PROPERTY(bool dmpInt2Status READ dmpInt2Status NOTIFY dmpInt2StatusChanged FINAL)
    Q_PROPERTY(bool dmpInt3Status READ dmpInt3Status NOTIFY dmpInt3StatusChanged FINAL)
    Q_PROPERTY(bool dmpInt4Status READ dmpInt4Status NOTIFY dmpInt4StatusChanged FINAL)
    Q_PROPERTY(bool dmpInt5Status READ dmpInt5Status NOTIFY dmpInt5StatusChanged FINAL)
    Q_PROPERTY(bool intDMPEnabled READ intDMPEnabled WRITE setIntDMPEnabled NOTIFY intDMPEnabledChanged FINAL)
    Q_PROPERTY(qint16 zGyroOffsetUser READ zGyroOffsetUser WRITE setZGyroOffsetUser NOTIFY zGyroOffsetUserChanged FINAL)
    Q_PROPERTY(qint16 yGyroOffsetUser READ yGyroOffsetUser WRITE setYGyroOffsetUser NOTIFY yGyroOffsetUserChanged FINAL)
    Q_PROPERTY(qint16 xGyroOffsetUser READ xGyroOffsetUser WRITE setXGyroOffsetUser NOTIFY xGyroOffsetUserChanged FINAL)
    Q_PROPERTY(qint16 zAccelOffset READ zAccelOffset WRITE setZAccelOffset NOTIFY zAccelOffsetChanged FINAL)
    Q_PROPERTY(qint16 yAccelOffset READ yAccelOffset WRITE setYAccelOffset NOTIFY yAccelOffsetChanged FINAL)
    Q_PROPERTY(qint16 xAccelOffset READ xAccelOffset WRITE setXAccelOffset NOTIFY xAccelOffsetChanged FINAL)
    Q_PROPERTY(qint8 zFineGrain READ zFineGrain WRITE setZFineGrain NOTIFY zFineGrainChanged FINAL)
    Q_PROPERTY(qint8 yFineGrain READ yFineGrain WRITE setYFineGrain NOTIFY yFineGrainChanged FINAL)
    Q_PROPERTY(qint8 xFineGrain READ xFineGrain WRITE setXFineGrain NOTIFY xFineGrainChanged FINAL)
    Q_PROPERTY(qint16 zGyroOffset READ zGyroOffset WRITE setZGyroOffset NOTIFY zGyroOffsetChanged FINAL)
    Q_PROPERTY(qint16 yGyroOffset READ yGyroOffset WRITE setYGyroOffset NOTIFY yGyroOffsetChanged FINAL)
    Q_PROPERTY(qint16 xGyroOffset READ xGyroOffset WRITE setXGyroOffset NOTIFY xGyroOffsetChanged FINAL)
    Q_PROPERTY(qint8 deviceID READ deviceID WRITE setDeviceID NOTIFY deviceIDChanged FINAL)
    Q_PROPERTY(bool zGyroStandby READ zGyroStandby WRITE setZGyroStandby NOTIFY zGyroStandbyChanged FINAL)
    Q_PROPERTY(bool yGyroStandby READ yGyroStandby WRITE setYGyroStandby NOTIFY yGyroStandbyChanged FINAL)
    Q_PROPERTY(bool xGyroStandby READ xGyroStandby WRITE setXGyroStandby NOTIFY xGyroStandbyChanged FINAL)
    Q_PROPERTY(bool zAccelStandby READ zAccelStandby WRITE setZAccelStandby NOTIFY zAccelStandbyChanged FINAL)
    Q_PROPERTY(bool yAccelStandby READ yAccelStandby WRITE setYAccelStandby NOTIFY yAccelStandbyChanged FINAL)
    Q_PROPERTY(bool xAccelStandby READ xAccelStandby WRITE setXAccelStandby NOTIFY xAccelStandbyChanged FINAL)
    Q_PROPERTY(WakeFrequency wakeFrequency READ wakeFrequency WRITE setWakeFrequency NOTIFY wakeFrequencyChanged FINAL)
    Q_PROPERTY(ClockSource clockSource READ clockSource WRITE setClockSource NOTIFY clockSourceChanged FINAL)
    Q_PROPERTY(bool isWakeCycleEnabled READ isWakeCycleEnabled WRITE setWakeCycleEnabled NOTIFY wakeCycleEnabledChanged FINAL)
    Q_PROPERTY(bool isTemperatureSensorEnabled READ isTemperatureSensorEnabled WRITE setTemperatureSensorEnabled NOTIFY temperatureSensorEnabledChanged FINAL)
    Q_PROPERTY(bool isSleepEnabled READ isSleepEnabled WRITE setSleepEnabled NOTIFY sleepEnabledChanged FINAL)
    Q_PROPERTY(bool isFIFOEnabled READ isFIFOEnabled WRITE setFIFOEnabled NOTIFY FIFOEnabledChanged FINAL)
    Q_PROPERTY(quint16 FIFOCount READ FIFOCount NOTIFY FIFOCountChanged FINAL)
    Q_PROPERTY(CounterDecrement motionDetectionCounterDecrement READ motionDetectionCounterDecrement WRITE setMotionDetectionCounterDecrement NOTIFY motionDetectionCounterDecrementChanged FINAL)
    Q_PROPERTY(CounterDecrement freefallDetectionCounterDecrement READ freefallDetectionCounterDecrement WRITE setFreefallDetectionCounterDecrement NOTIFY freefallDetectionCounterDecrementChanged FINAL)
    Q_PROPERTY(quint8 accelerometerPowerOnDelay READ accelerometerPowerOnDelay WRITE setAccelerometerPowerOnDelay NOTIFY accelerometerPowerOnDelayChanged FINAL)
    Q_PROPERTY(bool isZeroMotionDetected READ isZeroMotionDetected NOTIFY zeroMotionDetectedChanged FINAL)
    Q_PROPERTY(bool isZPositiveMotionDetected READ isZPositiveMotionDetected NOTIFY zPositiveMotionDetectedChanged FINAL)
    Q_PROPERTY(bool isZNegativeMotionDetected READ isZNegativeMotionDetected NOTIFY zNegativeMotionDetectedChanged FINAL)
    Q_PROPERTY(bool isYPositiveMotionDetected READ isYPositiveMotionDetected NOTIFY yPositiveMotionDetectedChanged FINAL)
    Q_PROPERTY(bool isYNegativeMotionDetected READ isYNegativeMotionDetected NOTIFY yNegativeMotionDetectedChanged FINAL)
    Q_PROPERTY(bool isXPositiveMotionDetected READ isXPositiveMotionDetected NOTIFY xPositiveMotionDetectedChanged FINAL)
    Q_PROPERTY(bool isXNegativeMotionDetected READ isXNegativeMotionDetected NOTIFY xNegativeMotionDetectedChanged FINAL)
    Q_PROPERTY(qint16 zRotation READ zRotation NOTIFY zRotationChanged FINAL)
    Q_PROPERTY(qint16 yRotation READ yRotation NOTIFY yRotationChanged FINAL)
    Q_PROPERTY(qint16 xRotation READ xRotation NOTIFY xRotationChanged FINAL)
    Q_PROPERTY(qint16 temperature READ temperature NOTIFY temperatureChanged FINAL)
    Q_PROPERTY(qint16 zAcceleration READ zAcceleration NOTIFY zAccelerationChanged FINAL)
    Q_PROPERTY(qint16 yAcceleration READ yAcceleration NOTIFY yAccelerationChanged FINAL)
    Q_PROPERTY(qint16 xAcceleration READ xAcceleration NOTIFY xAccelerationChanged FINAL)
    Q_PROPERTY(qint8 zeroMotionDetectionDuration READ zeroMotionDetectionDuration WRITE setZeroMotionDetectionDuration NOTIFY zeroMotionDetectionDurationChanged FINAL)
    Q_PROPERTY(qint8 zeroMotionDetectionThreshold READ zeroMotionDetectionThreshold WRITE setZeroMotionDetectionThreshold NOTIFY zeroMotionDetectionThresholdChanged FINAL)
    Q_PROPERTY(qint8 motionDetectionDuration READ motionDetectionDuration WRITE setMotionDetectionDuration NOTIFY motionDetectionDurationChanged FINAL)
    Q_PROPERTY(qint8 motionDetectionThreshold READ motionDetectionThreshold WRITE setMotionDetectionThreshold NOTIFY motionDetectionThresholdChanged FINAL)
    Q_PROPERTY(qint8 freefallDetectionDuration READ freefallDetectionDuration WRITE setFreefallDetectionDuration NOTIFY freefallDetectionDurationChanged FINAL)
    Q_PROPERTY(qint8 freefallDetectionThreshold READ freefallDetectionThreshold WRITE setFreefallDetectionThreshold NOTIFY freefallDetectionThresholdChanged FINAL)
    Q_PROPERTY(DHPFilterMode dhpFilterMode READ dhpFilterMode WRITE setDhpFilterMode NOTIFY dhpFilterModeChanged FINAL)
    Q_PROPERTY(quint8 fullScaleAccerometerRange READ fullScaleAccerometerRange WRITE setFullScaleAccerometerRange NOTIFY fullScaleAccerometerRangeChanged FINAL)
    Q_PROPERTY(bool isAccelerometerSelfTestZEnabled READ isAccelerometerSelfTestZEnabled WRITE setAccelerometerSelfTestZEnabled NOTIFY accelerometerSelfTestZEnabledChanged FINAL)
    Q_PROPERTY(bool isAccelerometerSelfTestYEnabled READ isAccelerometerSelfTestYEnabled WRITE setAccelerometerSelfTestYEnabled NOTIFY accelerometerSelfTestYEnabledChanged FINAL)
    Q_PROPERTY(bool isAccelerometerSelfTestXEnabled READ isAccelerometerSelfTestXEnabled WRITE setAccelerometerSelfTestXEnabled NOTIFY accelerometerSelfTestXEnabledChanged FINAL)
    Q_PROPERTY(quint16 fullScaleGyroscopeRange READ fullScaleGyroscopeRange WRITE setFullScaleGyroscopeRange NOTIFY fullScaleGyroscopeRangeChanged FINAL)
    Q_PROPERTY(DLPFilterMode dlpFilterMode READ dlpFilterMode WRITE setDlpFilterMode NOTIFY dlpFilterModeChanged FINAL)
    Q_PROPERTY(ExternalFrameSync externalFrameSync READ externalFrameSync WRITE setExternalFrameSync NOTIFY externalFrameSyncChanged FINAL)
    Q_PROPERTY(quint8 gyroscopeRateDivider READ gyroscopeRateDivider WRITE setGyroscopeRateDivider NOTIFY gyroscopeRateDividerChanged FINAL)
};

Q_DECLARE_METATYPE(QMPU6050)

QT_END_NAMESPACE

#endif // QMPU6_5__H
