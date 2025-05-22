#include "qmpu6050.h"
#include "qmpu6050backend.h"

QMPU6050::QMPU6050(QObject *parent) : QSensor(sensorType, parent) {}

QAccelerometerReading *QMPU6050::reading() const
{
    return qobject_cast<QAccelerometerReading*>(QSensor::reading());
}

bool QMPU6050::initialize()
{
    if(m_controller)
        return m_controller->initialize();

    return false;
}

QString QMPU6050::bus() const
{
    return m_bus;
}

void QMPU6050::setBus(const QString &bus)
{
    if (m_bus == bus)
        return;

    if(controller())
        controller()->setBus(bus);

    m_bus = bus;
    emit busChanged();
}

quint8 QMPU6050::address() const
{
    return m_address;
}

void QMPU6050::setAddress(quint8 address)
{
    if (m_address == address)
        return;

    if(controller())
        controller()->setAddress(address);

    m_address = address;
    emit addressChanged();
}

quint8 QMPU6050::dmpConfig2() const
{
    return m_dmpConfig2;
}

void QMPU6050::setDmpConfig2(quint8 dmpConfig2)
{
    if (m_dmpConfig2 == dmpConfig2)
        return;

    if(controller() && controller()->setDMPConfig2(dmpConfig2))
    {
        m_dmpConfig2 = dmpConfig2;
        emit dmpConfig2Changed();
    }
}

quint8 QMPU6050::dmpConfig1() const
{
    return m_dmpConfig1;
}

void QMPU6050::setDmpConfig1(quint8 dmpConfig1)
{
    if (m_dmpConfig1 == dmpConfig1)
        return;

    if(controller() && controller()->setDMPConfig1(dmpConfig1))
    {
        m_dmpConfig1 = dmpConfig1;
        emit dmpConfig1Changed();
    }
}

QMPU6050Backend *QMPU6050::controller() const
{
    return m_controller;
}

bool QMPU6050::isIntPLLReadyEnabled() const
{
    return m_isIntPLLReadyEnabled;
}

void QMPU6050::setIntPLLReadyEnabled(bool intPLLReadyEnabled)
{
    if (m_isIntPLLReadyEnabled == intPLLReadyEnabled)
        return;

    if(controller() && controller()->setIntPLLReadyEnabled(intPLLReadyEnabled))
    {
        m_isIntPLLReadyEnabled = intPLLReadyEnabled;
        emit intPLLReadyEnabledChanged();
    }
}

bool QMPU6050::intPLLReadyStatus() const
{
    return m_intPLLReadyStatus;
}

bool QMPU6050::intDMPStatus() const
{
    return m_intDMPStatus;
}

bool QMPU6050::isDMPEnabled() const
{
    return m_isDMPEnabled;
}

void QMPU6050::setDMPEnabled(bool dmpEnabled)
{
    if (m_isDMPEnabled == dmpEnabled)
        return;

    if(controller() && controller()->setDMPEnabled(dmpEnabled))
    {
        m_isDMPEnabled = dmpEnabled;
        emit dmpEnabledChanged();
    }
}

bool QMPU6050::dmpInt0Status() const
{
    return m_dmpInt0Status;
}

bool QMPU6050::dmpInt1Status() const
{
    return m_dmpInt1Status;
}

bool QMPU6050::dmpInt2Status() const
{
    return m_dmpInt2Status;
}

bool QMPU6050::dmpInt3Status() const
{
    return m_dmpInt3Status;
}

bool QMPU6050::dmpInt4Status() const
{
    return m_dmpInt4Status;
}

bool QMPU6050::dmpInt5Status() const
{
    return m_dmpInt5Status;
}

bool QMPU6050::intDMPEnabled() const
{
    return m_intDMPEnabled;
}

void QMPU6050::setIntDMPEnabled(bool intDMPEnabled)
{
    if (m_intDMPEnabled == intDMPEnabled)
        return;

    if(controller() && controller()->setIntDMPEnabled(intDMPEnabled))
    {
        m_intDMPEnabled = intDMPEnabled;
        emit intDMPEnabledChanged();
    }
}

qint16 QMPU6050::zGyroOffsetUser() const
{
    return m_zGyroOffsetUser;
}

void QMPU6050::setZGyroOffsetUser(qint16 zGyroOffsetUser)
{
    if (m_zGyroOffsetUser == zGyroOffsetUser)
        return;

    if(controller() && controller()->setZGyroOffsetUser(zGyroOffsetUser))
    {
        m_zGyroOffsetUser = zGyroOffsetUser;
        emit zGyroOffsetUserChanged();
    }
}

qint16 QMPU6050::yGyroOffsetUser() const
{
    return m_yGyroOffsetUser;
}

void QMPU6050::setYGyroOffsetUser(qint16 yGyroOffsetUser)
{
    if (m_yGyroOffsetUser == yGyroOffsetUser)
        return;

    if(controller() && controller()->setYGyroOffsetUser(yGyroOffsetUser))
    {
        m_yGyroOffsetUser = yGyroOffsetUser;
        emit yGyroOffsetUserChanged();
    }
}

qint16 QMPU6050::xGyroOffsetUser() const
{
    return m_xGyroOffsetUser;
}

void QMPU6050::setXGyroOffsetUser(qint16 xGyroOffsetUser)
{
    if (m_xGyroOffsetUser == xGyroOffsetUser)
        return;

    if(controller() && controller()->setXGyroOffsetUser(xGyroOffsetUser))
    {
        m_xGyroOffsetUser = xGyroOffsetUser;
        emit xGyroOffsetUserChanged();
    }
}

qint16 QMPU6050::zAccelOffset() const
{
    return m_zAccelOffset;
}

void QMPU6050::setZAccelOffset(qint16 zAccelOffset)
{
    if (m_zAccelOffset == zAccelOffset)
        return;

    if(controller() && controller()->setZAccelOffset(zAccelOffset))
    {
        m_zAccelOffset = zAccelOffset;
        emit zAccelOffsetChanged();
    }
}

qint16 QMPU6050::yAccelOffset() const
{
    return m_yAccelOffset;
}

void QMPU6050::setYAccelOffset(qint16 yAccelOffset)
{
    if (m_yAccelOffset == yAccelOffset)
        return;

    if(controller() && controller()->setYAccelOffset(yAccelOffset))
    {
        m_yAccelOffset = yAccelOffset;
        emit yAccelOffsetChanged();
    }
}

qint16 QMPU6050::xAccelOffset() const
{
    return m_xAccelOffset;
}

void QMPU6050::setXAccelOffset(qint16 xAccelOffset)
{
    if (m_xAccelOffset == xAccelOffset)
        return;

    if(controller() && controller()->setXAccelOffset(xAccelOffset))
    {
        m_xAccelOffset = xAccelOffset;
        emit xAccelOffsetChanged();
    }
}

qint8 QMPU6050::zFineGrain() const
{
    return m_zFineGrain;
}

void QMPU6050::setZFineGrain(qint8 zFineGrain)
{
    if (m_zFineGrain == zFineGrain)
        return;

    if(controller() && controller()->setZFineGain(zFineGrain))
    {
        m_zFineGrain = zFineGrain;
        emit zFineGrainChanged();
    }
}

qint8 QMPU6050::yFineGrain() const
{
    return m_yFineGrain;
}

void QMPU6050::setYFineGrain(qint8 yFineGrain)
{
    if (m_yFineGrain == yFineGrain)
        return;

    if(controller() && controller()->setYFineGain(yFineGrain))
    {
        m_yFineGrain = yFineGrain;
        emit yFineGrainChanged();
    }
}

qint8 QMPU6050::xFineGrain() const
{
    return m_xFineGrain;
}

void QMPU6050::setXFineGrain(qint8 xFineGrain)
{
    if (m_xFineGrain == xFineGrain)
        return;

    if(controller() && controller()->setXFineGain(xFineGrain))
    {
        m_xFineGrain = xFineGrain;
        emit xFineGrainChanged();
    }
}

qint16 QMPU6050::zGyroOffset() const
{
    return m_zGyroOffset;
}

void QMPU6050::setZGyroOffset(qint16 zGyroOffset)
{
    if (m_zGyroOffset == zGyroOffset)
        return;

    if(controller() && controller()->setZGyroOffset(zGyroOffset))
    {
        m_zGyroOffset = zGyroOffset;
        emit zGyroOffsetChanged();
    }
}

qint16 QMPU6050::yGyroOffset() const
{
    return m_yGyroOffset;
}

void QMPU6050::setYGyroOffset(qint16 yGyroOffset)
{
    if (m_yGyroOffset == yGyroOffset)
        return;

    if(controller() && controller()->setYGyroOffset(yGyroOffset))
    {
        m_yGyroOffset = yGyroOffset;
        emit yGyroOffsetChanged();
    }
}

qint16 QMPU6050::xGyroOffset() const
{
    return m_xGyroOffset;
}

void QMPU6050::setXGyroOffset(qint16 xGyroOffset)
{
    if (m_xGyroOffset == xGyroOffset)
        return;

    if(controller() && controller()->setXGyroOffset(xGyroOffset))
    {
        m_xGyroOffset = xGyroOffset;
        emit xGyroOffsetChanged();
    }
}

qint8 QMPU6050::deviceID() const
{
    return m_deviceID;
}

void QMPU6050::setDeviceID(qint8 deviceID)
{
    if (m_deviceID == deviceID)
        return;

    m_deviceID = deviceID;
    emit deviceIDChanged();
}

bool QMPU6050::updateDeviceID(qint8 deviceID)
{
    if(controller() && controller()->setDeviceID(deviceID))
    {
        setDeviceID(deviceID);
        return true;
    }

    return false;
}

bool QMPU6050::zGyroStandby() const
{
    return m_zGyroStandby;
}

void QMPU6050::setZGyroStandby(bool zGyroStandby)
{
    if (m_zGyroStandby == zGyroStandby)
        return;

    if(controller() && controller()->setStandbyZGyroEnabled(zGyroStandby))
    {
        m_zGyroStandby = zGyroStandby;
        emit zGyroStandbyChanged();
    }
}

bool QMPU6050::yGyroStandby() const
{
    return m_yGyroStandby;
}

void QMPU6050::setYGyroStandby(bool yGyroStandby)
{
    if (m_yGyroStandby == yGyroStandby)
        return;

    if(controller() && controller()->setStandbyYGyroEnabled(yGyroStandby))
    {
        m_yGyroStandby = yGyroStandby;
        emit yGyroStandbyChanged();
    }
}

bool QMPU6050::xGyroStandby() const
{
    return m_xGyroStandby;
}

void QMPU6050::setXGyroStandby(bool xGyroStandby)
{
    if (m_xGyroStandby == xGyroStandby)
        return;

    if(controller() && controller()->setStandbyXGyroEnabled(xGyroStandby))
    {
        m_xGyroStandby = xGyroStandby;
        emit xGyroStandbyChanged();
    }
}

bool QMPU6050::zAccelStandby() const
{
    return m_zAccelStandby;
}

void QMPU6050::setZAccelStandby(bool zAccelStandby)
{
    if (m_zAccelStandby == zAccelStandby)
        return;

    if(controller() && controller()->setStandbyZAccelEnabled(zAccelStandby))
    {
        m_zAccelStandby = zAccelStandby;
        emit zAccelStandbyChanged();
    }
}

bool QMPU6050::yAccelStandby() const
{
    return m_yAccelStandby;
}

void QMPU6050::setYAccelStandby(bool yAccelStandby)
{
    if (m_yAccelStandby == yAccelStandby)
        return;

    if(controller() && controller()->setStandbyYAccelEnabled(yAccelStandby))
    {
        m_yAccelStandby = yAccelStandby;
        emit yAccelStandbyChanged();
    }
}

bool QMPU6050::xAccelStandby() const
{
    return m_xAccelStandby;
}

void QMPU6050::setXAccelStandby(bool xAccelStandby)
{
    if (m_xAccelStandby == xAccelStandby)
        return;

    if(controller() && controller()->setStandbyXAccelEnabled(xAccelStandby))
    {
        m_xAccelStandby = xAccelStandby;
        emit xAccelStandbyChanged();
    }
}

QMPU6050::WakeFrequency QMPU6050::wakeFrequency() const
{
    return m_wakeFrequency;
}

void QMPU6050::setWakeFrequency(WakeFrequency wakeFrequency)
{
    if (m_wakeFrequency == wakeFrequency)
        return;

    if(controller() && controller()->setWakeFrequency(static_cast<quint8>(wakeFrequency)))
    {
        m_wakeFrequency = wakeFrequency;
        emit wakeFrequencyChanged();
    }
}

QMPU6050::ClockSource QMPU6050::clockSource() const
{
    return m_clockSource;
}

void QMPU6050::setClockSource(ClockSource clockSource)
{
    if (m_clockSource == clockSource)
        return;

    if(controller() && controller()->setClockSource(static_cast<quint8>(clockSource)))
    {
        m_clockSource = clockSource;
        emit clockSourceChanged();
    }
}

bool QMPU6050::isWakeCycleEnabled() const
{
    return m_isWakeCycleEnabled;
}

void QMPU6050::setWakeCycleEnabled(bool wakeCycleEnabled)
{
    if (m_isWakeCycleEnabled == wakeCycleEnabled)
        return;

    if(controller() && controller()->setWakeCycleEnabled(wakeCycleEnabled))
    {
        m_isWakeCycleEnabled = wakeCycleEnabled;
        emit wakeCycleEnabledChanged();
    }
}

bool QMPU6050::isTemperatureSensorEnabled() const
{
    return m_isTemperatureSensorEnabled;
}

void QMPU6050::setTemperatureSensorEnabled(bool temperatureSensorEnabled)
{
    if (m_isTemperatureSensorEnabled == temperatureSensorEnabled)
        return;

    if(controller() && controller()->setTempSensorEnabled(temperatureSensorEnabled))
    {
        m_isTemperatureSensorEnabled = temperatureSensorEnabled;
        emit temperatureSensorEnabledChanged();
    }
}

bool QMPU6050::isSleepEnabled() const
{
    return m_isSleepEnabled;
}

void QMPU6050::setSleepEnabled(bool sleepEnabled)
{
    if (m_isSleepEnabled == sleepEnabled)
        return;

    if(controller() && controller()->setSleepEnabled(sleepEnabled))
    {
        m_isSleepEnabled = sleepEnabled;
        emit sleepEnabledChanged();
    }
}

bool QMPU6050::isFIFOEnabled() const
{
    return m_isFIFOEnabled;
}

void QMPU6050::setFIFOEnabled(bool FIFOEnabled)
{
    if (m_isFIFOEnabled == FIFOEnabled)
        return;

    if(controller() && controller()->setFIFOEnabled(FIFOEnabled))
    {
        m_isFIFOEnabled = FIFOEnabled;
        emit FIFOEnabledChanged();
    }
}

quint16 QMPU6050::FIFOCount() const
{
    return m_FIFOCount;
}

QMPU6050::CounterDecrement QMPU6050::motionDetectionCounterDecrement() const
{
    return m_motionDetectionCounterDecrement;
}

void QMPU6050::setMotionDetectionCounterDecrement(CounterDecrement motionDetectionCounterDecrement)
{
    if (m_motionDetectionCounterDecrement == motionDetectionCounterDecrement)
        return;

    if(controller() && controller()->setMotionDetectionCounterDecrement(static_cast<quint8>(motionDetectionCounterDecrement)))
    {
        m_motionDetectionCounterDecrement = motionDetectionCounterDecrement;
        emit motionDetectionCounterDecrementChanged();
    }
}

QMPU6050::CounterDecrement QMPU6050::freefallDetectionCounterDecrement() const
{
    return m_freefallDetectionCounterDecrement;
}

void QMPU6050::setFreefallDetectionCounterDecrement(CounterDecrement freefallDetectionCounterDecrement)
{
    if (m_freefallDetectionCounterDecrement == freefallDetectionCounterDecrement)
        return;

    if(controller() && controller()->setFreefallDetectionCounterDecrement(static_cast<quint8>(freefallDetectionCounterDecrement)))
    {
        m_freefallDetectionCounterDecrement = freefallDetectionCounterDecrement;
        emit freefallDetectionCounterDecrementChanged();
    }
}

quint8 QMPU6050::accelerometerPowerOnDelay() const
{
    return m_accelerometerPowerOnDelay;
}

void QMPU6050::setAccelerometerPowerOnDelay(quint8 accelerometerPowerOnDelay)
{
    if(accelerometerPowerOnDelay > 3)
        accelerometerPowerOnDelay = 3;

    if (m_accelerometerPowerOnDelay == accelerometerPowerOnDelay)
        return;

    if(controller() && controller()->setAccelerometerPowerOnDelay(accelerometerPowerOnDelay))
    {
        m_accelerometerPowerOnDelay = accelerometerPowerOnDelay;
        emit accelerometerPowerOnDelayChanged();
    }
}

bool QMPU6050::isZeroMotionDetected() const
{
    return m_isZeroMotionDetected;
}

bool QMPU6050::isZPositiveMotionDetected() const
{
    return m_isZPositiveMotionDetected;
}

bool QMPU6050::isZNegativeMotionDetected() const
{
    return m_isZNegativeMotionDetected;
}

bool QMPU6050::isYPositiveMotionDetected() const
{
    return m_isYPositiveMotionDetected;
}

bool QMPU6050::isYNegativeMotionDetected() const
{
    return m_isYNegativeMotionDetected;
}

bool QMPU6050::isXPositiveMotionDetected() const
{
    return m_isXPositiveMotionDetected;
}

bool QMPU6050::isXNegativeMotionDetected() const
{
    return m_isXNegativeMotionDetected;
}

qint16 QMPU6050::zRotation() const
{
    return m_zRotation;
}

qint16 QMPU6050::yRotation() const
{
    return m_yRotation;
}

qint16 QMPU6050::xRotation() const
{
    return m_xRotation;
}

qint16 QMPU6050::temperature() const
{
    return m_temperature;
}

qint16 QMPU6050::zAcceleration() const
{
    return m_zAcceleration;
}

qint16 QMPU6050::yAcceleration() const
{
    return m_yAcceleration;
}

qint16 QMPU6050::xAcceleration() const
{
    return m_xAcceleration;
}

qint8 QMPU6050::zeroMotionDetectionDuration() const
{
    return m_zeroMotionDetectionDuration;
}

void QMPU6050::setZeroMotionDetectionDuration(qint8 zeroMotionDetectionDuration)
{
    if (m_zeroMotionDetectionDuration == zeroMotionDetectionDuration)
        return;

    if(controller() && controller()->setZeroMotionDetectionDuration(zeroMotionDetectionDuration))
    {
        m_zeroMotionDetectionDuration = zeroMotionDetectionDuration;
        emit zeroMotionDetectionDurationChanged();
    }
}

qint8 QMPU6050::zeroMotionDetectionThreshold() const
{
    return m_zeroMotionDetectionThreshold;
}

void QMPU6050::setZeroMotionDetectionThreshold(qint8 zeroMotionDetectionThreshold)
{
    if (m_zeroMotionDetectionThreshold == zeroMotionDetectionThreshold)
        return;

    if(controller() && controller()->setZeroMotionDetectionThreshold(zeroMotionDetectionThreshold))
    {
        m_zeroMotionDetectionThreshold = zeroMotionDetectionThreshold;
        emit zeroMotionDetectionThresholdChanged();
    }
}

qint8 QMPU6050::motionDetectionDuration() const
{
    return m_motionDetectionDuration;
}

void QMPU6050::setMotionDetectionDuration(qint8 motionDetectionDuration)
{
    if (m_motionDetectionDuration == motionDetectionDuration)
        return;

    if(controller() && controller()->setMotionDetectionDuration(motionDetectionDuration))
    {
        m_motionDetectionDuration = motionDetectionDuration;
        emit motionDetectionDurationChanged();
    }
}

qint8 QMPU6050::motionDetectionThreshold() const
{
    return m_motionDetectionThreshold;
}

void QMPU6050::setMotionDetectionThreshold(qint8 motionDetectionThreshold)
{
    if (m_motionDetectionThreshold == motionDetectionThreshold)
        return;

    if(controller() && controller()->setMotionDetectionThreshold(motionDetectionThreshold))
    {
        m_motionDetectionThreshold = motionDetectionThreshold;
        emit motionDetectionThresholdChanged();
    }
}

qint8 QMPU6050::freefallDetectionDuration() const
{
    return m_freefallDetectionDuration;
}

void QMPU6050::setFreefallDetectionDuration(qint8 freefallDetectionDuration)
{
    if (m_freefallDetectionDuration == freefallDetectionDuration)
        return;

    if(controller() && controller()->setFreefallDetectionDuration(freefallDetectionDuration))
    {
        m_freefallDetectionDuration = freefallDetectionDuration;
        emit freefallDetectionDurationChanged();
    }
}

qint8 QMPU6050::freefallDetectionThreshold() const
{
    return m_freefallDetectionThreshold;
}

void QMPU6050::setFreefallDetectionThreshold(qint8 freefallDetectionThreshold)
{
    if (m_freefallDetectionThreshold == freefallDetectionThreshold)
        return;

    if(controller() && controller()->setFreefallDetectionThreshold(freefallDetectionThreshold))
    {
        m_freefallDetectionThreshold = freefallDetectionThreshold;
        emit freefallDetectionThresholdChanged();
    }
}

QMPU6050::DHPFilterMode QMPU6050::dhpFilterMode() const
{
    return m_dhpFilterMode;
}

void QMPU6050::setDhpFilterMode(DHPFilterMode dhpFilterMode)
{
    if (m_dhpFilterMode == dhpFilterMode)
        return;

    if(controller() && controller()->setDHPFMode(static_cast<quint8>(dhpFilterMode)))
    {
        m_dhpFilterMode = dhpFilterMode;
        emit dhpFilterModeChanged();
    }
}

quint8 QMPU6050::fullScaleAccerometerRange() const
{
    return m_fullScaleAccerometerRange;
}

void QMPU6050::setFullScaleAccerometerRange(quint8 fullScaleAccerometerRange)
{
    switch (fullScaleAccerometerRange)
    {
    case 2:
        fullScaleAccerometerRange = 0;
        break;
    case 4:
        fullScaleAccerometerRange = 1;
        break;
    case 8:
        fullScaleAccerometerRange = 2;
        break;
    case 16:
        fullScaleAccerometerRange = 3;
        break;
    default:
        break;
    }

    if (m_fullScaleAccerometerRange == fullScaleAccerometerRange)
        return;

    if(controller() && controller()->setFullScaleAccelRange(fullScaleAccerometerRange))
    {
        m_fullScaleAccerometerRange = fullScaleAccerometerRange;
        emit fullScaleAccerometerRangeChanged();
    }
}

bool QMPU6050::isAccelerometerSelfTestZEnabled() const
{
    return m_isAccelerometerSelfTestZEnabled;
}

void QMPU6050::setAccelerometerSelfTestZEnabled(bool accelerometerSelfTestZEnabled)
{
    if (m_isAccelerometerSelfTestZEnabled == accelerometerSelfTestZEnabled)
        return;

    if(controller() && controller()->setAccelZSelfTest(accelerometerSelfTestZEnabled))
    {
        m_isAccelerometerSelfTestZEnabled = accelerometerSelfTestZEnabled;
        emit accelerometerSelfTestZEnabledChanged();
    }
}

bool QMPU6050::isAccelerometerSelfTestYEnabled() const
{
    return m_isAccelerometerSelfTestYEnabled;
}

void QMPU6050::setAccelerometerSelfTestYEnabled(bool accelerometerSelfTestYEnabled)
{
    if (m_isAccelerometerSelfTestYEnabled == accelerometerSelfTestYEnabled)
        return;

    if(controller() && controller()->setAccelYSelfTest(accelerometerSelfTestYEnabled))
    {
        m_isAccelerometerSelfTestYEnabled = accelerometerSelfTestYEnabled;
        emit accelerometerSelfTestYEnabledChanged();
    }
}

bool QMPU6050::isAccelerometerSelfTestXEnabled() const
{
    return m_isAccelerometerSelfTestXEnabled;
}

void QMPU6050::setAccelerometerSelfTestXEnabled(bool accelerometerSelfTestXEnabled)
{
    if (m_isAccelerometerSelfTestXEnabled == accelerometerSelfTestXEnabled)
        return;

    if(controller() && controller()->setAccelXSelfTest(accelerometerSelfTestXEnabled))
    {
        m_isAccelerometerSelfTestXEnabled = accelerometerSelfTestXEnabled;
        emit accelerometerSelfTestXEnabledChanged();
    }
}

quint16 QMPU6050::fullScaleGyroscopeRange() const
{
    return m_fullScaleGyroscopeRange;
}

void QMPU6050::setFullScaleGyroscopeRange(quint16 fullScaleGyroscopeRange)
{
    if (m_fullScaleGyroscopeRange == fullScaleGyroscopeRange)
        return;

    if(controller() && controller()->setFullScaleGyroRange(fullScaleGyroscopeRange))
    {
        m_fullScaleGyroscopeRange = fullScaleGyroscopeRange;
        emit fullScaleGyroscopeRangeChanged();
    }
}

QMPU6050::DLPFilterMode QMPU6050::dlpFilterMode() const
{
    return m_dlpFilterMode;
}

void QMPU6050::setDlpFilterMode(DLPFilterMode dlpFilterMode)
{
    if (m_dlpFilterMode == dlpFilterMode)
        return;

    if(controller() && controller()->setDLPFMode(static_cast<quint8>(dlpFilterMode)))
    {
        m_dlpFilterMode = dlpFilterMode;
        emit dlpFilterModeChanged();
    }
}

QMPU6050::ExternalFrameSync QMPU6050::externalFrameSync() const
{
    return m_externalFrameSync;
}

void QMPU6050::setExternalFrameSync(ExternalFrameSync externalFrameSync)
{
    if (m_externalFrameSync == externalFrameSync)
        return;

    if(controller() && controller()->setExternalFrameSync(static_cast<quint8>(externalFrameSync)))
    {
        m_externalFrameSync = externalFrameSync;
        emit externalFrameSyncChanged();
    }
}

quint8 QMPU6050::gyroscopeRateDivider() const
{
    return m_gyroscopeRateDivider;
}

void QMPU6050::setGyroscopeRateDivider(quint8 gyroscopeRateDivider)
{
    if (m_gyroscopeRateDivider == gyroscopeRateDivider)
        return;

    if(controller() && controller()->setRate(gyroscopeRateDivider))
    {
        m_gyroscopeRateDivider = gyroscopeRateDivider;
        emit gyroscopeRateDividerChanged();
    }
}
