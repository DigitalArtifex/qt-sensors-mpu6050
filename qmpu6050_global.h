#ifndef QMPU6_5__GLOBAL_H
#define QMPU6_5__GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(QMPU6_5__LIBRARY)
#define QMPU6_5__EXPORT Q_DECL_EXPORT
#else
#define QMPU6_5__EXPORT Q_DECL_IMPORT
#endif

#endif // QT_SENSORS_MPU6_5__GLOBAL_H
