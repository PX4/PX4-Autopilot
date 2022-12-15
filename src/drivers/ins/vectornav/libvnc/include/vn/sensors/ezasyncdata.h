#ifndef _VNEZASYNCDATA_H_
#define _VNEZASYNCDATA_H_

#include "vn/int.h"
#include "vn/error.h"
#include "vn/sensors/compositedata.h"
#include "vn/sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Structure supporting easy and reliable access to asynchronous data
*   from a VectorNav sensor at the cost of a slight performance hit. */
typedef struct
{
	/** \brief The associated connected sensor. */
	VnSensor* sensor;

	/** \brief Critical section for accessing the current data. */
	VnCriticalSection* curDataCS;

	/** \brief The current data received from asynchronous data packets. */
	VnCompositeData* curData;

} VnEzAsyncData;

/** \brief Initializes and connects to a VectorNav sensor with the specified
*   connection parameters.
*
* \param[in] ezAsyncData The associated VnEzAsyncData structure.
* \param]in] portName The name of the serial port to connect to.
* \param[in] baudrate The baudrate to connect at.
* \return Any errors encountered. */
VnError VnEzAsyncData_initializeAndConnect(VnEzAsyncData* ezAsyncData, const char* portName, uint32_t baudrate);

/** \brief Disconnects from a VectorNav sensor.
*
* \param[in] ezAsyncData The associated VnEzAsyncData structure.
* \return Any errors encountered. */
VnError VnEzAsyncData_disconnectAndUninitialize(VnEzAsyncData* ezAsyncData);

/** \brief Returns the most recent asynchronous data the VnEzAsyncData structure
*   has processed.
*
* \param[in] ezAsyncData The associated VnEzAsyncData structure.
* \return The latest data processed. */
VnCompositeData VnEzAsyncData_currentData(VnEzAsyncData* ezAsyncData);

/** \brief Returns the underlying VnSensor referenced.
*
* \param[in] ezAsyncData The associated VnEzAsyncData structure.
* \return The underlying VnSensor reference. */
VnSensor* VnEzAsyncData_sensor(VnEzAsyncData* ezAsyncData);

#ifdef __cplusplus
}
#endif

#endif
