#include "Adafruit_Sensor.h"

////////////////////////////////////////////////////////////////////////////////
// Linux porting (By. YongJae Kim)
////////////////////////////////////////////////////////////////////////////////
#include "stdio.h"
#define PRINTF(...)	printf(__VA_ARGS__)
////////////////////////////////////////////////////////////////////////////////

/**************************************************************************/
/*!
    @brief  Prints sensor information to serial console
*/
/**************************************************************************/
void Adafruit_Sensor::printSensorDetails(void) {
  sensor_t sensor;
  getSensor(&sensor);
  PRINTF("------------------------------------\n");
  PRINTF("Sensor:       %s\n", sensor.name);
  PRINTF("Type:         ");
  switch ((sensors_type_t)sensor.type) {
  case SENSOR_TYPE_ACCELEROMETER:
    PRINTF("Acceleration (m/s2)");
    break;
  case SENSOR_TYPE_MAGNETIC_FIELD:
    PRINTF("Magnetic (uT)");
    break;
  case SENSOR_TYPE_ORIENTATION:
    PRINTF("Orientation (degrees)");
    break;
  case SENSOR_TYPE_GYROSCOPE:
    PRINTF("Gyroscopic (rad/s)");
    break;
  case SENSOR_TYPE_LIGHT:
    PRINTF("Light (lux)");
    break;
  case SENSOR_TYPE_PRESSURE:
    PRINTF("Pressure (hPa)");
    break;
  case SENSOR_TYPE_PROXIMITY:
    PRINTF("Distance (cm)");
    break;
  case SENSOR_TYPE_GRAVITY:
    PRINTF("Gravity (m/s2)");
    break;
  case SENSOR_TYPE_LINEAR_ACCELERATION:
    PRINTF("Linear Acceleration (m/s2)");
    break;
  case SENSOR_TYPE_ROTATION_VECTOR:
    PRINTF("Rotation vector");
    break;
  case SENSOR_TYPE_RELATIVE_HUMIDITY:
    PRINTF("Relative Humidity (%%)");
    break;
  case SENSOR_TYPE_AMBIENT_TEMPERATURE:
    PRINTF("Ambient Temp (C)");
    break;
  case SENSOR_TYPE_OBJECT_TEMPERATURE:
    PRINTF("Object Temp (C)");
    break;
  case SENSOR_TYPE_VOLTAGE:
    PRINTF("Voltage (V)");
    break;
  case SENSOR_TYPE_CURRENT:
    PRINTF("Current (mA)");
    break;
  case SENSOR_TYPE_COLOR:
    PRINTF("Color (RGBA)");
    break;
  }

  PRINTF("\n");
  PRINTF("Driver Ver:   %d\n", sensor.version);
  PRINTF("Unique ID:    %d\n", sensor.sensor_id);
  PRINTF("Min Value:    %f\n", sensor.min_value);
  PRINTF("Max Value:    %f\n", sensor.max_value);
  PRINTF("Resolution:   %f\n", sensor.resolution);
  PRINTF("------------------------------------\n");
}
