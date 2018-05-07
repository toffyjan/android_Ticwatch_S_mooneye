/* CWMCU.h - header file for CyWee digital 3-axis gyroscope
 *
 * Copyright (C) 2010 CyWee Group Ltd.
 * Author: Joe Wei <joewei@cywee.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __CWMCUSENSOR_H__
#define __CWMCUSENSOR_H__
#include <linux/ioctl.h>
#include "SensorSupport.h"
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/irq_work.h>
#include <linux/ioctl.h>
#include <sensors_io.h>
#include <linux/i2c.h>



#define CWMCU_I2C_NAME "CwMcuSensor"
#define CWMCU_SPI_NAME "CwMcuSensor"
#define LOG_TAG_HAL "CwHal"
#define LOG_TAG_KERNEL "CwKernel"
#define LOG_TAG_MCU "CwMcu"

/* turn on gpio interrupt if defined */
#define CWM_USE_IRQ_WORK
//#define CWM_USE_DELAY_WORK
#define CWM_USE_ERROR_HANDLE_WORK

#define CWMCU_MUTEX

#define CWM_USE_TIME_SYNC_WORK
#define CMD_BUFFER_SIZE			16
#define SYNC_TIME_DELAY_MS (60000)



enum SCAN_status {
	CW_SCAN_ID = 0,
	CW_SCAN_X,
	CW_SCAN_Y,
	CW_SCAN_Z,
	CW_SCAN_XX,
	CW_SCAN_YY,
	CW_SCAN_ZZ,
	CW_SCAN_TIMESTAMP,
};

enum MCU_mode {
	CW_NORMAL = 0x00,
	CW_SLEEP,
	CW_NO_SLEEP,
	CW_BOOT
};

/* power manager status */
typedef enum {
	SWITCH_POWER_ENABLE = 0,
	SWITCH_POWER_DELAY,
	SWITCH_POWER_BATCH,
	SWITCH_POWER_FLUSH,
	SWITCH_POWER_NORMAL,
	SWITCH_POWER_CALIB,
	SWITCH_POWER_INTERRUPT,
	SWITCH_POWER_POLLING,
	SWITCH_POWER_PROBE,
	SWITCH_POWER_SHUTDOWN,
	SWITCH_POWER_LOG,
	SWITCH_POWER_FIRMWARE_COMMAND,
	SWITCH_POWER_VERSION,
	SWITCH_POWER_TIME,
	SWITCH_POWER_SYS,
	SWITCH_POWER_MCU_GPIO
} SWITCH_POWER_ID;

typedef enum {
	ERR_TASK_BLOCK = -10,
	SEND_QUEUE_FULL = -9,
	DRIVER_CHECK_CHIP_ID_FAIL = -8,
	DRIVER_ENABLE_FAIL = -7,
	DRIVER_DISABLE_FAIL = -6,
	DRIVER_GETDATA_FAIL = -5,
	I2C_FAIL = -4,
	DRIVER_NO_USE = -3,
	SENSORS_NO_INITIAL = -2,
	FAIL = -1,
	NO_ERROR = 0,
	NO_DATA = 1
} ERR_MSG;

/* interrupt status */
typedef enum {
	IRQ_INIT = 0,
	IRQ_DATA_READY = 1,
	IRQ_GESTURE = 2,
	IRQ_BATCH_TIMEOUT = 3,
	IRQ_BATCH_FULL = 4,
	IRQ_INFO = 5,
	IRQ_CALIB = 6,
	IRQ_ERROR = 7,
	IRQ_INPUT = 8,
	IRQ_HALL = 9,
	IRQ_HRLOG = 10,
	IRQ_MAX_SIZE
} IRQ_STATUS_LIST;

typedef enum {
	KERNEL_NON =
	    0, KERNEL_PROBE, KERNEL_SUSPEND, KERNEL_RESUME, KERNEL_SHUTDOWN
} KERNEL_STATUS;

typedef enum {
	CALIB_TYPE_NON = 0,
	CALIB_TYPE_DEFAULT = 1,
	CALIB_TYPE_SELFTEST = 2,
	CALIB_TYPE_SENSORS_ENABLE = 3,
	CALIB_TYPE_SENSORS_DISABLE = 4,
	CALIB_TYPE_CHIP_ID = 5,
} CALIBRATOR_TYPE;

typedef enum {
	CALIB_STATUS_OUT_OF_RANGE = -2,
	CALIB_STATUS_FAIL = -1,
	CALIB_STATUS_NON = 0,
	CALIB_STATUS_INPROCESS = 1,
	CALIB_STATUS_PASS = 2,
} CALIBRATOR_STATUS;

typedef enum {
	D_IRQ = 0,
	D_IIO_DATA = 1,
	D_EN = 2,
	D_CALIB = 3,
	D_DELAY_WQ = 4,
} DEBUG_E;

enum LOGCAT_STATUS {
	EVENT_COUNT = 1,
	EVENT_DATA
};

typedef enum {
	SYSCMD = 0,
	SYSINFO = 1,
	ACCESSORYINFO,
	GPSINFO,
	WIFIINFO,
	HEALTHINFO,
	TIMEINFO,
} INFO_Type;

/* calibrator command format */
/*
    CALIB_EN:
        cmd/id/type/

    CALIB_CHECK_STATUS:
    CALIB_DATA_WRITE:
    CALIB_DATA_READ:
    CALIB_FLASH_WRITE:
    CALIB_FLASH_READ:
        cmd/id/

    Acc/Gyro flow:
        echo CALIB_EN/(ACCELERATION or GYRO)/CALIB_TYPE_DEFAULT > calibrator_cmd
        echo CALIB_CHECK_STATUS/(ACCELERATION or GYRO)/CALIB_TYPE_DEFAULT > calibrator_cmd
        do{
            cat calibrator_data
               data format:
               [0] i2c error code
               [1] calibrator status
        }while(calibrator status != CALIB_STATUS_INPROCESS);
        if(status == CALIB_STATUS_PASS)
            echo CALIB_DATA_READ/(ACCELERATION or GYRO) > calibrator_cmd
            cat calibrator_data
               data format:
               [0] i2c error code
               [1:4] (int)X bias
               [5:8] (int)Y bias
               [9:12] (int)Z bias

        If Mcu reset or system reboot: need reload calibrator data to MCU
            echo CALIB_DATA_WRITE/(ACCELERATION or GYRO) > calibrator_cmd
            echo [0:29] > calibrator_data
               data format:
               [0:3] (int)X bias
               [4:7] (int)Y bias
               [8:11] (int)Z bias

    Mag flow:
        When system boot up or MCU reset, reload data to MCU
            echo CALIB_DATA_WRITE/MAGNETIC > calibrator_cmd
            echo [0:29] > calibrator_data
               data format:
               [0:29] mag private data

        When sensors disable need save MCU mag calibrator data to system
            echo CALIB_DATA_READ/MAGNETIC > calibrator_cmd
            cat calibrator_data
               data format:
               [0] i2c error code
               [1:30] mag private data

    Proximity flow:
        echo CALIB_EN/PROXIMITY/CALIB_TYPE_SENSORS_ENABLE > calibrator_cmd
        echo CALIB_DATA_READ/PROXIMITY > calibrator_cmd
        while(1)
            cat calibrator_data
               data format:
               [0] i2c error code
               [1:4]light bias(not use)
               [5:8](int) is Hight threshold to check sensors is near
               [9:12](int) is low threshold to check sensors is far
               [13:16](int) Sensors current raw data
        if calibrator successful:
        echo CALIB_DATA_WRITE/PROXIMITY > calibrator_cmd
        echo [0:29] > calibrator_data
           data format:
           [0:3]light bias(not use)
           [4:7](int) is Hight threshold to check sensors is near
           [8:11](int) is low threshold to check sensors is far
        disable sensors
        echo CALIB_EN/PROXIMITY/CALIB_TYPE_SENSORS_DISABLE > calibrator_cmd
*/

typedef enum {
	CALIB_EN = 0,
	CALIB_CHECK_STATUS,
	CALIB_DATA_WRITE,
	CALIB_DATA_READ,
	CALIB_FLASH_WRITE,
	CALIB_FLASH_READ,
	CALIB_GET_SEN_DATA,	//X:[0:3];Z:[4:7];Z:[8:11]; data format: android format * 10000
	CALIB_GET_ALL_CALIB_STATUS,
	CALIB_GET_ALL_SELFTEST_STATUS,
	CALIB_EN_SPECIAL_SENSOR,
	CALIB_CHECK_SPECIAL_SENSOR_STATUS,
	CALIB_FLASH_ERASE,
} CALIBRATOR_CMD;

/* firmware update command */
typedef enum {
	CHANGE_TO_BOOTLOADER_MODE = 1,
	CHANGE_TO_NORMAL_MODE = 5,
	CHECK_FIRMWAVE_VERSION,
	SET_DEBUG_LOG,
	SET_SYSTEM_COMMAND,
	SET_SENSORS_POSITION,
	SHOW_LOG_INFO,
	GET_FWPROJECT_ID,
	MCU_RESET,
	MCU_SET_GPIO_INPUT = 20,
	MCU_SET_GPIO_OUTPUT = 21,
	MCU_SET_GPIO_CONTROL = 22,
	MCU_SET_GPIO_CLEAN = 23,
	CMD_CALIBRATOR = 30,
	CMD_SELF_TEST = 31,
} FIRMWARE_CMD;

typedef enum {
	NonSystemComand = 0, WatchDogReset = 1, SystemNoSleep =
	    2, SystemCommandIndexEnd
} SYSTEM_COMMAND_INDEX;

typedef enum {
	//5Byte: 0:handle;1:id;2:Rate;[3:4]:timeout ms;
	RegMapW_SetEnable = 0x01,
	//5Byte: 0:handle;1:id;2:Rate;[3:4]:timeout ms;
	RegMapW_SetDisable = 0x02,
	//2Byte: 0:handle;1:id;
	RegMapW_SetFlush = 0x04,
	//16Byte: enable list;
	RegMapR_GetHostEnableList = 0x05,

    /*-- For Internal user used --*/
	//8Byte: enable list;
	RegMapR_GetInternalEnableList = 0x06,
	// The register of report the sensor/gesture data
	RegMapR_GetSensorData = 0x07,
	// Report the gesture data to Host(Android)
	RegMapW_ReportSensorToHost = 0x08,
	// Set and Get HW sensor setting
	RegMapW_SetHWSensorSetting = 0x09,

	// Send the enable request for customer algorithm
	RegMapR_GetCustomAlgoEnable = 0x0B,
	// Send the disable request for customer algorithm
	RegMapR_GetCustomAlgoDisable = 0x0C,
    /*----------------------------*/

	//4Byte
	RegMapR_ErrorCode = 0x0E,
	//2Byte(uint16_t)
	RegMapR_InterruptStatus = 0x0F,
    /**
    *   2byte
    *   byte 0: sensors id
    *   byte 1: position
    */
	RegMapW_SetSensorAxisReference = 0x1F,
    /**
    *   2byte
    *   count
    *   byte 0: count   L
    *   byte 1: count   H
    *
    *   9byte
    *   event
    *   byte 0: sensors id
    *   byte 1: sensors X   L
    *   byte 2: sensors X   H
    *   byte 3: sensors Y   L
    *   byte 4: sensors Y   H
    *   byte 5: sensors Z   L
    *   byte 6: sensors Z   H
    *   byte 7: timestamp   L
    *   byte 8: timestamp   H
    */
	RegMapR_StreamCount = 0x20,
	RegMapR_StreamEvent = 0x21,
	RegMapR_BatchCount = 0x22,
	RegMapR_BatchEvent = 0x23,
	/**
    *   4byte
    *   byte [0]:Hall0 Sensor Near[0] Far[1]
    *   byte [1]:Hall0 Sensor Status
    *   byte [2]:Hall1 Sensor Near[0] Far[1]
    *   byte [3]:Hall1 Sensor Status
    */
	RegMapR_HallStatus = 0x2C,

    /**
    *   1byte
    *   count
    *
    *   9byte
    *   byte 0: sensors id
    *   byte 1: Handle
    *   [byte 2:byte 8] data
    */
	RegMapR_GestureCount = 0x24,
	RegMapR_GestureEvent = 0x25,

    /**
    *   2byte
    *   count
    *
    *   30byte
    *   Log
    */
	RegMapR_SystemInfoMsgCount = 0x26,
	RegMapR_SystemInfoMsgEvent = 0x27,

    /**
    *   Warming message
    *   2byte
    *   count
    *
    *   30byte
    *   Log
    */
	RegMapR_WarningMsgCount = 0x28,
	RegMapR_WarningMsgEvent = 0x29,

    /**
    *   Log message
    *   2byte
    *   count
    *
    *   128
    *   Log messagebyte
    *   Log
    */
	RegMapR_LogMsgCount = 0x2A,
	RegMapR_LogMsgEvent = 0x2B,

	/**
    *   2byte
    *   byte 0: Sensors Id
    *   byte 1: Sensors Type
    *
    *   wbyte
    *   Calibrator Status
    *   byte 0: Sensors Id
    *   byte 1: Sensors Status
    */
	RegMapW_CalibratorCmd = 0x40,
	RegMapW_CalibratorData = 0x41,

	RegMapR_CalibratorData = 0x43,

    /**
    *   64byte
    */
	RegMapR_MagSpecialData = 0x44,

    /**
    *   1byte
    */
	RegMapW_SetSystemCommand = 0x5A,
	RegMapW_SetHostStatus = 0x5B,
    /**
    *   30byte
    */
	RegMapW_SetSystemMsg = 0x5C,
	RegMapR_GetSystemMsg = 0x5D,

	RegMapR_GetAccelerationRawData = 0x60,
	RegMapR_GetMagneticRawData = 0x61,
	RegMapR_GetGyroRawData = 0x62,
	RegMapR_GetLightRawData = 0x63,
	RegMapR_GetProximityRawData = 0x64,
	RegMapR_GetRawData5 = 0x65,
	RegMapR_GetRawData6 = 0x66,
	RegMapR_GetRawData7 = 0x67,
	RegMapR_GetRawData8 = 0x68,
	RegMapR_GetRawData9 = 0x69,
    /**
    *   4byte
    *   data[0] = BuildSubVersion;
    *   data[1] = BuildVersion;
    *   data[2] = BuildDay;
    *   data[3] = BuildMonth;
    */
	RegMapR_GetFWVersion = 0x80,
	RegMapR_GetSystemTimestamp = 0x81,
    /**
    *   64byte
    *   data
    */
	RegMapR_GetProjectID = 0x82,
	/*Project ID for function available check */
	RegMapR_GetCountOfRegID = 0x83,
	RegMapR_GetReadRegTableSetting = 0x84,
	RegMapR_GetWriteRegTableSetting = 0x85,
	RegMapW_SetUARTDebugList = 0x86,
	RegMapR_GetLibVersion = 0x87,

    /**
    *   2byte
    *   data
    *   0:0x43 1:0x57
    */
	RegMapR_ChipId = 0x88,

	/*
	   Adding by Jack Tsai 2016/2/2
	 */
	RegMapR_hrLog_Count = 0x8B,
	RegMapR_hrLog_Event = 0x8C,
    /*
        request time sync event from sensor hub 2017/10/17
    */
    RegMapR_ReqTimeSyncEvt = 0x92,
	/*Pass the regist table information to the
	   * host to detemine the read and write behavior
	 */
	CW_MAX_REG
} CwRegisterMapIndex;

typedef struct {
	uint8_t cIndex;
	uint8_t cObjLen;
} RegInformation;


struct CWMCU_SENSORS_INFO {
	uint8_t en;
	uint8_t mode;
	uint8_t rate;
	uint16_t timeout;
};
typedef struct {
	uint8_t hw_id;
	uint8_t deviceaddr;
	uint8_t rate;
	uint8_t mode;		//default:  MODE_BYPASS
	uint8_t position;
	uint8_t private_setting[4];	//private_setting[2] = INTERRUPT_SETTING
} SensorsInit_T;


#define CWMCU_NODATA	0xff

#define DPS_MAX			(1 << (16 - 1))

/* GPIO for MCU control */
#define QueueSystemInfoMsgSize  30
#define QueueSystemInfoMsgBuffSize      QueueSystemInfoMsgSize*5


struct CWMCU_T {
	struct i2c_client *client;
	struct spi_device *spi;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct workqueue_struct *driver_wq;
#ifdef CWM_USE_IRQ_WORK
	struct work_struct work;
#endif
#ifdef CWM_USE_DELAY_WORK
	struct delayed_work delay_work;
#endif
#ifdef CWM_USE_ERROR_HANDLE_WORK
	struct delayed_work error_handle_work;
#endif
#ifdef CWM_USE_TIME_SYNC_WORK
    struct delayed_work time_sync_work;
#endif
	struct work_struct resume_work;
	struct input_dev *input;
	struct CWMCU_SENSORS_INFO
	    sensors_info[HANDLE_ID_END][SENSORS_ID_END];
	SensorsInit_T hw_info[DRIVER_ID_END];
	RegInformation *pReadRegInfo;
	RegInformation *pWriteRegInfo;
	u8 m_cReadRegCount;
	u8 m_cWriteRegCount;

	int mcu_mode;
	uint8_t kernel_status;
	uint8_t meg_status[31];

	/* enable & batch list */
	uint32_t enabled_list[HANDLE_ID_END];
	uint32_t interrupt_status;
	uint8_t calibratordata[DRIVER_ID_END][30];
	uint8_t calibratorUpdate[DRIVER_ID_END];
	uint8_t CalibratorStatus[DRIVER_ID_END];
	uint8_t SelfTestStatus[DRIVER_ID_END];

	/* Mcu site enable list */

	/* power status */
	volatile uint32_t power_on_list;

	/* Calibrator status */
	int cal_cmd;
	int cal_type;
	int cal_id;

	/* gpio */
	int irq_gpio;
	int wakeup_gpio;
	int reset_gpio;
	int boot_gpio;

	uint32_t debug_log;

	int cmd;
	uint32_t addr;
	int len;
	int adjust_len;
	int mcu_slave_addr;
	int firmware_update_status;
	int cw_i2c_rw;		/* r = 0 , w = 1 */
	int cw_i2c_len;
	uint8_t cw_i2c_data[300];
#ifdef CWM_USE_ERROR_HANDLE_WORK
	int i2c_error_count;
	int i2c_error_state;
#endif

	s32 iio_data[6];
	struct iio_dev *indio_dev;
	struct irq_work iio_irq_work;
	struct iio_trigger *trig;
	atomic_t pseudo_irq_enable;

	struct class *sensor_class;
	struct device *sensor_dev;
	struct notifier_block cw_pm_notifier;
	atomic_t delay;

	int wq_polling_time;

	struct mutex mutex_lock;

	unsigned char loge_buff[QueueSystemInfoMsgSize * 2];
	unsigned char loge_buff_count;

	uint8_t logcat_cmd;

/*Data for GPS and Health Info*/
	uint8_t GPSData[40];
	uint8_t HEALTHData[30];
	uint8_t POSITIONData[3];

	int mcu_status;
	int mcu_init_count;
	int mcu_suspend_flag;
	int watch_orientation;
};


#define CW_INFO(fmt,arg...)		pr_info("-CWMCU_INF- "fmt"\n",##arg)
#define CW_ERROR(fmt,arg...)		pr_err("-CWMCU_ERR- "fmt"\n",##arg)
#define CW_DEBUG(fmt,arg...)		pr_debug("-CWMCU_DBG- "fmt"\n",##arg)


extern int CWMCU_bus_register(void);
extern void CWMCU_bus_unregister(void);
extern int CWMCU_bus_write_serial(struct CWMCU_T *sensor, u8 * data,
				  int len);
extern int CWMCU_bus_read_serial(struct CWMCU_T *sensor, u8 * data,
				 int len);
extern int CWMCU_bus_read(struct CWMCU_T *sensor, u8 reg_addr, u8 * data,
			  u8 len);
extern int CWMCU_bus_write(struct CWMCU_T *sensor, u8 reg_addr, u8 * data,
			   u8 len);
extern void CWMCU_bus_init(void *bus_dev);
extern int CWMCU_probe(struct i2c_client *client);
extern void CWMCU_shutdown(struct CWMCU_T *sensor);
extern void CWMCU_suspend(struct CWMCU_T *sensor);
extern void CWMCU_resume(struct CWMCU_T *sensor);
extern void factory_active_sensor(int sensor_id, int enabled);
extern int factory_data_read(int sensor_id, int data_event[]);
extern int factory_set_calibrator_data(int sensor_id,
				       SENSOR_DATA * cali_data);
extern void factory_clear_calibrator_data(struct CWMCU_T *sensor,
					  int sensor_id);
extern void factory_get_calibrator_data(int sensor_id,
					SENSOR_DATA * cali_data);
extern void factory_set_mcu_calibration_data(int sensor_id);
extern void init_factory_node(void);
extern int spi_rw_bytes_serial(u8 * wbuf, u8 * rbuf, u8 len);
extern int CWMCU_system_suspend(void);
extern void CWMCU_system_resume(void);











#ifdef __KERNEL__

#endif				/* __KERNEL */

#endif				/* __CWMCUSENSOR_H__ */
