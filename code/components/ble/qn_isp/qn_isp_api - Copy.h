/****************************************************************************************
 * @addtogroup QLIB_API Quintic Library API
 * @ingroup QLIB
 * @brief Quintic Library API
 *
 * The API communicates with QN902x Bootloader to implement downloading application
 * binary file and nvdsconfiguration file into QN902x. APIs obscures the details of
 * interactive operation with bootloader and provides a easy way to finish downloading feature.
 *
 ***************************************************************************************/

#ifndef __QLIB_H__
#define __QLIB_H__

#include "qn_isp_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// timeout
#define CMD_EXE_NORMAL_TIMEOUT                 1000  // ms
#define CMD_EXE_ERASE_TIMEOUT                  2000  // ms
#define CMD_EXE_CONNECT_TIMEOUT                10    // ms

// QN902x information
#define DEFAULT_QN902X_FLASH_PAGE_SIZE         (256)
#define DEFAULT_QN902X_FLASH_SECTOR_SIZE       (4*1024)
#define DEFAULT_QN902X_FLASH_START_ADDR        (0)
#define DEFAULT_QN902X_FLASH_TOTAL_SIZE        (128*1024)
#define DEFAULT_QN902X_FLASH_NVDS_START_ADDR   (0)
#define DEFAULT_QN902X_FLASH_NVDS_TOTAL_SIZE   (4*1024)
#define DEFAULT_QN902X_BOOTINFO_START_ADDR     (0x1000)
#define DEFAULT_QN902X_BOOTINFO_TOTAL_SIZE     (256)
#define DEFAULT_QN902X_FLASH_APP_START_ADDR    (0x1100)
#define DEFAULT_QN902X_RAM_START_ADDR          (0x10000000)
#define DEFAULT_QN902X_RAM_TOTAL_SIZE          (64*1024)
#define DEFAULT_QN902X_RAM_NVDS_START_ADDR     (0x1000B000)

// buffer information
#define POOL_BUFFER_SIZE                       (300)
#define MAX_RX_PAYLOAD_LEN                     (POOL_BUFFER_SIZE-1)
#define POOL_GUARD                             (0xCC)
#define SET_POOL_GUARD(payload)                (payload)->buffer[POOL_BUFFER_SIZE-1] = POOL_GUARD
#define IS_POOL_GUARD_HERE(payload)            ((payload)->buffer[POOL_BUFFER_SIZE-1] == POOL_GUARD)

// protocal information
#define RX_PAYLOAD_DATA_FRONT_LEN              (5)
#define RX_PAYLOAD_DATA_HEAD_LEN               (1)
#define RX_PAYLOAD_DATA_LEN_LOW_POS            (2)
#define RX_PAYLOAD_DATA_LEN_MID_POS            (3)
#define RX_PAYLOAD_DATA_LEN_HIGH_POS           (4)
#define TX_PAYLOAD_DATA_FRONT_LEN              (5)
#define TX_PAYLOAD_DATA_HEAD_LEN               (1)
#define TX_PAYLOAD_DATA_LEN_LOW_POS            (2)
#define TX_PAYLOAD_DATA_LEN_MID_POS            (3)
#define TX_PAYLOAD_DATA_LEN_HIGH_POS           (4)
#define CRC16_LEN                              (2)

// Endian transmition
#define SET_UINT16(addr, x)                  do{((uint8_t *)(addr))[0] = (x) >> 0;\
                                                ((uint8_t *)(addr))[1] = (x) >> 8;\
                                             }while(0)
#define SET_UINT32(addr, x)                  do{((uint8_t *)(addr))[0] = (x) >> 0; \
                                                ((uint8_t *)(addr))[1] = (x) >> 8; \
                                                ((uint8_t *)(addr))[2] = (x) >> 16;\
                                                ((uint8_t *)(addr))[3] = (x) >> 24;\
                                             }while(0)
#define GET_UINT16(addr)                     (((uint16_t)(((uint8_t *)(addr))[0]) << 0) |\
                                              ((uint16_t)(((uint8_t *)(addr))[1]) << 8)  \
                                             )
#define GET_UINT32(addr)                     (((uint16_t)(((uint8_t *)(addr))[0]) << 0)  |\
                                              ((uint16_t)(((uint8_t *)(addr))[1]) << 8)  |\
                                              ((uint16_t)(((uint8_t *)(addr))[2]) << 16) |\
                                              ((uint16_t)(((uint8_t *)(addr))[3]) << 24)  \
                                             )
#define le_to_cpu_32bit(addr)                GET_UINT32(addr)

// misc
#define UART_BAUD_NUM                        16
#define PAGE_SIZE(dev_index)                 (qlib_env[dev_index].dev.info.flash.page_size)
#define IS_IN_SAME_SECTOR(dev_index, a, b)   ((a & ~(qlib_env[dev_index].dev.info.flash.sector_size - 1))\
                                             == (b & ~(qlib_env[dev_index].dev.info.flash.sector_size - 1)))
#define ERRNO                                qlib_env[dev_index].__errno
#define NVDS_ERRNO(status)                   ((qerrno_t)((status) - NVDS_FAIL + QN_NVDS_FAIL))
#ifndef MIN
#define MIN(a, b)                            ((a) < (b) ? (a) : (b))
#endif

// nvds
#define CO_ALIGN4_HI(val)                    (((val)+3)&~3)

#define FLASH_TYPE_UNKNOWN                   0

/// NVDS is defined as read-write
#define NVDS_READ_WRITE                      1

/// NVDS is defined as packed
#define NVDS_PACKED                          0

/// NVDS has 8-bit length tags
#define NVDS_8BIT_TAGLENGTH                  0

#define NVDS_FLASH_ADDRESS                   (0x00000000)

#define NVDS_FLASH_SIZE                      (0x00001000)

/// NVDS parameter data maximum length
//#define NVDS_PARAMETER_MAX_LENGTH   NVDS_LEN_BASIC_THRESHOLD
#define NVDS_PARAMETER_MAX_LENGTH            128

/// TAG STATUS bit assignment
#define NVDS_STATUS_VALID_MASK               0x01
#define NVDS_STATUS_VALID                    0x00
#define NVDS_STATUS_NOT_VALID                0x01
#define NVDS_STATUS_LOCKED_MASK              0x02
#define NVDS_STATUS_LOCKED                   0x00
#define NVDS_STATUS_NOT_LOCKED               0x02
#define NVDS_STATUS_ERASED_MASK              0x04
#define NVDS_STATUS_ERASED                   0x00
#define NVDS_STATUS_NOT_ERASED               0x04

// NVDS Mapping
/// Size of magic number
#define NVDS_MAGIC_NUMBER_LENGTH                     4
/// Magic number offset
#define NVDS_MAGIC_NUMBER_ADDRESS(dev_index)         ((uint32_t)qlib_env[dev_index].nvds.data)
#define NVDS_STORAGE_AREA_SIZE(dev_index)            (          qlib_env[dev_index].nvds.length)
#define NVDS_TEMP_BUF(dev_index)                     (          qlib_env[dev_index].nvds.temp_buf)
/// Start of NVDS data
#define NVDS_START_STORAGE_AREA_ADDRESS(dev_index)   CO_ALIGN4_HI(NVDS_MAGIC_NUMBER_ADDRESS(dev_index)) + \
                                                     CO_ALIGN4_HI(NVDS_MAGIC_NUMBER_LENGTH)

/// Check is tag is the last one
#define NVDS_IS_TAG_LAST(h)        ((h).tag == NVDS_END_MARKER_TAG)
/// Check is tag is valid
#define NVDS_IS_TAG_OK(h)          ((((h).status) & (NVDS_STATUS_VALID_MASK|NVDS_STATUS_ERASED_MASK)) == \
                                    (NVDS_STATUS_VALID|NVDS_STATUS_NOT_ERASED))
/// Check is tag is locked
#define NVDS_IS_TAG_LOCKED(h)      ((((h).status) & NVDS_STATUS_LOCKED_MASK) == NVDS_STATUS_LOCKED)
/// Set tag as erased
#define NVDS_SET_TAG_ERASED(h)     ((((h).status) & (~NVDS_STATUS_ERASED_MASK)) | NVDS_STATUS_ERASED)
/// Set tag as locked
#define NVDS_SET_TAG_LOCKED(h)     ((((h).status) & (~NVDS_STATUS_LOCKED_MASK)) | NVDS_STATUS_LOCKED)
/// Set tag as valid
#define NVDS_SET_TAG_OK(h)         (NVDS_STATUS_VALID | NVDS_STATUS_NOT_LOCKED | NVDS_STATUS_NOT_ERASED)

/// Macro for alignment
#if (NVDS_PACKED == 1)
#define NVDS_ALIGNMENT(p) (p)
#else //(NVDS_PACKED == 0)
#define NVDS_ALIGNMENT(p) CO_ALIGN4_HI(p)
#endif //(NVDS_PACKED == 1)

/// Length of tag header
#define NVDS_TAG_HEADER_LENGTH     NVDS_ALIGNMENT(sizeof(struct nvds_tag_header))
/// Length of tag data
#define NVDS_TAG_CONTENT_LENGTH(h) NVDS_ALIGNMENT((h).length)
/// Full length of tag (header+data)
#define NVDS_TAG_FULL_LENGTH(h)    NVDS_TAG_HEADER_LENGTH + NVDS_TAG_CONTENT_LENGTH(h)

#if (NVDS_READ_WRITE == 1)
/// Max storage for the NVDS device which can be used for tags
#define NVDS_MAX_STORAGE_SIZE                (4*1024)
#endif //(NVDS_READ_WRITE == 1)


/*
 * Enum DEFINITIONS
 ****************************************************************************************
 */

/// Device operation error type
typedef enum
{
    // No error
    QE_OK,                       /// Operation success

    // General error
    QE_USER_ABORT,               /// User force abort current operation
    QE_TIMEOUT,                  /// Timer out
    QE_COMMUNICATION_FAIL,       /// Communication fail
    QE_FLASH_ERASE_FAIL,         /// Erase flash fail

    // Error when open device
    QE_DEVCIE_INDEX_OVERSIZE,    /// device index oversize, the max index is (QLIB_MAX_DEVICE_NUMBER - 1)
    QE_DEVCIE_IO_OPEN_FAIL,      /// Open device io fail
    QE_DEVCIE_IO_CLOSE_FAIL,     /// Close device io fail
    QE_DEVCIE_IO_WRITE_FAIL,     /// Write device io fail
    QE_DEVCIE_IO_RESET_FAIL,     /// Reset device io fail

    // Error when download application
    QE_APP_SIZE_OVERSIZE,        /// App size oversize
    QE_APP_FLASH_ADDRESS_INVALID,/// App flash address is invalid
    QE_APP_DOWNLOAD_FAIL,        /// Download application fail
    QE_APP_PROTECT_FAIL,         /// Protect application fail
    QE_APP_VERIFY_FAIL,          /// Verify application fail

    // Error when download NVDS
    QE_NVDS_INVALID,             /// NVDS is invalid
    QE_NVDS_DOWNLOAD_FAIL,       /// Download NVDS fail
    QE_NVDS_VERIFY_FAIL,         /// Verify NVDS fail

    // Error when close device
    QE_RUN_APP_FAIL,             /// Run application from bootload fail

    // Error when NVDS operation
    QN_NVDS_FAIL,                /// generic NVDS status
    QN_NVDS_TAG_NOT_DEFINED,     /// NVDS TAG unrecognized
    QN_NVDS_NO_SPACE_AVAILABLE,  /// No space for NVDS
    QN_NVDS_LENGTH_OUT_OF_RANGE, /// Length violation
    QN_NVDS_PARAM_LOCKED,        /// NVDS parameter locked
    QN_NVDS_CORRUPT,             /// NVDS corrupted
} qerrno_t;

/// Device type
typedef enum
{
    ISP_DEVICE_QN9020,
    ISP_DEVICE_QN9021,
} device_type_t;

/// device crystal type
typedef enum
{
    ISP_CRYSTAL_16MHZ,
    ISP_CRYSTAL_32MHZ,
} crystal_type_t;

/// device application location type
typedef enum
{
    ISP_LOCATION_RAM,
    ISP_LOCATION_FLASH,
} location_t;

/// device flash clock type
typedef enum
{
    ISP_FLASH_CLOCK_16MHZ  = 16000000,
    ISP_FLASH_CLOCK_8MHZ   = 8000000,
    ISP_FLASH_CLOCK_4MHZ   = 4000000,
    ISP_FLASH_CLOCK_2MHZ   = 2000000,
    ISP_FLASH_CLOCK_1MHZ   = 1000000,
    ISP_FLASH_CLOCK_500KHZ = 500000,
    ISP_FLASH_CLOCK_250KHZ = 250000,
    ISP_FLASH_CLOCK_125KHZ = 125000,
} flash_clock_t;

/// List of NVDS TAG identifiers
enum NVDS_TAG
{
    /// Definition of the tag associated to each parameters
    /// Local Bd Address
    NVDS_TAG_BD_ADDRESS                 = 0x01,
    /// Device Name
    NVDS_TAG_DEVICE_NAME                = 0x02,
    /// Radio Drift
    NVDS_TAG_LPCLK_DRIFT                = 0x03,
    /// factory setting
    NVDS_TAG_FACTORY_SETTING_0          = 0x04,
    /// Oscillator wake-up time
    NVDS_TAG_OSC_WAKEUP_TIME            = 0x05,
    /// Radio wake-up time
    NVDS_TAG_RM_WAKEUP_TIME             = 0x06,
    /// Enable sleep mode
    NVDS_TAG_SLEEP_ENABLE               = 0x07,
    /// factory setting
    NVDS_TAG_FACTORY_SETTING_1          = 0x08,
    NVDS_TAG_FACTORY_SETTING_2          = 0x09,
    NVDS_TAG_FACTORY_SETTING_3          = 0x0a,
    /// TK type
    NVDS_TAG_TK_TYPE                    = 0x0b,
    /// TK
    NVDS_TAG_TK_KEY                     = 0x0c,
    /// IRK
    NVDS_TAG_IRK_KEY                    = 0x0d,
    /// CSRK
    NVDS_TAG_CSRK_KEY                   = 0x0e,
    /// LTK
    NVDS_TAG_LTK_KEY                    = 0x0f,
    /// crystal oscillator cap loading selection
    NVDS_TAG_XCSEL                      = 0x10,
    /// temperature offset
    NVDS_TAG_TEMPERATURE_OFFSET         = 0x11,
    /// adc internal reference scale
    NVDS_TAG_ADC_INT_REF_SCALE          = 0x12,
    /// adc internal reference vcm
    NVDS_TAG_ADC_INT_REF_VCM            = 0x13,

    /// this is the TAG used to be the marker of the last TAG in NVDS (= 0xFF because when
    /// FLASH are erased, they are set to = 0xFF)
    NVDS_END_MARKER_TAG                 = 0xFF,
};

/// List of NVDS Tag lengths
enum NVDS_LEN
{
    // Definition of length associated to each parameters
    /// Local Bd Address
    NVDS_LEN_BD_ADDRESS                 = 6,
    /// Device Name
    NVDS_LEN_DEVICE_NAME                = 32,
    /// Low power clock drift
    NVDS_LEN_LPCLK_DRIFT                = 2,
    /// Factory setting
    NVDS_LEN_FACTORY_SETTING_0          = 2,
    /// Oscillator wake-up time
    NVDS_LEN_OSC_WAKEUP_TIME            = 2,
    /// Radio wake-up time
    NVDS_LEN_RM_WAKEUP_TIME             = 2,
    /// Enable sleep mode
    NVDS_LEN_SLEEP_ENABLE               = 1,
    /// Factory setting
    NVDS_LEN_FACTORY_SETTING_1          = 1,
    NVDS_LEN_FACTORY_SETTING_2          = 4,
    NVDS_LEN_FACTORY_SETTING_3          = 4,
    /// TK type
    NVDS_LEN_TK_TYPE                    = 1,
    /// TK
    NVDS_LEN_TK_KEY                     = 16,
    /// IRK
    NVDS_LEN_IRK_KEY                    = 16,
    /// CSRK
    NVDS_LEN_CSRK_KEY                   = 16,
    /// LTK
    NVDS_LEN_LTK_KEY                    = 16,
    /// crystal oscillator cap loading selection
    NVDS_LEN_XCSEL                      = 1,
    /// temperature offset
    NVDS_LEN_TEMPERATURE_OFFSET         = 4,
    /// adc internal reference scale
    NVDS_LEN_ADC_INT_REF_SCALE          = 4,
    /// adc internal reference vcm
    NVDS_LEN_ADC_INT_REF_VCM            = 4,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef uint16_t nvds_tag_len_t;           // Type of the tag length

typedef struct
{
    uint8_t integer_H;
    uint8_t integer_L;
    uint8_t fractional;
} baudrate_reg_t;

typedef struct                             // device flash information struct
{
    size_t page_size;
    size_t sector_size;
    uint32_t start_address;
    size_t total_size;
} flash_info_t;

typedef struct                             // device NVDS information struct
{
    uint32_t start_address;
    size_t total_size;
} nvds_info_t;

typedef struct                             // device application information struct
{
    uint32_t start_address;
    size_t total_size;
} app_info_t;

typedef struct                             // device bootloader information struct
{
    uint32_t boot_information_start_address;
    size_t boot_information_total_size;
} boot_info_t;

typedef struct                             // device ram information struct
{
    uint32_t start_address;
    size_t total_size;
} ram_info_t;

typedef struct                             // device information struct
{
    flash_info_t flash;
    nvds_info_t  nvds;
    app_info_t   app;
    boot_info_t  boot;
    ram_info_t   ram;
} device_info_t;

typedef struct
{
    size_t         length;
    uint8_t        data[NVDS_MAX_STORAGE_SIZE];
    uint8_t        temp_buf[NVDS_MAX_STORAGE_SIZE];
} nvds_t;

typedef struct                             // Memory pool, current use for payload alloc.
{
    uint8_t  buffer[POOL_BUFFER_SIZE];
    uint8_t *pdata;
    uint16_t len;
#ifdef QLIB_DEBUG
    uint16_t state;
#endif
} pool_t;

typedef struct                             // Device information
{                                          /* re-defined setting (option) */
    flash_clock_t  flash_clk;              // Flash clock
    int            app_protect;            // Whether to protect application. default is true
    int            app_verify;             // Whether to verify application when download it. default is true
    int            nvds_verify;            // Whether to verify nvds when download it. default is true
    location_t     app_location;           // Where to download application. default is flash.
                                           // If app is in RAM, its address is fix.
    location_t     nvds_location;          // Where to download NVDS. default is flash and its address is fixed.
                                           // If nvds is in ram, the application must be in ram.
    uint32_t       app_in_flash_address;   // address to download to flash, it must be integer multiples of FLASH_PAGE(256)
    uint32_t       nvds_in_ram_address;    // address to download to ram
    device_type_t  device;                 // Device type
    crystal_type_t crystal;                // Device crystal
    device_info_t  info;                   // Device information
    uint32_t       app_in_ram_address;     // The address to download app to ram. If app is in RAM, its address is fix
    uint32_t       nvds_in_flash_address;  // The address to download NVDS to flash, If NVDS is in flash, its address is fix
    int            nvds_valid;             // Whether is nvds data valid
} device_t;

typedef struct                         // qlib_env struct
{                                      // the device information must be set this struct first place
    device_t     dev;
    nvds_t       nvds;
    pool_t       pool;
    int          index;
    int          opened;
    int          nvds_should_purge;
    volatile int abort;
    qerrno_t     __errno;              // last error number
} qlib_t;


/**
 ****************************************************************************************
 * @brief Read data type
 *
 * @param[in]  offset     Read data offset
 * @param[in]  len        Read data length from offset
 * @param[out] pdata      Readed data buffer from (offset) to (offset + length)
 *
 * @return     length     The actual readed length
 ****************************************************************************************
 */
typedef int (*read_func_t)(uint32_t offset, size_t len, uint8_t *pdata);


/// @endcond

/// Device object structure
typedef struct
{
    /* re-defined setting (option) */

    /// Inside spi flash clock.
    flash_clock_t        flash_clk;
    /// The switch of protecting application. Default is true.
    int                 app_protect;

    /// The switch of verifying application when download it. Default is true.
    int                 app_verify;
    /// The switch of verifying NVDS when download it. Default is true.
    int                 nvds_verify;

    /// The download location of application. Default is flash. If application is in ram, its address is fixed.
    location_t           app_location;
    /// The download location of NVDS. Default is flash. If NVDS is in flash, its address is fixed.
    /// NOTE: If NVDS is in ram, the application must be in ram.
    location_t           nvds_location;
    /// The address to download to flash, it must be integer multiples of one page(256)
    uint32_t             app_in_flash_address;
    /// The address to download to ram
    uint32_t             nvds_in_ram_address;

    /* const information */
    /// Device type
    const device_type_t  device;
    /// Device cristal
    const crystal_type_t crystal;
    /// Device information
    const device_info_t  info;
    /// The address to download app to ram. If app is in RAM, its address is fix
    const uint32_t       app_in_ram_address;
    /// The address to download NVDS to flash, If NVDS is in flash, its address is fix
    const uint32_t       nvds_in_flash_address;
    /// Whether is nvds data valid
    const int           nvds_valid;
}qdevice_t;

/******************************************************************************************
 * @brief      Open a device.
 *             This function will try to connect to a deivce, the device is identified by
 *             "dev_index".
 *
 * @param[in]  dev_index     Device index, its range is from 0 to (QLIB_MAX_DEVICE_NUMBER - 1)
 * @param[in]  device_type   Device type
 * @param[in]  crystal       Device crystal
 * @param[in]  speed         Communication speed.
 *                           If the interface is UART, the speed is baudrate and limited by following:
 *                           16MHz crystal: 2400,4800,9600,19200,28800,38400,57600,76800,115200,
 *                                          128000,153600,230400,256000,460800,691200,921600
 *                           32MHz crystal: 4800,9600,19200,38400,57600,76800,115200,153600,230400,
 *                                          256000,307200,460800,512000,921600,1382400,1843200
 * @param[in]  timeout       open device timeout, it unit is ms
 *
 * @return     NULL:         Open device fail. Call qn_api_errno() to get the detail error information
 * @return     Other:        The device object. There are some setting and device infomation
 *                           in this object
 *****************************************************************************************/
qdevice_t *qn_api_open(int dev_index, device_type_t device_type, crystal_type_t crystal, uint32_t speed, uint32_t timeout);

/******************************************************************************************
 * @brief Download application to device
 *
 * @param[in]  dev           Device object. Get it from qn_api_open()
 * @param[in]  app_size      Download application size
 * @param[in]  read_app      Read application content method
 *
 * @return     Fail / Success. If fail, call qn_api_errno() to get the detail error information
 *****************************************************************************************/
int qn_api_download_app(qdevice_t *dev, size_t app_size, read_func_t read_app);

/******************************************************************************************
 * @brief Download NVDS to device
 *
 * @param[in]  dev           Device object. Get it from qn_api_open()
 *
 * @return     Fail / Success. If fail, call qn_api_errno() to get the detail error information
 *****************************************************************************************/
int qn_api_download_nvds(qdevice_t *dev);

/******************************************************************************************
 * @brief This function adds a specific TAG to the NVDS.
 *
 * @param[in]  dev        Device object. Get it from qn_api_open()
 * @param[in]  tag        TAG to look for whose DATA is to be retrieved
 * @param[in]  length     Expected length of the TAG
 * @param[in]  buf        Pointer to the buffer containing the DATA part of the TAG to add to
 *                        the NVDS
 *
 * @return     Fail / Success. If fail, call qn_api_errno() to get the detail error information
 *****************************************************************************************/
int qn_api_nvds_put(qdevice_t *dev, uint8_t tag, nvds_tag_len_t length, uint8_t *buf);

/******************************************************************************************
 * Look for a specific tag and return, if found and matching (in length), the DATA part of the TAG.
 *
 * If the length does not match, the TAG header structure is still filled, in order for
 * the caller to be able to check the actual length of the TAG.
 *
 * @param[in]       dev        Device object. Get it from qn_api_open()
 * @param[in]       tag        TAG to look for whose DATA is to be retrieved
 * @param[in][out]  lengthPtr  Expected length of the TAG
 * @param[out]      buf        A pointer to the buffer allocated by the caller to be filled with
 *                             the DATA part of the TAG
 *
 * @return     Fail / Success. If fail, call qn_api_errno() to get the detail error information
 *****************************************************************************************/
int qn_api_nvds_get(qdevice_t *dev, uint8_t tag, nvds_tag_len_t * lengthPtr, uint8_t *buf);

/******************************************************************************************
 * @brief Look for a specific tag and delete it (Status set to invalid)
 *
 * @param[in]  dev        Device object. Get it from qn_api_open()
 * @param[in]  tag        TAG to mark as deleted
 *
 * @return     Fail / Success. If fail, call qn_api_errno() to get the detail error information
 *****************************************************************************************/
int qn_api_nvds_del(qdevice_t *dev, uint8_t tag);

/******************************************************************************************
 * @brief Close the opened device
 *
 * @param[in]  dev           Device object. Get it from qn_api_open()
 *
 * @return     None.
 *****************************************************************************************/
void qn_api_close(qdevice_t *dev);

/******************************************************************************************
 * @brief Get last error No.
 *
 * @param[in]  dev_index     Device Index
 *
 * @return     Error information
 *****************************************************************************************/
qerrno_t qn_api_errno(int dev_index);

#ifdef __cplusplus
}
#endif

#endif

