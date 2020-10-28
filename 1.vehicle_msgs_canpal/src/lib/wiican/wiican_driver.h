#ifndef WIICAN_DRIVER_H
#define WIICAN_DRIVER_H

//v2.0
//Eric Ng. 2018-2-26

#ifndef DLL_CALL
    #define DLL_CALL
    #if defined(__linux) || defined(linux)
        #define LIBAPI
    #else
        #ifdef WIN32
        #define LIBAPI __stdcall
        #endif
    #endif
#endif //DLL_CALL

//begin of the data type definition section.
#define CAN_MODE_NORMAL                         0
#define CAN_MODE_LOOPBACK                       1
#define CAN_MODE_SILENT                         2

#define DEVICE_TYPE_A_C                         1
#define DEVICE_TYPE_A_CW                        5
#define DEVICE_TYPE_A_CE                        7
#define DEVICE_TYPE_A_CC                        11
#define DEVICE_TYPE_A_C_S                       12
#define DEVICE_TYPE_CP_COM                      17
#define DEVICE_TYPE_CP_AUTO                     18
#define DEVICE_TYPE_CP_PRO                      19
#define DEVICE_TYPE_A_CCW                       21

#define ERROR_NONE                              0
#define ERROR_LINK_FAILURE                      1

#define IOCTL_ERR_NONE                          0
#define IOCTL_ERR_PARAMETER_ERROR               1
#define IOCTL_ERR_NOT_SUPPORT                   2
#define IOCTL_ERR_BUFFER_TOO_SHORT              3
#define IOCTL_ERR_DEVICE_NOT_READY              4
#define IOCTL_ERR_ACK_FAILURE                   5
#define IOCTL_ERR_ACK_TIMEOUT                   6
#define IOCTL_ERR_PENDING_RESULT                7

#define FILTER_ANALYZER_MODE                    1

#define FILTER_CHANNEL_CAN1                     0
#define FILTER_CHANNEL_CAN1ExtCAN2              1

#define FILTER_TYPE_MASK                        0
#define FILTER_TYPE_RANGE                       1

#define CAN_PARA_CHANNEL_CAN1                   0
#define CAN_PARA_CHANNEL_CAN2                   1

#define CAN_PARA_RUN_MODE_ANALYZER              1

#define COMM_PARA_RUN_MODE_ANALYZER             0

#define CAN_CHANNEL_CAN1                        0x01
#define CAN_CHANNEL_CAN2                        0x02
#define CAN_CHANNEL_MCU                         0x04
#define CAN_CHANNEL_NET                         0x08

#define CHANNEL_DISABLE                         0
#define CHANNEL_ENABLE                          1
#define CHANNEL_RESET                           2
#define CHANNEL_INQUIRY                         3

#define SAVE_ANALYZER_MODE_SETTING              0

#define CAN_TX_BUFFER_MAX_LEN                   10000
#define CAN_RX_BUFFER_MAX_LEN                   1000000

#define CAN_SJW_1tq                 0x00        /*!< 1 time quantum */
#define CAN_SJW_2tq                 0x01        /*!< 2 time quantum */
#define CAN_SJW_3tq                 0x02        /*!< 3 time quantum */
#define CAN_SJW_4tq                 0x03        /*!< 4 time quantum */

#define CAN_BS1_1tq                 0x00        /*!< 1 time quantum */
#define CAN_BS1_2tq                 0x01        /*!< 2 time quantum */
#define CAN_BS1_3tq                 0x02        /*!< 3 time quantum */
#define CAN_BS1_4tq                 0x03        /*!< 4 time quantum */
#define CAN_BS1_5tq                 0x04        /*!< 5 time quantum */
#define CAN_BS1_6tq                 0x05        /*!< 6 time quantum */
#define CAN_BS1_7tq                 0x06        /*!< 7 time quantum */
#define CAN_BS1_8tq                 0x07        /*!< 8 time quantum */
#define CAN_BS1_9tq                 0x08        /*!< 9 time quantum */
#define CAN_BS1_10tq                0x09        /*!< 10 time quantum */
#define CAN_BS1_11tq                0x0A        /*!< 11 time quantum */
#define CAN_BS1_12tq                0x0B        /*!< 12 time quantum */
#define CAN_BS1_13tq                0x0C        /*!< 13 time quantum */
#define CAN_BS1_14tq                0x0D        /*!< 14 time quantum */
#define CAN_BS1_15tq                0x0E        /*!< 15 time quantum */
#define CAN_BS1_16tq                0x0F        /*!< 16 time quantum */

#define CAN_BS2_1tq                 0x00        /*!< 1 time quantum */
#define CAN_BS2_2tq                 0x01        /*!< 2 time quantum */
#define CAN_BS2_3tq                 0x02        /*!< 3 time quantum */
#define CAN_BS2_4tq                 0x03        /*!< 4 time quantum */
#define CAN_BS2_5tq                 0x04        /*!< 5 time quantum */
#define CAN_BS2_6tq                 0x05        /*!< 6 time quantum */
#define CAN_BS2_7tq                 0x06        /*!< 7 time quantum */
#define CAN_BS2_8tq                 0x07        /*!< 8 time quantum */

#define CAN_SJW_DEFAULT             CAN_SJW_1tq
#define CAN_BS1_DEFAULT             CAN_BS1_13tq
#define CAN_BS2_DEFAULT             CAN_BS2_4tq

#define CAN_HANDLE                  void*

typedef enum
{
    COMM_TYPE_TCP_CLIENT            = 0,
    COMM_TYPE_UDP_CLIENT            = 2,
    COMM_TYPE_USBCOM                = 3
} tCOMM_TYPE;

typedef enum
{
    IOCTL_GET_RX_MSG_NUM            = 0,
    IOCTL_CLR_RX_BUFFER             = 1,
    IOCTL_GET_DEVICE_STATUS         = 2,
    IOCTL_GET_CAN_PARA_A            = 3,
    IOCTL_SET_CAN_PARA_A            = 4,
    IOCTL_GET_DEVICE_INFO_A         = 5,
    IOCTL_GET_FILTER_A              = 6,
    IOCTL_SET_FILTER_A              = 7,
    IOCTL_SET_CHANNEL_A             = 8,
    IOCTL_SAVE_SETTING_A            = 9,
    IOCTL_VERIFY_PASSWORD_A         = 10,
    IOCTL_CHANGE_PASSWORD_A         = 11,
    IOCTL_SET_WIN_READ_EVENT        = 17,
    IOCTL_WRITE_USER_DATA_A         = 18,
    IOCTL_READ_USER_DATA_A          = 19
} tIOCTL_ACT;

typedef struct
{
    // channel:
    // bit0=From/To CAN1,
    // bit1=From/To CAN2,
    // bit2=From/To MCU,
    // bit3=Reserved.
    unsigned char   channel;
    bool            ext;
    bool            rtr;
    unsigned int    can_id;     //SID[0:10]/ EID[0:28]
    unsigned char   len;
    unsigned char   data[8];
    unsigned int    timestamp;
} tCAN_MESSAGE;

typedef struct
{
   bool             CAN1_enabled;
   bool             CAN1_init_failed;
   bool             CAN1_rx_overflow;
   bool             CAN1_bus_off;
   bool             CAN1_error_passive;
   bool             CAN1_error_warning;
   unsigned char    CAN1_tx_error_count;
   unsigned char    CAN1_rx_error_count;
   bool             CAN2_enabled;
   bool             CAN2_init_failed;
   bool             CAN2_rx_overflow;
   bool             CAN2_bus_off;
   bool             CAN2_error_passive;
   bool             CAN2_error_warning;
   unsigned char    CAN2_tx_error_count;
   unsigned char    CAN2_rx_error_count;
   bool             NET_online;
   bool             CAN_message_lost;
   unsigned int     timestamp;
} tDEVICE_STATUS;

typedef struct
{
    char            name[20];
    unsigned int    type;
    unsigned short  hw_ver;
    unsigned short  sw_ver;
    unsigned char   unique_id[12];
} tDEVICE_INFO;

typedef struct
{
    unsigned char   applied_run_mode;          //Bridge mode(0); Analyzer mode(1).
    unsigned char   applied_CAN_channel;       //CAN1(0); CAN1Ext/CAN2(1)
    unsigned char   filter_no;                 //0-15;
    unsigned int    acc;                       //[0:10] = [S_ID]; [11-28]=[E_ID]; [29]=Type(0:Mask/1:Range); [30]=RTR; [31]=EXT;
    unsigned int    mask;
    bool            deactivate;                //Activate(false); Deactivate(true)
} tCAN_FILTER;

typedef struct
{
   unsigned char    applied_CAN_channel;       //0-CAN1; 1-CAN2; 2-NET(Wifi/Ethernet)
   unsigned char    applied_run_mode;          //Bridge Mode(0); Diagnostic Mode(1)
   unsigned char    mode;
   unsigned int     baud_rate;
   unsigned char    SJW;
   unsigned char    BS1;
   unsigned char    BS2;
} tCAN_PARAMETER;

typedef struct
{
    tCOMM_TYPE      comm_type;
    char            dest_host[256];
    unsigned short  dest_port;
    char            local_host[256];
    unsigned short  local_port;
    unsigned char   run_mode;
} tCOMM_PARAMETER;

typedef struct
{
    unsigned char   CAN1_channel;
    unsigned char   CAN2_channel;
} tCAN_CHANNEL_CONTROL;

typedef struct
{
    unsigned char index;
    unsigned char data[32];
} tUSER_DATA;

typedef struct
{
    tCOMM_PARAMETER		comm_para;
    tDEVICE_INFO		dev_info;
} tCAN_SEARCH;

typedef void (*wiican_notify_func)(void* which, int nEvent);
typedef CAN_HANDLE (LIBAPI* wiican_open_proc)();
typedef void (LIBAPI* wiican_close_proc)(CAN_HANDLE handle);
typedef bool (LIBAPI* wiican_connect_proc)(CAN_HANDLE handle, tCOMM_PARAMETER* comm_para);
typedef bool (LIBAPI* wiican_connect_ex_proc)(CAN_HANDLE handle, tCOMM_PARAMETER* comm_para, wiican_notify_func notify_func);
typedef bool (LIBAPI* wiican_connected_proc)(CAN_HANDLE handle);
typedef unsigned int (LIBAPI* wiican_send_proc)(CAN_HANDLE handle, const tCAN_MESSAGE* m, unsigned int len);
typedef unsigned int (LIBAPI* wiican_receive_proc)(CAN_HANDLE handle, tCAN_MESSAGE* m, unsigned int len, unsigned int timeout, unsigned char channel);
typedef unsigned int (LIBAPI* wiican_ioctl_proc)(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
typedef unsigned int (LIBAPI* wiican_ioctl_non_blocking_proc)(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
typedef unsigned int (LIBAPI* wiican_ioctl_pending_result_proc)(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
typedef unsigned int (LIBAPI* wiican_error_proc)(CAN_HANDLE handle);
typedef bool (LIBAPI* wiican_search_proc)(tCOMM_PARAMETER* in_out);
typedef bool (LIBAPI* wiican_search_detail_proc)(tCAN_SEARCH* in_out, unsigned int* device_count);

//end for the data type definition section.

//begin of the function definition section.
extern "C"
{
//begin of the basic APIs
    CAN_HANDLE LIBAPI   wiican_open();
    void LIBAPI         wiican_close(CAN_HANDLE handle);
    bool LIBAPI         wiican_connect(CAN_HANDLE handle, tCOMM_PARAMETER* comm_para);
    bool LIBAPI         wiican_connect_ex(CAN_HANDLE handle, tCOMM_PARAMETER* comm_para, wiican_notify_func notify_func);
    unsigned int LIBAPI wiican_send(CAN_HANDLE handle, const tCAN_MESSAGE m[], unsigned int len);
    unsigned int LIBAPI wiican_receive(CAN_HANDLE handle, tCAN_MESSAGE m[], unsigned int len, unsigned int timeout, unsigned char channel);
    unsigned int LIBAPI wiican_ioctl(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_non_blocking(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_pending_result(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    bool LIBAPI         wiican_connected(CAN_HANDLE handle);
    unsigned int LIBAPI wiican_error(CAN_HANDLE handle);
    bool LIBAPI         wiican_search(tCOMM_PARAMETER* in_out);
    bool LIBAPI         wiican_search_detail(tCAN_SEARCH* in_out, unsigned int* device_count);

//end of the basic APIs

//begin of the java JNA wrapper for IOCTL function.
    unsigned int LIBAPI wiican_ioctl_8(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_32(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_64(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_tDEVICE_STATUS(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_tCAN_PARAMETER(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_tDEVICE_INFO(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_tCAN_FILTER(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_tCAN_CHANNEL_CONTROL(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_Byte_Array(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);

    unsigned int LIBAPI wiican_ioctl_non_blocking_8(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_non_blocking_32(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_non_blocking_64(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_non_blocking_tDEVICE_STATUS(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_non_blocking_tCAN_PARAMETER(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_non_blocking_tDEVICE_INFO(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_non_blocking_tCAN_FILTER(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_non_blocking_tCAN_CHANNEL_CONTROL(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_non_blocking_Byte_Array(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);

    unsigned int LIBAPI wiican_ioctl_pending_result_8(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_pending_result_32(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_pending_result_64(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_pending_result_tDEVICE_STATUS(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_pending_result_tCAN_PARAMETER(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_pending_result_tDEVICE_INFO(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_pending_result_tCAN_FILTER(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_pending_result_tCAN_CHANNEL_CONTROL(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
    unsigned int LIBAPI wiican_ioctl_pending_result_Byte_Array(CAN_HANDLE handle, tIOCTL_ACT act, void* data, unsigned int len);
//end of the java JNA wrapper for IOCTL function.
}
//end of the function definition section.

#endif // WIICAN_DRIVER_H
