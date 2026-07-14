/**
 * @copyright Copyright (c) 2020 - 2020 Beijing Sixents Technology Co., Ltd.
 *       All rights reserved.
 * @file    sixents_sdk.h
 * @author  sixents@sixents.com
 * @version 1.0
 * @date    2020-5-30
 * @brief   定义外部SDK接口函数，数据结构、返回码；
 * @details
 *
 * @note
 *    change history:
 *    <2020-5-30>  | 1.0 | sixents@sixents.com | Create initial version
 */

#ifndef _SIXENTS_SDK_H_
#define _SIXENTS_SDK_H_

#include "sixents_types.h"

#ifdef _WIN32
#define STD_CALL _stdcall
#ifdef DLL_EXPORT
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif
#else
#define DLL_API __attribute__((visibility("default")))
#define STD_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif
// 返回码定义
typedef enum
{
    // 成功
    SIXENTS_RET_OK                      = 0,

    // 接口错误码（-1~-10）
    SIXENTS_RET_FAILED                  = -1,   ///< 失败
    SIXENTS_RET_NULL_PTR                = -2,   ///< 空指针
    SIXENTS_RET_INVALID_PARAM           = -3,   ///< 无效参数

    //编译器宽度
    SIXENTS_RTT_COMPILER_SIZE           = -4, //编译器宽度未对齐(请采用系统默认对齐方式)
    // 状态错误 （-11~-100）
    SIXENTS_RET_INVALID_STATUS          = -11,  ///< SDK状态不合法

    // 网络错误 (-101~-200)
    SIXENTS_RET_SOCKET_DISCONNECTED     = -101, ///< 网络被断开
    SIXENTS_RET_SOCKET_CREATE_ERROR     = -102, ///< 网络创建失败
    SIXENTS_RET_SOCKET_BIND_ERROR       = -103, ///< 网络绑定本地IP失败
    SIXENTS_RET_SOCKET_CONNECT_ERROR    = -104, ///< 网络connetct失败
    SIXENTS_RET_SOCKET_SEND_ERROR       = -105, ///< 网络send失败
    SIXENTS_RET_SOCKET_RECV_ERROR       = -106, ///< 网络recv失败
    SIXENTS_RET_SOCKET_TIMEOUT_ERROR    = -107, ///< 网络超时
    SIXENTS_RET_DNS_ERROR               = -108, ///< 域名解析错误
    SIXENTS_RET_NW_STATUS_ERROR         = -109, ///< 网络状态错误
    SIXENTS_RET_SOCKET_INVALID_ERROR    = -110, ///< 无效的socket

    SIXENTS_RET_SSL_INIT_ERROR          = -111, ///< SSL初始化失败
    SIXENTS_RET_SSL_LOADCA_ERROR        = -112, ///< SSL Load RootCA失败
    SIXENTS_RET_SSL_HANDSHAKE_ERROR     = -113, ///< SSL handshake失败
    SIXENTS_RET_SSL_VERIFYSVRCERT_ERROR = -114, ///< SSL校验服务证书失败
    SIXENTS_RET_SSL_LOAD_CLIENT_CERT_ERROR = -115,    ///< SSL Load CLCA失败
    SIXENTS_RET_SSL_LOAD_CLIENT_PKEY_ERROR = -116,   ///< SSL Load CLPK失败
    // GGA (-201~-300)
    SIXENTS_RET_GGA_OUT_OF_SERVICE_AREA = -201, ///< the uploaded GGA is out of service area
    SIXENTS_RET_INVALID_GGA             = -202, ///< the uploaded GGA is invalid

    // 鉴权(-301~-400)
    SIXENTS_RET_AUTH_FAILED             = -301, ///< 鉴权失败

    // 启动服务失败(-401~-500)
    SIXENTS_RET_START_FAILED            = -401, ///< 启动失败

    // 系统错误 -501~-600
    SIXENTS_RET_OUT_OF_MEMORY           = -501, ///< 内存溢出
    SIXENTS_RET_START_THREAD_ERROR      = -502, ///< 启动线程失败
    SIXENTS_RET_OUT_OF_BOUNDARY         = -507,    ///< 数组越界

    // unknown error code
    SIXENTS_RET_UNKNOWN = -999,
} sixents_retCode;

// 状态码定义
typedef enum
{
    SIXENTS_STAT_INIT                               = 0,     ///< 状态码的初始值
    SIXENTS_STATE_NET_REQUEEST_OK                   = 1001,  ///< 网络请求成功
    SIXENTS_STATE_NET_REQUEEST_FAIL                 = 1002,  ///< 网络请求失败
    SIXENTS_STATE_NET_DISABLE                       = 1003,  ///< 网络不可用
    SIXENTS_STATE_AUTHENTICATE_OK                   = 1201,  ///< 鉴权成功
    SIXENTS_STATE_AUTHENTICATE_AKORAS_INVALID       = 1202,  ///< AK或AS参数无效
    SIXENTS_STATE_AUTHENTICATE_AS_INVALID           = 1203,  ///< AS无效
    SIXENTS_STATE_AUTHENTICATE_ACCOUNT_PASS_LIMIT   = 1204,  ///< 账号已用完
    SIXENTS_STATE_AUTHENTICATE_ACCOUNT_INVALID      = 1205,  ///< 账号不存在
    SIXENTS_STATE_AUTHENTICATE_FAIL                 = 1206,  ///< 鉴权失败
    SIXENTS_STATE_AUTHENTICATE_ACCOUNT_NOT_ACTIVE   = 1207,  ///< 账号未激活
    SIXENTS_STATE_AUTHENTICATE_ACCOUNT_OVERDUE      = 1208,  ///< 账号已过期
    SIXENTS_STATE_AUTHENTICATE_SERVER_EXCEPTION     = 1209,  ///< 服务器异常
    SIXENTS_STATE_AUTHENTICATE_LOADING              = 1210,  ///< AK/AS鉴权中
    SIXENTS_STATE_AUTHENTICATE_ACCOUNT_EXCEPTION    = 1211,  ///< 账号异常，为空等
    SIXENTS_STATE_AUTHENTICATE_BINDING_FAILURE      = 1212,  ///< 绑定失败
    SIXENTS_STATE_AUTHENTICATE_UNSUPPORTED_PROTOCOL = 1213,  ///< 不支持的协议
    SIXENTS_STATE_AUTHENTICATE_DEVICE_NO_BINDING    = 1214,  ///< 设备未手动绑定
    SIXENTS_STATE_AUTHENTICATE_ACTIVED_FAILURE      = 1215,  ///< 激活失败
    SIXENTS_STATE_HAVE_LOGIN_FAILURE                = 1216,  ///< 已登录
    SIXENTS_STATE_INSTANCE_UNUSED_FAILURE           = 1217,  ///< 没有账号
    SIXENTS_STATE_INSTANCE_IN_BLACKLIST             = 1218,  ///< 账号在黑名单
    SIXENTS_STATE_NW_CONNECT_SUCCESS                = 1301,  ///< 网络连接成功
    SIXENTS_STATE_NW_CONNECT_FAIL                   = 1302,  ///< 网络连接失败
    SIXENTS_STATE_NW_CONNECT_LOADING                = 1303,  ///< 网络连接中
    SIXENTS_STATE_NW_CONNECT_TIMEOUT                = 1304,  ///< 网络连接超时
    SIXENTS_STATE_NW_ACCOUNT_OUT                    = 1305,  ///< 同一账号在另外设备登录，提示账号不可以用
    SIXENTS_STATE_SERVER_AUTHENTICATE_FAIL          = 1306,  ///< 数据服务认证失败
    SIXENTS_STATE_SERVER_AUTHENTICATE_SUCCESS       = 1307,  ///< 数据服务认证成功
    SIXENTS_STATE_SERVER_DOMAIN_ERROR               = 1308,  ///< 域名解析错误
    SIXENTS_STATE_TLS_CONF_ERROR                    = 1311,  ///< tls init/set 失败
    SIXENTS_STATE_TLS_CONN_ERROR                    = 1312,  ///< tls conn 失败
    SIXENTS_STATE_RTCM_GET_SUCCESS                  = 1401,  ///< RTCM数据获取成功
    SIXENTS_STATE_RTCM_GRID_OUT_OF_RANGE            = 1402,  ///< 地理网格码不在服务范围
    SIXENTS_STATE_RTCM_GGA_OUT_OF_RANGE             = 1403,  ///< GGA不在服务范围
    SIXENTS_STATE_RTCM_GGA_GET_TIMEOUT              = 1404,  //< 60秒未接收到GGA数据
    SIXENTS_STATE_RTCM_GGA_SEND_EXCEPTION           = 1405,  ///< 发送GGA数据时异常
    SIXENTS_STATE_RTCM_GET_RTCM_TIMEOUT             = 1406,  ///< 60秒未获取到RTCM数据
    SIXENTS_STATE_RTCM_SERVER_ERR                   = 1407,  ///< RTCM服务器错误
    SIXENTS_STATE_RTCM_UNKNOWN_ERR                  = 1408,  ///< 未知错误
    SIXENTS_STATE_GGA_SUCCESS                       = 1501,  ///< GGA数据有效
    SIXENTS_STATE_GGA_INVALID                       = 1502,  ///< GGA数据无效

    SIXENTS_STATE_AGNSS_REQUEST                     = 1601,  ///< AGNSS数据请求
    SIXENTS_STATE_AGNSS_RECIVED                     = 1602,  ///< AGNSS数据接收完成
    SIXENTS_STATE_AGNSS_DECODE_SUCC                 = 1603,  ///< 解sixa成功
    SIXENTS_STATE_AGNSS_DECODE_FAIL                 = 1604,  ///< 解sixa失败
    SIXENTS_STATE_AGNSS_RECV_FAIL                   = 1605,  ///< AGNSS数据接收失败
    SIXENTS_STATE_AGNSS_SER_SUCCESS                 = 60000, ///< AGNSS数据响应成功
    SIXENTS_STATE_AGNSS_SER_TIME_EP                 = 61000, ///< 时间参数据异常
    SIXENTS_STATE_AGNSS_SER_GNSS_EP                 = 61001, ///< GNSS参数为空
    SIXENTS_STATE_AGNSS_SER_LON_ERR                 = 61002, ///< 经度参数错误
    SIXENTS_STATE_AGNSS_SER_LAT_ERR                 = 61003, ///< 纬度参数错误
    SIXENTS_STATE_AGNSS_SER_OUT_ERR                 = 61004, ///< 超出服务范围
    SIXENTS_STATE_AGNSS_SER_GNSS_ERR                = 61005, ///< GNSS参数错误
    SIXENTS_STATE_AGNSS_SER_LEN_ERR                 = 61006, ///< 用户名密码超15字符
    SIXENTS_STATE_AGNSS_SER_USR_EMP                 = 61007, ///< 用户名密码为空
    SIXENTS_STATE_AGNSS_SER_UNKNOW_ERR              = 69000, ///< 服务未知错误
} sixents_statCode;

// 鉴权类型
typedef enum
{
    SIXENTS_KEY_TYPE_AK  = 1, ///< AK
    SIXENTS_KEY_TYPE_DSK = 2, ///< DSK
    SIXENTS_KEY_TYPE_MIN = SIXENTS_KEY_TYPE_AK,
    SIXENTS_KEY_TYPE_MAX = SIXENTS_KEY_TYPE_DSK,
} sixents_keyType;

// 日志级别
typedef enum
{
    SIXENTS_LL_OFF   = 0, ///< 所有级别都不打印，即关闭日志打印
    SIXENTS_LL_ERROR = 1, ///< Error级别，只打印错误日志
    SIXENTS_LL_WARN,      ///< Warn级别，打印错误及警告的日志
    SIXENTS_LL_INFO,      ///< Info级别，用于信息输出，打印错误、警告、一般信息日志
    SIXENTS_LL_DEBUG,     ///< Debug级别，用于调试，打印所有日志信息
} sixents_logLevel;

// tls协议类型
typedef enum
{
    SIXENTS_PT_TLS_ONE = 1,
    SIXENTS_PT_TLS_TWO = 2,
    SIXENTS_PT_MIN     = SIXENTS_PT_TLS_ONE,
    SIXENTS_PT_MAX     = SIXENTS_PT_TLS_TWO,
} sixents_protocolType;

// 网络阻塞模式标志位
typedef enum
{
    SIXENTS_SOCK_IOFLAG_BLOCK   = 0, ///< 阻塞模式
    SIXENTS_SOCK_IOFLAG_NOBLOCK = 1, ///< 非阻塞模式
    SIXENTS_SOCK_IOFLAG_MIN = SIXENTS_SOCK_IOFLAG_BLOCK,
    SIXENTS_SOCK_IOFLAG_MAX = SIXENTS_SOCK_IOFLAG_NOBLOCK,
} sixents_sockIOFlag;

// 物理网络状态
typedef enum
{
    SIXENTS_NWSTATUS_ON  = 0, ///< 物理网络正常
    SIXENTS_NWSTATUS_OFF = 1, ///< 物理网络异常
    SIXENTS_NWSTATUS_MIN = SIXENTS_NWSTATUS_ON,
    SIXENTS_NWSTATUS_MAX = SIXENTS_NWSTATUS_OFF,
} sixents_nwStatus;

// rtcm缓存大小状态
typedef enum
{
    SIXENTS_BUFF_BIG = 0, ///< 大缓存1024
    SIXENTS_BUFF_MID = 1, ///< 中缓存512
    SIXENTS_BUFF_SMA = 2, ///< 小缓存256
    SIXENTS_BUFF_MIN = SIXENTS_BUFF_SMA,
    SIXENTS_BUFF_MAX = SIXENTS_BUFF_BIG,
} sixents_buffLevel;

// GeoHash精度等级(GeoHash precision Level)
typedef enum
{
    SIXENTS_GHP_LV1 = 1,
    SIXENTS_GHP_LV2,
    SIXENTS_GHP_LV3,
    SIXENTS_GHP_LV4,
    SIXENTS_GHP_LV5,
    SIXENTS_GHP_LV6,
    SIXENTS_GHP_LV7,
    SIXENTS_GHP_LV8,
    SIXENTS_GHP_LV9,
    SIXENTS_GHP_LV10,
    SIXENTS_GHP_LV11,
    SIXENTS_GHP_LV12,
} sixents_ghpLevel;

// 区域类型
typedef enum
{
    SIXENTS_LOCATE_LOCAL = 1,    ///< 国内区域
    SIXENTS_LOCATE_ABROAD = 2,   ///< 国外区域
    SIXENTS_LOCATE_GLOBAL = 3,   ///< 全球区域
    SIXENTS_LOCATE_OTHER  = 4,   ///< 其他区域 
} sixents_locateFlag;

// 回调函数原型
/**
 * @brief          获取差分数据的回调函数
 * @param[in]      buff: 差分数据
 * @param[in]      len:  差分数据的字节数
 * @return         N/A
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK
 */
typedef void (*sixents_cbGetDiffData)(const sixents_char* buff, sixents_uint32 len);

/**
 * @brief          获取当前状态的回调函数
 * @param[in]      status: 状态码
 * @return         N/A
 * @note           需开发者自己实现该函数，可获取网络、认证等状态，在SDK初始化时把函数指针传给SDK
 */
typedef void (*sixents_cbGetStatus)(sixents_uint32 status);

/**
 * @brief          日志打印回调函数
 * @param[in]      msg:     日志信息内容
 * @param[in]      msgLen:  日志信息长度
 * @return         函数执行情况
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK
 */
typedef sixents_int32 (*sixents_cbTrace)(const sixents_char* msg, sixents_uint16 msgLen);

/**
 * @brief          网络连接回调函数
 * @param[in]      serverIP:      服务器IP地址
 * @param[in]      serverPort:    服务器端口号
 * @param[in]      localIP:       本地IP地址
 * @param[in]      localPort:     本地端口号
 * @param[in]      sockIOBlockFlag:    是否阻塞模式的标志位，0为阻塞模式，1为非阻塞模式
 * @return         函数执行情况
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK
 */
typedef sixents_int32 (*sixents_cbConn)(const sixents_char* serverIP,
                                        sixents_uint16 serverPort,
                                        const sixents_char* localIP,
                                        sixents_uint16 localPort,
                                        sixents_sockIOFlag sockIOBlockFlag);

/**
 * @brief          网络发送回调函数
 * @param[in]      buff: 发送数据
 * @param[in]      len:  发送数据字节数
 * @return         函数执行情况
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK
 */
typedef sixents_int32 (*sixents_cbSend)(const sixents_char* buff, sixents_uint16 len);

/**
 * @brief          网络接收回调函数
 * @param[in]      buff: 接收数据
 * @param[in]      len:  接收数据字节数
 * @return         函数执行情况
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK
 */
typedef sixents_int32 (*sixents_cbRecv)(sixents_char* buff, sixents_uint16 len);

/**
 * @brief          网络断开连接回调函数
 * @return         函数执行情况
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK
 */
typedef sixents_int32 (*sixents_cbDisConn)(void);

// ToDo:增加返回值及形参的详细说明
/**
 * @brief          获取本地IP地址回调函数
 * @param[in]      localIp: 用于存放ip地址的buffer，需以'\0'结束
 * @param[in]      ipLen:  localIp buffer的最大长度，为固定值SIXENTS_MAX_HOST_LEN（32字节）
 * @return         实际IP的长度（不含结束符的长度）
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK
 */
typedef sixents_int32 (*sixents_cbGetLocalIp)(sixents_char* localIp, sixents_uint16 ipLen);

/**
 * @brief          获取本地端口回调函数
 * @return         本地端口号
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK
 */
typedef sixents_int32 (*sixents_cbGetLocalPort)(void);

// 回调函数原型
/**
 * @brief          获取辅助定位数据的回调函数
 * @param[in]      buff: 辅助定位数据
 * @param[in]      len:  数据的字节数
 * @return         N/A
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK
 */
typedef void(* sixents_agnss_cbData) (const sixents_char *buff, sixents_uint32 len);

/**
 * @brief          获取当前状态的回调函数
 * @param[in]      status : 状态码
 * @return         N/A
 * @note           需开发者自己实现该函数，可获取网络、认证等状态，在SDK初始化时把函数指针传给SDK
 */
typedef void(* sixents_agnss_cbStatus) (sixents_uint32 status);

/**
 * @brief          获取log日志的回调函数 
 * @param[in]      buff: 日志数据
 * @param[in]      len:  数据的字节数
 * @return         N/A
 * @note           需要开发者自己实现该函数，在SDK初始化时把函数指针传给SDK 。
 */
typedef void(* sixents_agnss_cbTrace) (const sixents_char *buff, sixents_uint32 len);

// 字符串长度常量定义（含结束符），用于sixents_sdkConf结构体
#define SIXENTS_MAX_AK_LEN          32  ///< AK的最大长度
#define SIXENTS_MAX_AS_LEN          68  ///< AS的最大长度
#define SIXENTS_MAX_DEV_ID_LEN      104 ///< Device ID的最大长度
#define SIXENTS_MAX_DEV_TYPE_LEN    24  ///< Device Type的最大长度
#define SIXENTS_MAX_HOST_LEN        64  ///< 域名的最大长度
#define SIXENTS_MAX_IPV4_LEN        64  ///< IP地址（v4）的最大长度
#define SIXENTS_MAX_MOUNT_POINT_LEN 32  ///< 挂载点的最大长度

// 字符串长度常量定义（含结束符），用于sixents_agnssConf结构体
#define SIXENTS_MAX_USR_LEN         32  ///< 用户名最大长度
#define SIXENTS_MAX_PWD_LEN         32  ///< 密码最大长度
#define SIXENTS_MAX_COMMON_LEN      32  ///< 字段最大长度
#define SIXENTS_MAX_DTYPE_LEN       6   ///< 数据类型长度

// 经纬度的范围
#define SIXENTS_MIN_LON         -180.0  ///< 经度下限
#define SIXENTS_MAX_LON          180.0  ///< 经度上限
#define SIXENTS_MIN_LAT          -90.0  ///< 纬度下限
#define SIXENTS_MAX_LAT           90.0  ///< 纬度上限
#define SIXENTS_INVALID_BLH   -20000.0  ///< 无效blh

//cellID无效值
#define SIXENTS_INVALID_CELLID "00-00-00-00" ///< 无效cellID

/// SDK初始化配置参数结构体
typedef struct
{
    sixents_uint32 paramSize;                             ///< 用于编译器宽度对齐检查
    sixents_keyType keyType;                              ///< 1 = AK, 2 = DSK，必填项
    sixents_char key[SIXENTS_MAX_AK_LEN];                 ///< 鉴权，用户名/AK，必填项
    sixents_char secret[SIXENTS_MAX_AS_LEN];              ///< 鉴权，密码/AS，必填项
    sixents_char devID[SIXENTS_MAX_DEV_ID_LEN];           ///< 鉴权，设备ID，必填项
    sixents_char devType[SIXENTS_MAX_DEV_TYPE_LEN];       ///< 鉴权，设备类型，必填项
    sixents_char openApiHost[SIXENTS_MAX_HOST_LEN];       ///< open Api(鉴权服务器) IP，域名
    sixents_uint16 openApiPort;                           ///< open Api(鉴权服务器) 端口
    sixents_char serverHost[SIXENTS_MAX_HOST_LEN];        ///< 差分服务器IP，域名
    sixents_uint16 serverPort;                            ///< 差分服务器端口
    sixents_char mountPoint[SIXENTS_MAX_MOUNT_POINT_LEN]; ///< 挂载点
    sixents_uint32 timeout;                               ///< 网络超时时间,单位为s,取值范围[1,60]
    sixents_sockIOFlag sockIOBlockFlag;                   ///< 网络模式标志位,0为阻塞模式,1为非阻塞模式
    sixents_logLevel logPrintLevel;                       ///< 日志打印级别
    sixents_protocolType pid;                             ///< tls协议标志位, 1为signal
    const sixents_char* rootCA;                           ///< 指向根证书,当pid=1时此标志位生效
    const sixents_char* clientCrt;            ///< 指向根证书,当pid=2时此标志位生效
    const sixents_char* clientKey;            ///< 指向根证书,当pid=2时此标志位生效
    sixents_cbGetDiffData cbGetDiffData;                  ///< 注册获取RTCM数据函数，必填项
    sixents_cbGetStatus cbGetStatus;                      ///< 注册获取当前状态函数，必填项
    sixents_cbTrace cbTrace;                              ///< 注册日志打印函数
    sixents_cbConn cbConn;                                ///< 注册网络连接函数
    sixents_cbSend cbSend;                                ///< 注册网络发送函数
    sixents_cbRecv cbRecv;                                ///< 注册网络接收函数
    sixents_cbDisConn cbDisConn;                          ///< 注册网络断开函数
    sixents_cbGetLocalIp cbGetLocalIp;                    ///< 注册获取本地IP函数
    sixents_cbGetLocalPort cbGetLocalPort;                ///< 注册获取本地端口函数
} sixents_sdkConf;


/// SDK初始化配置参数结构体
typedef struct 
{
/* data */
    sixents_char agnssHost[SIXENTS_MAX_HOST_LEN];         ///<agnss服務器地址，域名 
    sixents_uint16 agnssPort;                             ///<agnss服務端口 
    sixents_char localIP [SIXENTS_MAX_HOST_LEN];          ///<本地IP地址 
    sixents_uint16 localPort;                             ///<本地Port
    const sixents_char* rootCert;                         ///<安全证书
    sixents_logLevel logPrintLevel;                       ///<日志打印级别
    sixents_agnss_cbTrace cbTrace;                        ///<获取log的回调函数指针
    sixents_agnss_cbData cbData;                          ///<获取数据的回调函数指针，必填项 
    sixents_agnss_cbStatus cbStatus;                      ///<获取当前状态的回调函数指针，必填项
    sixents_cbConn cbConn;                                ///<20221109 lvxiaoyu 配置注册网络连接函数
    sixents_cbSend cbSend;                                ///<20221109 lvxiaoyu 配置注册网络发送函数
    sixents_cbRecv cbRecv;                                ///<20221109 lvxiaoyu 配置注册网络接收函数
    sixents_cbDisConn cbDisConn;                          ///<20221109 lvxiaoyu 配置注册网络断开函数
    sixents_char user [SIXENTS_MAX_USR_LEN];              ///<用户名，选填项 
    sixents_char pwd [SIXENTS_MAX_PWD_LEN];               ///<密码，选填项 
    sixents_char gnss [SIXENTS_MAX_COMMON_LEN];           ///<定位系统
    sixents_char dType [SIXENTS_MAX_COMMON_LEN];          ///<提供的数据类型
    ///sdk 客户端证书、私钥配置指针 by jhb
    const sixents_char* clientCrt;                        ///< 指向客户端证书,当pid=1，使用双向认证时使用该指针
    const sixents_char* clientKey;                        ///< 指向客户端私钥,当pid=1，使用双向认证时使用该指针
    /// 配置获取星历区域主要分国内国外 add by jhb 2025.03.12
    sixents_locateFlag  locateFlag;                       ///< 客户设置需要那个区域的星历
    sixents_uint32 paramSize;                             ///< 用于编译器宽度对齐检查
} sixents_agnssConf;
/*--------------------------caps-----------------------*/
/*
 * capability identifier
 */
#define SIXENTS_SDK_CAP_ID_NULL     (0U)        // 空
#define SIXENTS_SDK_CAP_ID_NOSR     (1U)        // n-rtk 服务
#define SIXENTS_SDK_CAP_ID_NSSR     (1U << 1)   // n-ppprtk 服务
#define SIXENTS_SDK_CAP_ID_LSSR     (1U << 2)   // L band 服务
#define SIXENTS_SDK_CAP_ID_EPH      (1U << 5)   // rtcm 格式星历
#define SIXENTS_SDK_CAP_ID_EPH_SUPL (1U << 6)   // supl 格式星历
#define SIXENTS_SDK_CAP_ID_SIDS     (1U << 7)   // 完好性服务

/*
 * capability state
 */
#define SIXENTS_SDK_CAP_STATE_NULL      (0U)    // 空
#define SIXNETS_SDK_CAP_STATE_INSERVICE (1U)    // 已激活并处于服务状态
#define SIXNETS_SDK_CAP_STATE_INACTIVE  (2U)    // 未激活状态
#define SIXENTS_SDK_CAP_STATE_SUSPENDED (3U)    // 能力已暂停状态
#define SIXENTS_SDK_CAP_STATE_EXPIRED   (4U)    // 能力已过期
#define SIXNETS_SDK_CAP_STATE_DISABLED  (5U)    // 能力未使能（未绑定）

/*
 * capability activation method
 */
#define SIXENTS_SDK_CAP_ACT_METHOD_AUTO   (0U)  // 默认可自动绑定进行激活
#define SIXENTS_SDK_CAP_ACT_METHOD_MANUAL (1U)  // 必须手动绑定进行激活

/*
 * capability Struct Max Num
 */
#define SIXENTS_SDK_MAX_CAPS (20U)

/*
 * capability Struct 
 */
typedef struct {
    sixents_uint32 caps_num;                  ///< 能力数量
    
    struct {
        sixents_uint32 cap_id;                ///< 能力 ID
        sixents_uint8  state;                 ///< 能力状态
        sixents_uint8  act_method;            ///< 能力激活方式
        sixents_uint64 expire_time;           ///< 能力过期时间
    } caps[SIXENTS_SDK_MAX_CAPS];
} sixents_sdk_cap_info_t;

/*--------------------------caps-----------------------*/

/**
 * @brief          初始化SDK
 * @param[in]      paramObj:  配置参数
 * @return         函数执行情况 \n
 *                  SIXENTS_RET_OK: 执行成功 \n
 *                  SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 *                  SIXENTS_RET_NULL_PTR: paramObj为空指针 \n
 *                  SIXENTS_RET_INVALID_PARAM: paramObj的成员变量中存在未设置或不符合要求的值 \n
 *                  SIXENTS_RET_FAILED: 初始Socket执行失败
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkInit(const sixents_sdkConf* paramObj);

/**
 * @brief          注销SDK
 * @return         函数执行情况 \n
 *                  SIXENTS_RET_OK: 执行成功
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkFinal(void);

/**
 * @brief          启动服务
 * @return         函数执行情况 \n
 *                  SIXENTS_RET_OK: 执行成功 \n
 *                  SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 *                  SIXENTS_RET_START_FAILED: 鉴权失败,导致启动失败 \n
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkStart(void);

/**
 * @brief          驱动SDK执行
 * @return         函数执行情况 \n
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkTick(void);

/**
 * @brief          停止服务
 * @return         函数执行情况 \n
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkStop(void);

/**
 * @brief          发送GGA数据字符串
 * @param[in]      gga:    按照NMEA0183标准格式拼接的字符串
 * @param[in]      ggaLen: gga的长度
 * @return         函数执行情况 \n
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 *                 SIXENTS_RET_NULL_PTR:paramObj为空指针 \n
 *                 SIXENTS_RET_INVALID_GGA: GGA无效 \n
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkSendGGAStr(const sixents_char* gga, sixents_uint16 ggaLen);

/**
 * @brief          发送GGA数据
 * @param[in]      lat:    纬度，单位为角度
 * @param[in]      lon:    经度，单位为角度
 * @param[in]      height: 高度，单位为米，精度到毫米
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 *                 SIXENTS_RET_NULL_PTR: paramObj为空指针 \n
 *                 SIXENTS_RET_INVALID_GGA: GGA无效 \n
 * @return         函数执行情况
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkSendGGA(sixents_float64 lat, sixents_float64 lon, sixents_float64 height);

/**
 * @brief          使用GeoHash的方式发送位置信息
 * @param[in]      lat:    纬度，单位为角度
 * @param[in]      lon:    经度，单位为角度
 * @param[in]      precision: 精度
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 *                 SIXENTS_RET_NULL_PTR: paramObj为空指针 \n
 *                 SIXENTS_RET_INVALID_GGA: GGA无效 \n
 * @return         函数执行情况
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkSendGeoHash(sixents_float64 lat, sixents_float64 lon, sixents_ghpLevel precision);

/**
 * @brief          使用隐藏经纬度，保留排障信息+geohash位置
 * @param[in]      gga:    位置信息
 * @param[in]      precision: 精度
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 *                 SIXENTS_RET_NULL_PTR: paramObj为空指针 \n
 *                 SIXENTS_RET_INVALID_GGA: GGA无效 \n
 * @return         函数执行情况
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkSendGeGga(const sixents_char* gga, sixents_ghpLevel precision);

/**
 * @brief          设置物理网络状态
 * @param[in]      curNwStatus: 当前物理网络状态
 * @return         函数执行情况 \n
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_PARAM: 无效参数,curNwStatus不在枚举范围内 \n
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkSetNwStatus(sixents_nwStatus curNwStatus);

/**
 * @brief          设置数据缓存等级
 * @param[in]      recvBuffLevel: 当前缓存等级状态
 * @return         函数执行情况 \n
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_PARAM: 无效参数,recvBuffLevel不在枚举范围内 \n
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_sdkSetBuff(sixents_buffLevel recvBuffLevel);

/**
 * @brief          获取SDK版本号
 * @return         SDK的版本号
 * @note           N/A
 */
DLL_API const sixents_char* STD_CALL sixents_sdkGetVer(void);

/*----------------------------------------------------------- AGNSS ------------------------------------------------*/

/**
 * @brief          初始化SDK
 * @param[in]      paramObj:  配置参数
 * @return         函数执行情况 \n
 *                  SIXENTS_RET_OK: 执行成功 \n
 *                  SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 *                  SIXENTS_RET_NULL_PTR: paramObj为空指针 \n
 *                  SIXENTS_RET_INVALID_PARAM: paramObj的成员变量中存在未设置或不符合要求的值 \n
 *                  SIXENTS_RET_FAILED: 初始Socket执行失败
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_agnssInit(const sixents_agnssConf* paramObj);

/**
 * @brief          注销SDK
 * @return         函数执行情况 \n
 *                  SIXENTS_RET_OK: 执行成功
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL  sixents_agnssFinal(void);


/**
 * @brief          发送请求参数
 * @param[in]      cellID:蜂窝小区id
 * @param[in]      time:参考时间
 * @param[in]      lon:   经度，单位为度
 * @param[in]      lat:   纬度，单位为度
 * @param[in]      alt:   海拔，单位为米
 * @return         函数执行情况 \n
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_agnssRequest(sixents_uint32 time, sixents_char* cellId, sixents_float64  lat, sixents_float64  lon, sixents_float64  alt);

/**
 * @brief          驱动获取服务器返回定位辅助数据
 * @return         函数执行情况 \n
 *                 SIXENTS_RET_OK: 执行成功 \n
 *                 SIXENTS_RET_INVALID_STATUS: SDK状态不正确 \n
 * @note           N/A
 */
DLL_API sixents_retCode STD_CALL sixents_agnssTake(void);


#ifdef __cplusplus
}
#endif

#endif  // INC_SIXENTS_SDK_H_
