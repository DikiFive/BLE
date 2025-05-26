/**
 * @file gattc_demo.c
 * @brief 这是一个BLE GATT客户端的演示程序。
 *
 * 该演示程序展示了BLE GATT客户端的功能。它能够扫描附近的BLE设备，并连接到其中一个设备。
 * 如果运行gatt_server演示程序，此客户端演示程序将自动连接到gatt_server演示程序。
 * 连接成功后，客户端演示程序将启用gatt_server的通知（notify）功能。
 * 之后，两个设备将开始交换数据。
 *
 */

#include <stdint.h>    // 标准整数类型定义
#include <string.h>    // 字符串操作函数
#include <stdbool.h>   // 布尔类型定义
#include <stdio.h>     // 标准输入输出函数
#include "nvs.h"       // 非易失性存储 (NVS) 库，用于存储配置数据
#include "nvs_flash.h" // NVS Flash 特定功能

#include "esp_bt.h"              // ESP32蓝牙通用API
#include "esp_gap_ble_api.h"     // ESP32 BLE GAP (Generic Access Profile) API
#include "esp_gattc_api.h"       // ESP32 BLE GATTC (Generic Attribute Profile Client) API
#include "esp_gatt_defs.h"       // ESP32 GATT定义，如UUID、权限等
#include "esp_bt_main.h"         // ESP32 蓝牙主控制器初始化
#include "esp_gatt_common_api.h" // ESP32 GATT通用API
#include "esp_log.h"             // ESP32 日志系统
#include "freertos/FreeRTOS.h"   // FreeRTOS 实时操作系统

#define GATTC_TAG "GATTC_DEMO"         // 日志标签，用于标识GATTC演示程序的日志输出
#define REMOTE_SERVICE_UUID 0x00FF     // 远程GATT服务器的服务UUID
#define REMOTE_NOTIFY_CHAR_UUID 0xFF01 // 远程GATT服务器的通知特性UUID
#define PROFILE_NUM 1                  // 客户端配置文件数量
#define PROFILE_A_APP_ID 0             // 客户端配置文件A的应用程序ID
#define INVALID_HANDLE 0               // 无效句柄值

static const char remote_device_name[] = "ESP_GATTS_DEMO"; // 目标远程设备的名称，客户端将尝试连接此设备
static bool connect = false;                               // 连接状态标志，true表示已连接或正在连接
static bool get_server = false;                            // 获取服务器信息状态标志，true表示已找到目标服务
static esp_gattc_char_elem_t *char_elem_result = NULL;     // 用于存储发现的特性信息
static esp_gattc_descr_elem_t *descr_elem_result = NULL;   // 用于存储发现的描述符信息

/* 声明静态函数 */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);                                          // GAP事件回调函数
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);                // GATTC事件回调函数
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param); // GATTC配置文件事件处理函数

// 远程服务UUID过滤器，用于搜索特定服务
static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16, // UUID长度为16位
    .uuid = {
        .uuid16 = REMOTE_SERVICE_UUID,
    }, // 远程服务UUID
};

// 远程特性UUID过滤器，用于搜索特定特性
static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16, // UUID长度为16位
    .uuid = {
        .uuid16 = REMOTE_NOTIFY_CHAR_UUID,
    }, // 远程通知特性UUID
};

// 通知描述符UUID，用于启用或禁用特性通知
static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16, // UUID长度为16位
    .uuid = {
        .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
    }, // 客户端特性配置描述符UUID
};

// BLE扫描参数配置
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,               // 扫描类型：主动扫描（会发送扫描请求）
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,           // 本地地址类型：公共地址
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL, // 扫描过滤策略：允许所有广播包
    .scan_interval = 0x50,                           // 扫描间隔：单位为0.625ms，0x50 = 80 * 0.625ms = 50ms
    .scan_window = 0x30,                             // 扫描窗口：单位为0.625ms，0x30 = 48 * 0.625ms = 30ms
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE     // 扫描重复过滤：禁用（每次发现都会报告）
};

// GATT客户端配置文件实例结构体
struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;       // GATT客户端回调函数
    uint16_t gattc_if;             // GATT客户端接口ID
    uint16_t app_id;               // 应用程序ID
    uint16_t conn_id;              // 连接ID
    uint16_t service_start_handle; // 服务起始句柄
    uint16_t service_end_handle;   // 服务结束句柄
    uint16_t char_handle;          // 特性句柄
    esp_bd_addr_t remote_bda;      // 远程设备的蓝牙地址
};

/*
 * 一个基于GATT的配置文件对应一个app_id和一个gattc_if。
 * 这个数组将存储由ESP_GATTS_REG_EVT事件返回的gattc_if。
 */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler, // 指定配置文件A的回调函数
        .gattc_if = ESP_GATT_IF_NONE,            /* 初始时未获取gatt_if，因此设置为ESP_GATT_IF_NONE */
    },
};

/**
 * @brief GATT客户端配置文件事件处理函数
 *
 * 该函数处理特定GATT客户端配置文件（PROFILE_A_APP_ID）的所有事件。
 *
 * @param event GATT客户端回调事件类型
 * @param gattc_if GATT客户端接口ID
 * @param param 事件参数，包含事件相关数据
 */
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param; // 将通用参数转换为GATTC特定参数

    switch (event)
    {
    case ESP_GATTC_REG_EVT: // 注册GATTC应用程序事件
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        // 设置BLE扫描参数
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret)
        {
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:
    { // GATT客户端连接事件
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id; // 存储连接ID
        // 存储远程设备的蓝牙地址
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        // 打印远程设备的蓝牙地址（十六进制）
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        // 发送MTU交换请求
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
        if (mtu_ret)
        {
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT: // GATT客户端打开连接事件
        if (param->open.status != ESP_GATT_OK)
        { // 如果打开连接失败
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success"); // 打开连接成功
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT: // GATT客户端服务发现完成事件（此事件通常在连接后自动触发，表示服务发现过程已完成）
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        { // 如果服务发现失败
            ESP_LOGE(GATTC_TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        // 搜索指定UUID的服务
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT: // GATT客户端配置MTU事件
        if (param->cfg_mtu.status != ESP_GATT_OK)
        { // 如果MTU配置失败
            ESP_LOGE(GATTC_TAG, "config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
    { // GATT客户端搜索结果事件（发现服务）
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        // 检查发现的服务是否是目标服务
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(GATTC_TAG, "service found");
            get_server = true;                                                                       // 标记已找到服务器
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle; // 存储服务起始句柄
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;     // 存储服务结束句柄
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: // GATT客户端搜索完成事件
        if (p_data->search_cmpl.status != ESP_GATT_OK)
        { // 如果搜索服务失败
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        // 打印服务来源信息
        if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
        {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        }
        else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
        {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server)
        { // 如果已找到目标服务
            uint16_t count = 0;
            // 获取服务中特性的数量
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC, // 属性类型：特性
                                                                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0)
            { // 如果找到特性
                // 为特性结果分配内存
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                {
                    ESP_LOGE(GATTC_TAG, "gattc no mem"); // 内存分配失败
                }
                else
                {
                    // 根据UUID获取特性
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                            remote_filter_char_uuid, // 目标特性UUID
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*
                     * 在我们的'ESP_GATTS_DEMO'演示中，每个服务只有一个特性，
                     * 因此我们使用第一个'char_elem_result'。
                     */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {                                                                                   // 如果特性支持通知
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle; // 存储特性句柄
                        // 注册通知
                        esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* 释放char_elem_result内存 */
                free(char_elem_result);
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "no char found"); // 未找到特性
            }
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    { // GATT客户端注册通知事件
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
        { // 如果注册通知失败
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1; // 启用通知的值
            // 获取特性描述符的数量
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR, // 属性类型：描述符
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0)
            { // 如果找到描述符
                // 为描述符结果分配内存
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                {
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem"); // 内存分配失败
                }
                else
                {
                    // 根据特性句柄和UUID获取描述符
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        p_data->reg_for_notify.handle, // 特性句柄
                                                                        notify_descr_uuid,             // 目标描述符UUID (客户端特性配置描述符)
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }
                    /*
                     * 在我们的'ESP_GATTS_DEMO'演示中，每个特性只有一个描述符，
                     * 因此我们使用第一个'descr_elem_result'。
                     */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        // 向客户端特性配置描述符写入值以启用通知
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                    descr_elem_result[0].handle, // 描述符句柄
                                                                    sizeof(notify_en),           // 写入数据长度
                                                                    (uint8_t *)&notify_en,       // 写入数据（启用通知）
                                                                    ESP_GATT_WRITE_TYPE_RSP,     // 写入类型：需要响应
                                                                    ESP_GATT_AUTH_REQ_NONE);     // 认证要求：无
                    }

                    if (ret_status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* 释放descr_elem_result内存 */
                    free(descr_elem_result);
                }
            }
            else
            {
                ESP_LOGE(GATTC_TAG, "decsr not found"); // 未找到描述符
            }
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT: // GATT客户端接收到通知或指示事件
        if (p_data->notify.is_notify)
        { // 如果是通知
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }
        else
        { // 如果是指示
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        // 打印接收到的通知/指示数据（十六进制）
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT: // GATT客户端写入描述符完成事件
        if (p_data->write.status != ESP_GATT_OK)
        { // 如果写入描述符失败
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success "); // 写入描述符成功
        uint8_t write_char_data[35];                 // 准备要写入特性的一些数据
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256; // 填充数据
        }
        // 向特性写入数据
        esp_ble_gattc_write_char(gattc_if,
                                 gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                 gl_profile_tab[PROFILE_A_APP_ID].char_handle, // 特性句柄
                                 sizeof(write_char_data),                      // 写入数据长度
                                 write_char_data,                              // 写入数据
                                 ESP_GATT_WRITE_TYPE_RSP,                      // 写入类型：需要响应
                                 ESP_GATT_AUTH_REQ_NONE);                      // 认证要求：无
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
    { // GATT客户端服务改变事件
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t)); // 复制远程设备的蓝牙地址
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t)); // 打印蓝牙地址
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT: // GATT客户端写入特性完成事件
        if (p_data->write.status != ESP_GATT_OK)
        { // 如果写入特性失败
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write char success "); // 写入特性成功
        break;
    case ESP_GATTC_DISCONNECT_EVT:                                                               // GATT客户端断开连接事件
        connect = false;                                                                         // 重置连接状态
        get_server = false;                                                                      // 重置获取服务器信息状态
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason); // 打印断开连接原因
        break;
    default:
        break;
    }
}

/**
 * @brief GAP (Generic Access Profile) 事件回调函数
 *
 * 该函数处理所有BLE GAP相关的事件，例如扫描结果、扫描完成等。
 *
 * @param event GAP回调事件类型
 * @param param 事件参数，包含事件相关数据
 */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL; // 广播名称指针
    uint8_t adv_name_len = 0; // 广播名称长度

    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    { // 扫描参数设置完成事件
        // 扫描持续时间，单位为秒
        uint32_t duration = 30;
        // 开始BLE扫描
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: // 扫描开始完成事件
        // 检查扫描是否成功启动
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success"); // 扫描成功启动

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {                                                                          // 扫描结果事件
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param; // 将通用参数转换为扫描结果特定参数
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT: // 发现设备结果事件
            // 打印设备的蓝牙地址（十六进制）
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            // 解析广播数据中的设备名称
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
            // 打印设备名称（字符形式）
            esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP // 如果启用了DUMP_ADV_DATA_AND_SCAN_RESP配置
            if (scan_result->scan_rst.adv_data_len > 0)
            {
                ESP_LOGI(GATTC_TAG, "adv data:");
                // 打印广播数据
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0)
            {
                ESP_LOGI(GATTC_TAG, "scan resp:");
                // 打印扫描响应数据
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif
            ESP_LOGI(GATTC_TAG, "\n");

            if (adv_name != NULL)
            {
                // 检查发现的设备名称是否与目标设备名称匹配
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0)
                {
                    ESP_LOGI(GATTC_TAG, "searched device %s\n", remote_device_name);
                    if (connect == false)
                    {                   // 如果尚未连接
                        connect = true; // 设置连接标志为true
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning(); // 停止扫描
                        // 打开GATT客户端连接到远程设备
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT: // 查询完成事件
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: // 扫描停止完成事件
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop scan successfully"); // 扫描成功停止
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: // 广播停止完成事件
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop adv successfully"); // 广播成功停止
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: // 连接参数更新事件
        ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT: // 数据包长度设置完成事件
        ESP_LOGI(GATTC_TAG, "packet length updated: rx = %d, tx = %d, status = %d",
                 param->pkt_data_lenth_cmpl.params.rx_len,
                 param->pkt_data_lenth_cmpl.params.tx_len,
                 param->pkt_data_lenth_cmpl.status);
        break;
    default:
        break;
    }
}

/**
 * @brief GATT客户端通用回调函数
 *
 * 该函数是所有GATT客户端事件的入口点。它根据事件类型和gattc_if将事件分发到
 * 相应的配置文件事件处理函数。
 *
 * @param event GATT客户端回调事件类型
 * @param gattc_if GATT客户端接口ID
 * @param param 事件参数，包含事件相关数据
 */
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* 如果事件是注册事件，则为每个配置文件存储gattc_if */
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if; // 存储gattc_if
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /*
     * 如果gattc_if等于PROFILE_A的gattc_if，则调用PROFILE_A的回调处理函数，
     * 因此这里调用每个配置文件的回调函数。
     */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE表示未指定特定gatt_if，需要调用所有配置文件回调函数 */
                gattc_if == gl_profile_tab[idx].gattc_if)
            {
                if (gl_profile_tab[idx].gattc_cb)
                {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param); // 调用对应配置文件的回调函数
                }
            }
        }
    } while (0);
}

/**
 * @brief 应用程序主函数
 *
 * 这是ESP-IDF应用程序的入口点。它负责初始化NVS、蓝牙控制器和Bluedroid协议栈，
 * 并注册GAP和GATTC的回调函数，最后注册GATTC应用程序。
 */
void app_main(void)
{
    // 初始化NVS (Non-Volatile Storage)。
    esp_err_t ret = nvs_flash_init();
    // 如果NVS初始化失败（例如，没有空闲页面或NVS版本不匹配），则擦除NVS并重新初始化。
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase()); // 擦除NVS
        ret = nvs_flash_init();             // 重新初始化NVS
    }
    ESP_ERROR_CHECK(ret); // 检查NVS初始化是否成功

    // 释放经典蓝牙模式的内存，因为此演示只使用BLE。
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 获取蓝牙控制器默认配置
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    // 初始化蓝牙控制器
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    // 启用BLE模式的蓝牙控制器
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    // 初始化Bluedroid协议栈
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    // 启用Bluedroid协议栈
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    // 将GAP回调函数注册到GAP模块
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    // 将GATTC回调函数注册到GATTC模块
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    // 注册GATTC应用程序，并指定应用程序ID
    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }
    // 设置本地MTU (Maximum Transmission Unit) 大小
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}
