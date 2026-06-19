/*
 * ESP32-S3 CSI Receiver
 * 连接路由器(AP)，作为 Station 接收 WiFi 数据包并提取 CSI 数据
 * 数据同时输出到：1) 串口 (实时监控)  2) 裸 Flash 分区 (持久化保存)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

/* ========== 用户配置 (通过 sdkconfig.defaults 或在此修改) ========== */
#ifndef CONFIG_ESP_WIFI_SSID
#define WIFI_SSID       "YOUR_ROUTER_SSID"
#else
#define WIFI_SSID       CONFIG_ESP_WIFI_SSID
#endif

#ifndef CONFIG_ESP_WIFI_PASSWORD
#define WIFI_PASSWORD   "YOUR_ROUTER_PASSWORD"
#else
#define WIFI_PASSWORD   CONFIG_ESP_WIFI_PASSWORD
#endif

/* 存储分区标签 (与 partitions_csi.csv 一致) */
#define CSI_PARTITION_LABEL  "storage"

/* CSI 采样冷却期：跳过连续同 MAC 的包以减小存储压力 (0 = 不限制, 单位 ms) */
#define CSI_SAMPLE_COOLDOWN_MS  0

/* 串口子载波详细打印 (数据量大时建议关闭以提高性能) */
#define CSI_PRINT_SUBCARRIERS  1

/* ========== CSI 记录二进制格式 ==========
 * ┌──────────────┬────────────────┬──────────┬──────────────┬───────────┬──────────────┐
 * │ magic (4B)   │ timestamp (8B) │ rssi(1B) │ src_mac (6B) │ len (2B)  │ csi_buf (nB) │
 * │ 0x43534921   │ uint64_t (us)  │ int8_t   │              │ uint16_t  │ raw bytes    │
 * └──────────────┴────────────────┴──────────┴──────────────┴───────────┴──────────────┘
 */
#define CSI_RECORD_MAGIC     0x43534921
#define CSI_RECORD_HEADER_SZ (4 + 8 + 1 + 6 + 2)   /* 21 字节头部 */

/* ========== 常量 ========== */
static const char *TAG = "CSI_RX";

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* ========== 全局状态 ========== */
static EventGroupHandle_t s_wifi_event_group;
static SemaphoreHandle_t   s_part_mutex     = NULL;
static const esp_partition_t *s_csi_part    = NULL;
static uint32_t            s_part_offset     = 0;    /* 当前写偏移 */
static uint32_t            s_part_size       = 0;    /* 分区总大小 */
static uint32_t            s_record_cnt      = 0;    /* 已写记录数 */
static uint8_t             s_last_mac[6]     = {0};
static int64_t             s_last_csi_ts_us  = 0;

/* ========== 分区存储管理 ========== */

static esp_err_t partition_init(void)
{
    /* 按标签查找分区 */
    s_csi_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY,
        CSI_PARTITION_LABEL);

    if (s_csi_part == NULL) {
        ESP_LOGE(TAG, "Partition '%s' not found! Check partitions_csi.csv",
                 CSI_PARTITION_LABEL);
        return ESP_ERR_NOT_FOUND;
    }

    s_part_size   = s_csi_part->size;
    s_part_offset = 0;
    s_record_cnt  = 0;

    ESP_LOGI(TAG, "Partition '%s' found: addr=0x%"PRIx32", size=%"PRIu32" KB",
             CSI_PARTITION_LABEL,
             s_csi_part->address,
             s_part_size / 1024);

    /* 擦除整个分区 (干净起始) */
    esp_err_t ret = esp_partition_erase_range(s_csi_part, 0, s_part_size);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Partition erased OK");
    } else {
        ESP_LOGE(TAG, "Partition erase failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/** 将一条 CSI 记录写入分区 (线程安全) */
static void partition_write_record(const uint8_t *mac, int8_t rssi,
                                   const int8_t *csi_buf, uint16_t csi_len)
{
    if (s_csi_part == NULL) return;
    if (xSemaphoreTake(s_part_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    uint32_t rec_sz = CSI_RECORD_HEADER_SZ + csi_len;

    /* 检查剩余空间 (留 4KB 安全余量) */
    if (s_part_offset + rec_sz > s_part_size - 4096) {
        ESP_LOGW(TAG, "Partition full! offset=%"PRIu32"/%"PRIu32", records=%"PRIu32,
                 s_part_offset, s_part_size, s_record_cnt);
        xSemaphoreGive(s_part_mutex);
        return;
    }

    int64_t ts = esp_timer_get_time();

    /* 组装头部 */
    uint8_t header[CSI_RECORD_HEADER_SZ];
    uint32_t magic  = CSI_RECORD_MAGIC;
    uint16_t len_u16 = csi_len;

    memcpy(header + 0,  &magic,  4);
    memcpy(header + 4,  &ts,     8);
    memcpy(header + 12, &rssi,   1);
    memcpy(header + 13, mac,     6);
    memcpy(header + 19, &len_u16, 2);

    /* 写入头部 + CSI 数据 */
    esp_err_t ret = esp_partition_write(s_csi_part, s_part_offset, header,
                                        CSI_RECORD_HEADER_SZ);
    if (ret == ESP_OK) {
        ret = esp_partition_write(s_csi_part, s_part_offset + CSI_RECORD_HEADER_SZ,
                                  (const void *)csi_buf, csi_len);
    }

    if (ret == ESP_OK) {
        s_part_offset += rec_sz;
        s_record_cnt++;
    } else {
        ESP_LOGE(TAG, "Write failed at offset %"PRIu32": %s",
                 s_part_offset, esp_err_to_name(ret));
    }

    xSemaphoreGive(s_part_mutex);
}

/* ========== CSI 回调 ========== */

static void csi_rx_callback(void *ctx, wifi_csi_info_t *data)
{
    if (data->first_word_invalid || data->len <= 0 || data->buf == NULL) {
        return;
    }

    /* 可选：冷却期限制 */
    if (CSI_SAMPLE_COOLDOWN_MS > 0) {
        if (memcmp(s_last_mac, data->mac, 6) == 0) {
            int64_t now = esp_timer_get_time();
            if ((now - s_last_csi_ts_us) < (CSI_SAMPLE_COOLDOWN_MS * 1000)) {
                return;
            }
        }
        memcpy(s_last_mac, data->mac, 6);
        s_last_csi_ts_us = esp_timer_get_time();
    }

    int8_t rssi = data->rx_ctrl.rssi;

    /* ==== 串口：一行 CSV 概要 ==== */
    printf("CSI,%02X:%02X:%02X:%02X:%02X:%02X,%d,%d,%d,%d\n",
           data->mac[0], data->mac[1], data->mac[2],
           data->mac[3], data->mac[4], data->mac[5],
           rssi,
           data->rx_ctrl.rate,
           data->len,
           data->rx_ctrl.sig_mode);

#if CSI_PRINT_SUBCARRIERS
    /* 可选：详细子载波幅值 (数据量大时可关闭)
     * 每个子载波 = 2 个 int16_t (real, imag) = 4 字节
     * 当 ltf_merge_en=true 时, buffer 前有 header (约 24 字节) */
    {
        int n_sub = data->len / 4;           /* 每子载波 4 字节 */
        int header_words = 0;
        if (data->len > 24) {
            header_words = 6;                /* header = 24 bytes = 6 个 int16 */
        }
        int valid = n_sub - header_words;
        if (valid > 0) {
            int16_t *sub = (int16_t *)(data->buf) + header_words;
            printf("CSI_SUB,");
            for (int i = 0; i < valid && i < 128; i++) {
                printf("%d,%d", sub[2 * i], sub[2 * i + 1]);
                if (i < valid - 1) putchar(',');
            }
            putchar('\n');
        }
    }
#endif

    /* ==== 写入 Flash 分区 ==== */
    partition_write_record(data->mac, rssi, data->buf, data->len);
}

/* ========== WiFi 事件处理 ========== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            /* 不自动连接 — 等 scan + set_config 后手动 connect */
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *ev =
                (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "Disconnected, reason=%d, reconnecting...", ev->reason);
            esp_wifi_connect();
            break;
        }
        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

/* WiFi STA 初始化 + CSI 配置 */
static void wifi_init_sta_with_csi(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    /* 启动 WiFi (不设 SSID, 先扫描) */
    ESP_ERROR_CHECK(esp_wifi_start());

    /* ---- 扫描可见 AP (必须在设置 STA config 之前) ---- */
    {
        wifi_scan_config_t scan_cfg = {
            .ssid    = NULL,
            .bssid   = NULL,
            .channel = 0,
            .show_hidden = false,
        };
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_cfg, true));
        uint16_t ap_count = 0;
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
        ESP_LOGI(TAG, "Scan found %d AP(s):", ap_count);

        if (ap_count > 0) {
            wifi_ap_record_t *ap_list = calloc(ap_count, sizeof(wifi_ap_record_t));
            ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));
            for (int i = 0; i < ap_count; i++) {
                ESP_LOGI(TAG, "  [%d] SSID:%-24s RSSI:%-4d CH:%-2d AUTH:%-2d",
                         i, ap_list[i].ssid, ap_list[i].rssi,
                         ap_list[i].primary, ap_list[i].authmode);
            }
            free(ap_list);
        } else {
            ESP_LOGW(TAG, "No AP found! Check: 2.4GHz band? Antenna connected?");
        }
    }

    /* 设置 STA 配置并连接 */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /* 手动发起连接 */
    ESP_LOGI(TAG, "Connecting to %s...", WIFI_SSID);
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "WiFi connection failed!");
        return;
    }
    ESP_LOGI(TAG, "WiFi connected to: %s", WIFI_SSID);

    /* 关闭省电模式，确保能持续接收数据包 */
    esp_wifi_set_ps(WIFI_PS_NONE);

    /* ---- CSI 配置 (v6.0 必须在连接后配置, 否则干扰扫描/连接) ---- */
    wifi_csi_config_t csi_cfg = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = false,
        .manu_scale        = false,
        .shift             = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&csi_rx_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    ESP_LOGI(TAG, "CSI enabled, receiving...");
}

/* ========== 主入口 ========== */

static void traffic_gen_task(void *arg);

void app_main(void)
{
    /* 1. NVS 初始化 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. 存储分区初始化 */
    s_part_mutex = xSemaphoreCreateMutex();
    ret = partition_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Storage init failed, CSI will NOT be saved. Continuing...");
        /* 不中止运行，串口输出仍然有效 */
    }

    /* 3. WiFi + CSI 初始化 */
    wifi_init_sta_with_csi();

    /* 4. 启动流量生成任务 (持续发 UDP 包产生 RX 流量 → 触发 CSI) */
    xTaskCreate(&traffic_gen_task, "traffic", 4096, NULL, 5, NULL);

    /* 5. 定期状态上报 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        ESP_LOGI(TAG, "Records: %"PRIu32" | Used: %"PRIu32" / %"PRIu32" KB (%.1f%%)",
                 s_record_cnt,
                 s_part_offset / 1024,
                 s_part_size / 1024,
                 100.0 * s_part_offset / s_part_size);
    }
}

static void traffic_gen_task(void *arg)
{
    /* 等待 WiFi 拿到 IP */
    vTaskDelay(pdMS_TO_TICKS(3000));

    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        ESP_LOGE(TAG, "traffic: no netif");
        vTaskDelete(NULL);
        return;
    }
    esp_netif_get_ip_info(netif, &ip_info);
    uint32_t gw = ip_info.gw.addr;
    ESP_LOGI(TAG, "Traffic gen: sending to gateway " IPSTR, IP2STR(&ip_info.gw));

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(9),
        .sin_addr   = { .s_addr = gw },
    };

    char dummy[32] = "CSI_TRAFFIC";
    while (1) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock >= 0) {
            sendto(sock, dummy, sizeof(dummy), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
            close(sock);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
