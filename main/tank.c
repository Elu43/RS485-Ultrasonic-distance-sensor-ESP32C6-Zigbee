\
#include "tank.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"

#include "../switch_driver/include/switch_driver.h"

const static char *TAG = "tank_sensor_a02_rs485";

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile router source code.
#endif

// -------------------- RS485 / Modbus configuration --------------------
// Pins MUST match your Arduino sketch:
#define RS485_UART_NUM      UART_NUM_1
#define RS485_TX_GPIO       GPIO_NUM_22
#define RS485_RX_GPIO       GPIO_NUM_23
#define RS485_DE_GPIO       GPIO_NUM_2     // DE/RE enable pin (TX when HIGH, RX when LOW)
#define RS485_BAUDRATE      9600

// Modbus request used in your Arduino sketch:
// 01 03 01 01 00 01 D4 36  -> read holding register 0x0101, qty=1, slave=1
static const uint8_t MODBUS_REQ_READ_DISTANCE[] = {0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xD4, 0x36};

// -------------------- Zigbee cluster/attribute used to publish distance --------------------
#define CLUSTER_ID   ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT
#define ATTRIBUTE_ID ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID

// Reporting parameters (can be updated from Z2M; persisted in NVS)
static uint16_t mm_delta = 50;        // reportable change in millimeters
static uint16_t sample_period_s = 5;  // sampling/reporting period in seconds

// -------------------- Smoothing (moving average) --------------------
#define SAMPLE_COUNT 5
static uint16_t samples[SAMPLE_COUNT];
static size_t sample_i = 0;

// -------------------- Button (factory reset Zigbee storage) --------------------
static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};

static void esp_app_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL)
    {
        ESP_EARLY_LOGW(TAG, "Factory reset Zigbee settings. The device will completely erase the zb_storage partition and then restart.");
        esp_zb_factory_reset();
    }
}

static uint16_t getMovingAverage(uint16_t latestSample)
{
    samples[sample_i] = latestSample;
    sample_i++;
    if (sample_i >= SAMPLE_COUNT) {
        sample_i = 0;
    }

    uint32_t sum = 0;
    uint16_t samplesSummed = 0;

    for (int i = 0; i < SAMPLE_COUNT; ++i)
    {
        if (samples[i] > 0)
        {
            sum += samples[i];
            samplesSummed++;
        }
    }

    if (samplesSummed == 0) {
        return latestSample;
    }
    return (uint16_t)(((float)sum) / samplesSummed);
}

// -------------------- Modbus CRC16 (RTU) --------------------
static uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static inline void rs485_set_tx(bool tx)
{
    gpio_set_level(RS485_DE_GPIO, tx ? 1 : 0);
}

// Read A02 distance via Modbus RTU.
// Returns true on success, and fills distance_mm.
static bool a02_read_distance_mm(uint16_t *distance_mm)
{
    // Flush RX buffer before transaction
    uart_flush_input(RS485_UART_NUM);

    // TX
    rs485_set_tx(true);
    vTaskDelay(pdMS_TO_TICKS(5));

    uart_write_bytes(RS485_UART_NUM, (const char *)MODBUS_REQ_READ_DISTANCE, sizeof(MODBUS_REQ_READ_DISTANCE));
    if (uart_wait_tx_done(RS485_UART_NUM, pdMS_TO_TICKS(100)) != ESP_OK) {
        rs485_set_tx(false);
        return false;
    }

    // RX
    rs485_set_tx(false);

    // Typical response for 1 register: 01 03 02 HI LO CRClo CRChi  -> 7 bytes
    uint8_t buf[64];
    int len = uart_read_bytes(RS485_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(200));

    if (len < 7) {
        return false;
    }

    // Search for a valid frame start in the buffer
    for (int i = 0; i <= len - 7; i++) {
        if (buf[i] == 0x01 && buf[i + 1] == 0x03 && buf[i + 2] == 0x02) {

            // Validate CRC for the 7-byte frame
            uint16_t crc_calc = modbus_crc16(&buf[i], 5); // address..data (5 bytes)
            uint16_t crc_rx = (uint16_t)buf[i + 5] | ((uint16_t)buf[i + 6] << 8); // low, high
            if (crc_calc != crc_rx) {
                continue;
            }

            uint16_t raw = ((uint16_t)buf[i + 3] << 8) | buf[i + 4];

            // Convert raw register to millimeters:
            // Many A02 RS485 firmwares return distance in millimeters. If yours differs, adapt here.
            *distance_mm = raw;

            return true;
        }
    }

    return false;
}

// -------------------- Zigbee reporting config --------------------
static void update_reporting()
{
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .ep = 1,
        .cluster_id = CLUSTER_ID,
        .attr_id = ATTRIBUTE_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,

        .u.send_info.min_interval = sample_period_s,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = sample_period_s,
        .u.send_info.def_max_interval = 0,

        .u.send_info.delta.u16 = mm_delta,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_info);
}

// If Z2M changes reporting (min interval / reportable change), persist it and apply it.
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *msg)
{
    (void)callback_id;
    (void)msg;

    esp_zb_zcl_attr_location_info_t filter = {
        .attr_id = ATTRIBUTE_ID,
        .cluster_id = CLUSTER_ID,
        .endpoint_id = 1,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
    };

    esp_zb_zcl_reporting_info_t *report = esp_zb_zcl_find_reporting_info(filter);
    if (report != NULL)
    {
        uint16_t new_mm_delta = report->u.send_info.delta.u16;
        uint16_t new_sample_period = report->u.send_info.min_interval;

        if (new_mm_delta != mm_delta || new_sample_period != sample_period_s)
        {
            mm_delta = new_mm_delta;
            sample_period_s = new_sample_period;

            nvs_handle_t my_handle;
            ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));
            ESP_ERROR_CHECK(nvs_set_u16(my_handle, "mm_delta", mm_delta));
            ESP_ERROR_CHECK(nvs_set_u16(my_handle, "sample_period_s", sample_period_s));
            ESP_ERROR_CHECK(nvs_commit(my_handle));
            nvs_close(my_handle);

            ESP_LOGW(TAG, "Set new mm_delta: %u, sample_period_s: %u", mm_delta, sample_period_s);
        }
    }

    return ESP_OK;
}

// -------------------- Endpoint + clusters --------------------
static void add_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep_num)
{
    esp_zb_endpoint_config_t endpoint = {
        .endpoint = ep_num,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CONSUMPTION_AWARENESS_DEVICE_ID,
        .app_device_version = 0
    };

    esp_zb_basic_cluster_cfg_t basic = {
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE
    };

    esp_zb_identify_cluster_cfg_t identify = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
    };

    esp_zb_illuminance_meas_cluster_cfg_t cfg = {
        .max_value = 65535,
        .min_value = 0,
        .measured_value = 0
    };

    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic);
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list, esp_zb_illuminance_meas_cluster_create(&cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint);
}

// -------------------- NVS config load --------------------
static void load_config()
{
    nvs_handle_t my_handle;
    bool didWrite = false;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));

    if (nvs_get_u16(my_handle, "mm_delta", &mm_delta) == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "mm_delta", mm_delta));
        didWrite = true;
    }

    if (nvs_get_u16(my_handle, "sample_period_s", &sample_period_s) == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "sample_period_s", sample_period_s));
        didWrite = true;
    }

    if (didWrite)
    {
        ESP_ERROR_CHECK(nvs_commit(my_handle));
    }

    nvs_close(my_handle);
    ESP_LOGW(TAG, "Read from memory: mm_delta: %u, sample_period_s: %u", mm_delta, sample_period_s);
}

// -------------------- Periodic task: read RS485 and publish to Zigbee --------------------
static void read_rs485_task(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(5000));

    for (;;)
    {
        uint16_t dist_mm = 0;
        if (a02_read_distance_mm(&dist_mm))
        {
            uint16_t smooth = getMovingAverage(dist_mm);

            ESP_LOGI(TAG, "Distance: %u mm (smoothed: %u)", dist_mm, smooth);

            if (esp_zb_lock_acquire(portMAX_DELAY))
            {
                esp_zb_zcl_set_attribute_val(
                    1, CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ATTRIBUTE_ID, &smooth, false
                );
                esp_zb_lock_release();
            }
        }
        else {
            ESP_LOGW(TAG, "Failed to read A02 distance (RS485/Modbus)");
        }

        vTaskDelay(pdMS_TO_TICKS(sample_period_s * 1000));
    }
}

// -------------------- Deferred driver init (after Zigbee stack is ready) --------------------
static esp_err_t deferred_driver_init(void)
{
    // UART init
    ESP_LOGI(TAG, "Init RS485 UART (TX=GPIO22 RX=GPIO23 DE=GPIO2, 9600 8N1)");
    uart_config_t uart_config = {
        .baud_rate = RS485_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_NUM, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_NUM, RS485_TX_GPIO, RS485_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // DE/RE pin
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << RS485_DE_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    rs485_set_tx(false); // RX by default

    // Init smoothing buffer
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        samples[i] = 0;
    }

    // Button driver (factory reset)
    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), esp_app_buttons_handler),
                        ESP_FAIL, TAG, "Failed to initialize switch driver");

    // Start periodic RS485 read task
    BaseType_t ret = xTaskCreate(read_rs485_task, "read_a02_rs485", 4096, NULL, 4, NULL);
    return (ret == pdTRUE) ? ESP_OK : ESP_FAIL;
}

// -------------------- Zigbee task + commissioning --------------------
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");

            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG,
                     "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    (void)pvParameters;

    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    add_endpoint(ep_list, 1);
    esp_zb_device_register(ep_list);

    load_config();
    update_reporting();

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
