#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "i2c_bus.h"
#if CONFIG_I2C_BUS_SUPPORT_SOFTWARE
#include "i2c_bus_soft.h" // 如果有软件I2C需求，这里是你自己的头文件
#endif

static const char *TAG = "i2c_bus";

/*
   在这里，定义一个结构体，记录当前总线信息。
   例如包括：是否已初始化、互斥锁、端口号、配置信息、引用计数等
*/
typedef struct {
    i2c_port_t i2c_port;            // 硬件 I2C 编号
    i2c_config_bus_t conf_activate; // 用户请求的 bus 配置
    bool is_init;                   // 是否已初始化
    SemaphoreHandle_t mutex;        // 用于实现线程安全
    int32_t ref_counter;            // 当前总线上已创建的 device 数量
} i2c_bus_t;

/*
   每个从设备信息：记录其 bus 和 7bit 地址(有些人会加自己的速度/ACK设置等)
*/
typedef struct {
    i2c_bus_t *i2c_bus;
    uint8_t dev_addr;
    uint32_t clk_speed; // 如果有需求可以记录设备期望的速度，但实际只能统一设置
} i2c_bus_device_t;

/* 定义一些宏，简化检查过程 */
#define I2C_BUS_CHECK(a, str, ret) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s:%d, %s", __FUNCTION__, __LINE__, str); \
        return (ret); \
    }

#define I2C_BUS_CHECK_GOTO(a, str, label) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s:%d, %s", __FUNCTION__, __LINE__, str); \
        goto label; \
    }

#define I2C_BUS_INIT_CHECK(is_init, ret) \
    if (!is_init) { \
        ESP_LOGE(TAG, "%s:%d, i2c_bus not inited", __FUNCTION__, __LINE__); \
        return (ret); \
    }

#define I2C_BUS_MUTEX_TAKE(mutex, ret) \
    if (!xSemaphoreTake(mutex, portMAX_DELAY)) { \
        ESP_LOGE(TAG, "%s:%d, take mutex fail", __FUNCTION__, __LINE__); \
        return (ret); \
    }

#define I2C_BUS_MUTEX_GIVE(mutex, ret) \
    if (!xSemaphoreGive(mutex)) { \
        ESP_LOGE(TAG, "%s:%d, give mutex fail", __FUNCTION__, __LINE__); \
        return (ret); \
    }

/* 转换并安装硬件 I2C 驱动 */
static esp_err_t i2c_driver_reinit(i2c_port_t port, const i2c_config_bus_t *conf)
{
#if CONFIG_I2C_BUS_SUPPORT_SOFTWARE
    /* 若要支持软件 I2C，请加相应逻辑 */
    /* 例如 if (port >= I2C_NUM_MAX) { ... } */
#endif
    // 先卸载可能遗留的 driver
    i2c_driver_delete(port);

    // 组装 i2c_config_t
    i2c_config_t param = {
        .mode = conf->mode,
        .sda_io_num = conf->sda_io_num,
        .scl_io_num = conf->scl_io_num,
        .sda_pullup_en = conf->sda_pullup_en,
        .scl_pullup_en = conf->scl_pullup_en,
        .clk_flags = conf->clk_flags
    };
    if (conf->mode == I2C_MODE_MASTER) {
        param.master.clk_speed = conf->master.clk_speed;
    }
#if SOC_I2C_SUPPORT_SLAVE
    else {
        // 如果要支持从机模式，这里可以配置 param.slave.xxx
    }
#endif
    esp_err_t ret = i2c_param_config(port, &param);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed, port=%d", port);
        return ret;
    }

    // 安装驱动。主机模式下 rx/tx buffer 都可以为 0
    ret = i2c_driver_install(port, conf->mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed, port=%d", port);
        return ret;
    }
    ESP_LOGI(TAG, "i2c_bus: i2c_driver_install done, port=%d", port);
    return ESP_OK;
}

/* 卸载硬件 I2C 驱动 */
static esp_err_t i2c_driver_deinit(i2c_port_t port)
{
#if CONFIG_I2C_BUS_SUPPORT_SOFTWARE
    /* 若要支持软件 I2C，在此处也需要区分port */
#endif
    // 直接删除硬件驱动
    esp_err_t ret = i2c_driver_delete(port);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_driver_delete failed, port=%d", port);
    }
    return ret;
}

static bool i2c_config_compare(const i2c_bus_t *bus, const i2c_config_bus_t *conf)
{
    // 如果需要更细致地对比 (如时钟等) 可以继续加
    if (bus->conf_activate.mode == conf->mode &&
        bus->conf_activate.sda_io_num == conf->sda_io_num &&
        bus->conf_activate.scl_io_num == conf->scl_io_num &&
        bus->conf_activate.sda_pullup_en == conf->sda_pullup_en &&
        bus->conf_activate.scl_pullup_en == conf->scl_pullup_en &&
        bus->conf_activate.master.clk_speed == conf->master.clk_speed) {
        return true;
    }
    return false;
}

/*========================= 对外 API 实现 =========================*/

i2c_bus_handle_t i2c_bus_create(i2c_port_t port, const i2c_config_bus_t *conf)
{
    I2C_BUS_CHECK(conf, "conf param null", NULL);

#if CONFIG_I2C_BUS_SUPPORT_SOFTWARE
    // 如果要兼容软件 I2C，判断 port >= I2C_NUM_MAX，走软件分支
#endif

    static i2c_bus_t s_i2c_bus[I2C_NUM_MAX]; // 简单示例，使用静态数组存放

    // 先检查 port 范围
    I2C_BUS_CHECK(port < I2C_NUM_MAX, "port invalid", NULL);

    i2c_bus_t *bus = &s_i2c_bus[port];
    if (!bus->mutex) {
        // 第一次创建时才分配互斥锁
        bus->mutex = xSemaphoreCreateMutex();
        if (!bus->mutex) {
            ESP_LOGE(TAG, "create mutex fail");
            return NULL;
        }
        bus->ref_counter = 0;
    }

    // 如果已初始化过并且配置相同，直接返回
    if (bus->is_init && i2c_config_compare(bus, conf)) {
        ESP_LOGW(TAG, "i2c_bus port %d already inited, reusing...", port);
        return (i2c_bus_handle_t)bus;
    }

    // 重新初始化
    I2C_BUS_MUTEX_TAKE(bus->mutex, NULL);
    esp_err_t ret = i2c_driver_reinit(port, conf);
    if (ret == ESP_OK) {
        bus->i2c_port = port;
        bus->conf_activate = *conf;
        bus->is_init = true;
        ESP_LOGI(TAG, "i2c_bus_create done, port=%d", port);
    } else {
        ESP_LOGE(TAG, "i2c_driver_reinit fail, port=%d", port);
        I2C_BUS_MUTEX_GIVE(bus->mutex, NULL);
        return NULL;
    }
    I2C_BUS_MUTEX_GIVE(bus->mutex, NULL);

    return (i2c_bus_handle_t)bus;
}

esp_err_t i2c_bus_delete(i2c_bus_handle_t *p_bus)
{
    I2C_BUS_CHECK(p_bus && *p_bus, "param null", ESP_ERR_INVALID_ARG);

    i2c_bus_t *bus = (i2c_bus_t *)(*p_bus);
    I2C_BUS_INIT_CHECK(bus->is_init, ESP_FAIL);

    // 若还有设备在用，ref_counter>0，则不卸载
    if (bus->ref_counter > 0) {
        ESP_LOGW(TAG, "cannot delete i2c_bus, ref_counter=%"PRIi32, bus->ref_counter);
        return ESP_FAIL;
    }

    // 安全处理
    I2C_BUS_MUTEX_TAKE(bus->mutex, ESP_FAIL);
    esp_err_t ret = i2c_driver_deinit(bus->i2c_port);
    bus->is_init = false;
    I2C_BUS_MUTEX_GIVE(bus->mutex, ESP_FAIL);

    vSemaphoreDelete(bus->mutex);
    bus->mutex = NULL;
    *p_bus = NULL;
    return ret;
}

uint32_t i2c_bus_get_current_clk_speed(i2c_bus_handle_t bus_handle)
{
    if (!bus_handle) {
        return 0;
    }
    i2c_bus_t *bus = (i2c_bus_t *)bus_handle;
    if (!bus->is_init) {
        return 0;
    }
    return bus->conf_activate.master.clk_speed;
}

uint8_t i2c_bus_get_created_device_num(i2c_bus_handle_t bus_handle)
{
    if (!bus_handle) {
        return 0;
    }
    i2c_bus_t *bus = (i2c_bus_t *)bus_handle;
    if (!bus->is_init) {
        return 0;
    }
    return (uint8_t)bus->ref_counter;
}

/*--------- 设备相关 ----------*/
i2c_bus_device_handle_t i2c_bus_device_create(i2c_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed)
{
    I2C_BUS_CHECK(bus_handle, "bus_handle null", NULL);
    i2c_bus_t *bus = (i2c_bus_t *)bus_handle;
    I2C_BUS_INIT_CHECK(bus->is_init, NULL);

    i2c_bus_device_t *dev = calloc(1, sizeof(i2c_bus_device_t));
    I2C_BUS_CHECK(dev, "calloc fail", NULL);

    // 这里如果要支持“不同设备用不同时钟”，由于 ESP-IDF 硬件 I2C 本身只能统一配置某个端口的频率，
    // 所以只能记录起来，但无法真正生效。（也可以直接忽略参数 clk_speed）
    dev->i2c_bus = bus;
    dev->dev_addr = dev_addr;
    dev->clk_speed = (clk_speed == 0) ? bus->conf_activate.master.clk_speed : clk_speed;

    I2C_BUS_MUTEX_TAKE(bus->mutex, NULL);
    bus->ref_counter++;
    I2C_BUS_MUTEX_GIVE(bus->mutex, NULL);

    return (i2c_bus_device_handle_t)dev;
}

esp_err_t i2c_bus_device_delete(i2c_bus_device_handle_t *p_dev_handle)
{
    I2C_BUS_CHECK(p_dev_handle && *p_dev_handle, "dev_handle null", ESP_ERR_INVALID_ARG);
    i2c_bus_device_t *dev = (i2c_bus_device_t *)(*p_dev_handle);

    i2c_bus_t *bus = dev->i2c_bus;
    I2C_BUS_MUTEX_TAKE(bus->mutex, ESP_FAIL);
    bus->ref_counter--;
    I2C_BUS_MUTEX_GIVE(bus->mutex, ESP_FAIL);

    free(dev);
    *p_dev_handle = NULL;
    return ESP_OK;
}

uint8_t i2c_bus_device_get_address(i2c_bus_device_handle_t dev_handle)
{
    if (!dev_handle) {
        return 0;
    }
    i2c_bus_device_t *dev = (i2c_bus_device_t *)dev_handle;
    return dev->dev_addr;
}

/*--------- 扫描总线 ----------*/
uint8_t i2c_bus_scan(i2c_bus_handle_t bus_handle, uint8_t *buf, uint8_t num)
{
    I2C_BUS_CHECK(bus_handle, "bus_handle null", 0);
    i2c_bus_t *bus = (i2c_bus_t *)bus_handle;
    I2C_BUS_INIT_CHECK(bus->is_init, 0);

    uint8_t found = 0;
    // 扫描 1~126  (0x00 和 0x7F大多 reserved)
    I2C_BUS_MUTEX_TAKE(bus->mutex, 0);
    for (uint8_t addr = 1; addr < 127; addr++) {
#if CONFIG_I2C_BUS_SUPPORT_SOFTWARE
        // 如果是软件 I2C，请改为 i2c_master_soft_bus_probe
#endif
        // 使用写 0 字节来探测
        esp_err_t ret = i2c_master_write_to_device(bus->i2c_port, addr, NULL, 0, pdMS_TO_TICKS(50));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "found device @0x%02X", addr);
            if (buf && found < num) {
                buf[found] = addr;
            }
            found++;
        }
        else if (ret == ESP_ERR_TIMEOUT) {
            // 总线忙或硬件故障
            ESP_LOGW(TAG, "i2c bus busy/time-out, addr=0x%02X", addr);
        }
        else {
            // ESP_FAIL => NACK 表示地址无设备
        }
    }
    I2C_BUS_MUTEX_GIVE(bus->mutex, 0);

    return found;
}

/*--------- 低层读/写辅助函数 ----------*/

static esp_err_t i2c_bus_write_reg8(i2c_bus_device_handle_t dev_handle, uint8_t mem_address, size_t data_len, const uint8_t *data)
{
    i2c_bus_device_t *dev = (i2c_bus_device_t *)dev_handle;
    I2C_BUS_CHECK(dev && data, "param error", ESP_ERR_INVALID_ARG);

    i2c_bus_t *bus = dev->i2c_bus;
    I2C_BUS_MUTEX_TAKE(bus->mutex, ESP_ERR_TIMEOUT);

    esp_err_t ret;
    if (mem_address != 0xFF) { // 约定: NULL_I2C_MEM_ADDR 用 0xFF 或别的值表示无需写寄存器地址
        // 先组合: [mem_address, data...]
        uint8_t *tmp = malloc(data_len + 1);
        if (!tmp) {
            I2C_BUS_MUTEX_GIVE(bus->mutex, ESP_ERR_NO_MEM);
            return ESP_ERR_NO_MEM;
        }
        tmp[0] = mem_address;
        memcpy(&tmp[1], data, data_len);
        ret = i2c_master_write_to_device(bus->i2c_port, dev->dev_addr, tmp, data_len + 1, pdMS_TO_TICKS(100));
        free(tmp);
    } else {
        // 不需要寄存器地址，直接写 data
        ret = i2c_master_write_to_device(bus->i2c_port, dev->dev_addr, data, data_len, pdMS_TO_TICKS(100));
    }

    I2C_BUS_MUTEX_GIVE(bus->mutex, ret);
    return ret;
}

static esp_err_t i2c_bus_read_reg8(i2c_bus_device_handle_t dev_handle, uint8_t mem_address, size_t data_len, uint8_t *data)
{
    i2c_bus_device_t *dev = (i2c_bus_device_t *)dev_handle;
    I2C_BUS_CHECK(dev && data, "param error", ESP_ERR_INVALID_ARG);

    i2c_bus_t *bus = dev->i2c_bus;
    I2C_BUS_MUTEX_TAKE(bus->mutex, ESP_ERR_TIMEOUT);

    esp_err_t ret;
    if (mem_address != 0xFF) {
        // 先写 mem_address, 再读 data
        ret = i2c_master_write_read_device(bus->i2c_port,
                                           dev->dev_addr,
                                           &mem_address,
                                           1,
                                           data,
                                           data_len,
                                           pdMS_TO_TICKS(100));
    } else {
        // 无寄存器地址，直接读
        ret = i2c_master_read_from_device(bus->i2c_port,
                                          dev->dev_addr,
                                          data,
                                          data_len,
                                          pdMS_TO_TICKS(100));
    }

    I2C_BUS_MUTEX_GIVE(bus->mutex, ret);
    return ret;
}

/*--------- 公开的读/写接口 ----------*/
esp_err_t i2c_bus_read_bytes(i2c_bus_device_handle_t dev_handle, uint8_t mem_address,
                             size_t data_len, uint8_t *data)
{
    return i2c_bus_read_reg8(dev_handle, mem_address, data_len, data);
}

esp_err_t i2c_bus_read_byte(i2c_bus_device_handle_t dev_handle, uint8_t mem_address,
                            uint8_t *data)
{
    return i2c_bus_read_reg8(dev_handle, mem_address, 1, data);
}

esp_err_t i2c_bus_read_bit(i2c_bus_device_handle_t dev_handle, uint8_t mem_address,
                           uint8_t bit_num, uint8_t *data)
{
    uint8_t byte;
    esp_err_t ret = i2c_bus_read_reg8(dev_handle, mem_address, 1, &byte);
    if (ret == ESP_OK) {
        *data = (byte >> bit_num) & 0x01;
    }
    return ret;
}

esp_err_t i2c_bus_read_bits(i2c_bus_device_handle_t dev_handle, uint8_t mem_address,
                            uint8_t bit_start, uint8_t length, uint8_t *data)
{
    uint8_t byte;
    esp_err_t ret = i2c_bus_read_byte(dev_handle, mem_address, &byte);
    if (ret != ESP_OK) {
        return ret;
    }
    // 先构造掩码，再提取
    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    byte &= mask;
    byte >>= (bit_start - length + 1);
    *data = byte;
    return ESP_OK;
}

esp_err_t i2c_bus_write_bytes(i2c_bus_device_handle_t dev_handle, uint8_t mem_address,
                              size_t data_len, const uint8_t *data)
{
    return i2c_bus_write_reg8(dev_handle, mem_address, data_len, data);
}

esp_err_t i2c_bus_write_byte(i2c_bus_device_handle_t dev_handle, uint8_t mem_address,
                             uint8_t data)
{
    return i2c_bus_write_reg8(dev_handle, mem_address, 1, &data);
}

esp_err_t i2c_bus_write_bit(i2c_bus_device_handle_t dev_handle, uint8_t mem_address,
                            uint8_t bit_num, uint8_t data)
{
    uint8_t byte;
    esp_err_t ret = i2c_bus_read_byte(dev_handle, mem_address, &byte);
    if (ret != ESP_OK) {
        return ret;
    }
    byte = data ? (byte | (1 << bit_num)) : (byte & ~(1 << bit_num));
    return i2c_bus_write_byte(dev_handle, mem_address, byte);
}

esp_err_t i2c_bus_write_bits(i2c_bus_device_handle_t dev_handle, uint8_t mem_address,
                             uint8_t bit_start, uint8_t length, uint8_t data)
{
    uint8_t byte;
    esp_err_t ret = i2c_bus_read_byte(dev_handle, mem_address, &byte);
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    data <<= (bit_start - length + 1);
    data &= mask;
    byte &= ~(mask);
    byte |= data;
    return i2c_bus_write_byte(dev_handle, mem_address, byte);
}

/*--------- 16bit 地址读写接口 ----------*/
esp_err_t i2c_bus_read_reg16(i2c_bus_device_handle_t dev_handle, uint16_t mem_address,
                             size_t data_len, uint8_t *data)
{
    I2C_BUS_CHECK(dev_handle && data, "param null", ESP_ERR_INVALID_ARG);
    i2c_bus_device_t *dev = (i2c_bus_device_t *)dev_handle;
    i2c_bus_t *bus = dev->i2c_bus;
    I2C_BUS_MUTEX_TAKE(bus->mutex, ESP_ERR_TIMEOUT);

    esp_err_t ret;
    if (mem_address != 0xFFFF) { // 约定 0xFFFF 表示无需寄存器地址
        // 先把 mem_address 高低字节放在一起
        uint8_t mem_buf[2];
        mem_buf[0] = (mem_address >> 8) & 0xFF;
        mem_buf[1] = mem_address & 0xFF;
        ret = i2c_master_write_read_device(bus->i2c_port,
                                           dev->dev_addr,
                                           mem_buf,
                                           2,
                                           data,
                                           data_len,
                                           pdMS_TO_TICKS(100));
    } else {
        ret = i2c_master_read_from_device(bus->i2c_port,
                                          dev->dev_addr,
                                          data,
                                          data_len,
                                          pdMS_TO_TICKS(100));
    }

    I2C_BUS_MUTEX_GIVE(bus->mutex, ret);
    return ret;
}

esp_err_t i2c_bus_write_reg16(i2c_bus_device_handle_t dev_handle, uint16_t mem_address,
                              size_t data_len, const uint8_t *data)
{
    I2C_BUS_CHECK(dev_handle && data, "param null", ESP_ERR_INVALID_ARG);
    i2c_bus_device_t *dev = (i2c_bus_device_t *)dev_handle;
    i2c_bus_t *bus = dev->i2c_bus;
    I2C_BUS_MUTEX_TAKE(bus->mutex, ESP_ERR_TIMEOUT);

    esp_err_t ret;
    if (mem_address != 0xFFFF) {
        // 组装 [mem_high, mem_low, data...]
        uint8_t *tmp = malloc(data_len + 2);
        if (!tmp) {
            I2C_BUS_MUTEX_GIVE(bus->mutex, ESP_ERR_NO_MEM);
            return ESP_ERR_NO_MEM;
        }
        tmp[0] = (mem_address >> 8) & 0xFF;
        tmp[1] = mem_address & 0xFF;
        memcpy(&tmp[2], data, data_len);

        ret = i2c_master_write_to_device(bus->i2c_port, dev->dev_addr, tmp, data_len + 2, pdMS_TO_TICKS(100));
        free(tmp);
    } else {
        ret = i2c_master_write_to_device(bus->i2c_port, dev->dev_addr, data, data_len, pdMS_TO_TICKS(100));
    }

    I2C_BUS_MUTEX_GIVE(bus->mutex, ret);
    return ret;
}
