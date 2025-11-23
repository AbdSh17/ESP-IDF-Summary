# ESP32-S3 I2C Summary (ESP-IDF New Driver)

> Based on ESP‑IDF I2C docs for **ESP32‑S3** (new `i2c_master.h` / `i2c_slave.h` drivers, ESP‑IDF 5.x).

---

## 1. Overview

- **Protocol**: Serial, synchronous, multi‑device, half‑duplex bus over two open‑drain lines:
  - `SDA` – data
  - `SCL` – clock
- **Controllers**: ESP32‑S3 has **2 I²C controllers (ports)**; each can be **master or slave** (not at the same time).
- **Addressing**:
  - 7‑bit and 10‑bit slave addresses supported.
- **Speeds**:
  - Standard‑mode (Sm): up to **100 kHz**
  - Fast‑mode (Fm): up to **400 kHz**
- **Limit**: In **master mode**, SCL **must not exceed 400 kHz**.

---

## 2. Electrical Notes & Pull‑Ups

- Both lines are **open‑drain**, require **pull‑up resistors** to 3.3 V.
- Recommended pull‑up range: **1 kΩ – 10 kΩ**.
  - Typical: **2 kΩ – 5 kΩ**.
  - Higher speed ⇒ **smaller resistor** (but not < 1 kΩ).
  - Larger resistor ⇒ slower edges, lower maximum stable frequency.
- Internal pull‑ups can be enabled by the driver, but **external pull‑ups are strongly recommended**, especially at higher speeds or with multiple devices.

---

## 3. Driver Variants & Headers

### 3.1 Headers

- **Legacy driver** (deprecated):
  - `#include "driver/i2c.h"`
- **New master driver**:
  - `#include "driver/i2c_master.h"`
- **New slave driver (v2.0)**:
  - `#include "driver/i2c_slave.h"`
- Shared public types:
  - `i2c_types.h`  (new common types)
  - `i2c_types_legacy.h` (legacy‑only types)

### 3.2 Coexistence Rules

- **Legacy and new drivers cannot coexist**:
  - Use **either** `i2c.h` **or** (`i2c_master.h` + `i2c_slave.h`), not both in the same project/SoC instance.
- Legacy driver is **deprecated** and will be removed in future ESP‑IDF.
- New **slave v2** driver is the recommended slave implementation; enable via Kconfig option `CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2` (if needed).

---

## 4. Clock Source Configuration

`i2c_clock_source_t`:

- `I2C_CLK_SRC_DEFAULT`
- `I2C_CLK_SRC_XTAL` – external crystal
- `I2C_CLK_SRC_RC_FAST` – internal 20 MHz RC

Notes:

- Clock source affects **power consumption** and **timing resolution**.
- For low‑power, `XTAL` is often preferred when it still gives enough resolution for your I²C speed.

---

## 5. Functional Overview (New Driver)

The I²C driver provides:

1. **Resource allocation**
   - Create / destroy **master bus** and **devices**.
   - Create / destroy **slave device**.
2. **Master controller**
   - Write / read / write‑then‑read (combined).
   - Device probe.
   - Optional async operation via callbacks.
3. **Slave controller**
   - Buffered transmit (TX FIFO).
   - Buffered receive (RX FIFO).
   - Optional RAM access mode.
   - Async receive via callbacks.
4. **Power management**
   - Behavior depends on chosen clock source and Kconfig options.
5. **IRAM safety**
   - Optional IRAM‑safe ISR mode for use when flash cache is disabled.
6. **Thread safety**
   - Bus/device factory functions are thread‑safe.
   - Most other APIs are **not** thread‑safe; user must guard them.

---

## 6. Resource Allocation (Handles & Configs)

The driver uses **handles** for resources.

### 6.1 Master Bus Config — `i2c_master_bus_config_t`

Key fields:

- `i2c_port` – physical I²C controller (e.g. `I2C_NUM_0` / `I2C_NUM_1`)
- `sda_io_num` – SDA GPIO
- `scl_io_num` – SCL GPIO
- `clk_source` – clock source (see section 4)
- `glitch_ignore_cnt`
  - Glitch filter threshold on the lines (pulses shorter than this are ignored).
  - Typical value: **7**.
- `intr_priority`
  - Interrupt priority (1–3) if non‑zero.
  - If `0`, driver chooses a low/medium priority automatically.
  - Once set, priority cannot change until bus is deleted.
- `trans_queue_depth`
  - Depth of internal transfer queue for **asynchronous** transactions.
- `flags.enable_internal_pullup`
  - Enable internal pull‑ups (still recommended to use externals).
- `flags.allow_pd`
  - Allow controller power‑down during light sleep (uses extra RAM internally).

### 6.2 Creating a Master Bus — `i2c_new_master_bus()`

```c
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *bus_cfg,
                             i2c_master_bus_handle_t *ret_bus);
```

- Allocates and initializes a master bus.
- Returns standard `esp_err_t`:
  - `ESP_OK` – success
  - `ESP_ERR_INVALID_ARG`, `ESP_ERR_NO_MEM`, `ESP_ERR_NOT_FOUND` (no free bus)

### 6.3 Master Device Config — `i2c_device_config_t`

Key fields:

- `dev_addr_length`
  - `I2C_ADDR_BIT_LEN_7` or `I2C_ADDR_BIT_LEN_10`
- `device_address`
  - Raw 7‑bit / 10‑bit address **without R/W bit**, e.g. `0x28`.
- `scl_speed_hz`
  - Per‑device SCL frequency (≤ 400 kHz).
- `scl_wait_us`
  - Extra SCL wait time in microseconds (0 for default), relevant for clock stretching / slow slaves.

### 6.4 Adding a Master Device — `i2c_master_bus_add_device()`

```c
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
                                    const i2c_device_config_t *dev_cfg,
                                    i2c_master_dev_handle_t *ret_dev);
```

- Creates a device on the bus and returns a `i2c_master_dev_handle_t`.
- Errors: invalid args, no memory.

### 6.5 Removing Devices & Buses

- Remove device:
  ```c
  esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t dev);
  ```
- Delete bus (after all devices removed):
  ```c
  esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus);
  ```

---

## 7. Slave Device Allocation — `i2c_slave_config_t`

Key fields:

- `i2c_port` – controller port used as slave
- `sda_io_num` – SDA GPIO
- `scl_io_num` – SCL GPIO
- `clk_source` – clock source
- `send_buf_depth` – depth of TX buffer (FIFO used to answer master reads)
- `slave_addr` – slave address
- `intr_priority` – interrupt priority (similar rule as master)
- `addr_bit_len` – `I2C_ADDR_BIT_LEN_7` or `I2C_ADDR_BIT_LEN_10`
- `flags.access_ram_en`
  - Enable “RAM mode”: use FIFO as RAM accessible via address offsets.

APIs:

- Create / init slave:
  ```c
  esp_err_t i2c_new_slave_device(const i2c_slave_config_t *cfg,
                                 i2c_slave_dev_handle_t *ret_handle);
  ```
- Delete slave:
  ```c
  esp_err_t i2c_del_slave_device(i2c_slave_dev_handle_t dev);
  ```

---

## 8. Master Controller Operations

Once you have a `i2c_master_dev_handle_t`, you can perform standard transactions.

### 8.1 Master Write — `i2c_master_transmit()`

```c
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev,
                              const uint8_t *write_buf,
                              size_t write_size,
                              int xfer_timeout_ms);
```

- Performs a single **write** transaction:  
  `START → addr+W → data… → STOP`.
- Blocks until finished or timeout, **unless** callbacks are registered (then it becomes async, see section 10).

### 8.2 Master Read — `i2c_master_receive()`

```c
esp_err_t i2c_master_receive(i2c_master_dev_handle_t dev,
                             uint8_t *read_buf,
                             size_t read_size,
                             int xfer_timeout_ms);
```

- Performs a single **read** transaction:  
  `START → addr+R → data… → STOP`.

### 8.3 Master Write + Read — `i2c_master_transmit_receive()`

```c
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t dev,
                                      const uint8_t *write_buf,
                                      size_t write_size,
                                      uint8_t *read_buf,
                                      size_t read_size,
                                      int xfer_timeout_ms);
```

- Combined transaction (register access pattern):  
  `START → addr+W → reg… → RESTART → addr+R → data… → STOP`.
- Useful for sensors like BNO055, IMUs, etc.

### 8.4 Master Probe — `i2c_master_probe()`

```c
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus,
                           uint16_t address,
                           int xfer_timeout_ms);
```

- Sends an address frame and checks for ACK.
- Returns `ESP_OK` if there is a device responding at that address.

---

## 9. Slave Controller Operations

### 9.1 Slave Transmit (Master Reads) — `i2c_slave_transmit()`

```c
esp_err_t i2c_slave_transmit(i2c_slave_dev_handle_t dev,
                             const uint8_t *data,
                             size_t data_len,
                             int timeout_ms);
```

- Enqueues data in the **TX buffer**.
- Master reads will de‑queue bytes in FIFO order.

### 9.2 Slave Receive (Master Writes) — `i2c_slave_receive()`

```c
esp_err_t i2c_slave_receive(i2c_slave_dev_handle_t dev,
                            uint8_t *data,
                            size_t data_len);
```

- Non‑blocking read from RX buffer (receive FIFO).
- Usually paired with **callbacks** and a queue:
  - Register callbacks via `i2c_slave_register_event_callbacks()`.
  - Use `on_recv_done` to know when a full message is received.

### 9.3 Slave RAM Access

If `access_ram_en` is enabled in config:

- Use **RAM mode** helpers:
  - `i2c_slave_read_ram()`
  - `i2c_slave_write_ram()`
- This lets the master read/write data at specific RAM offsets exposed by the slave, useful for custom protocols.

---

## 10. Event Callbacks & Asynchronous Operation

### 10.1 Master Callbacks — `i2c_master_register_event_callbacks()`

- Register callbacks in `i2c_master_event_callbacks_t`, especially:
  - `on_recv_done` – called when a master transaction finishes.
- When a callback is registered:
  - Transactions (e.g. `i2c_master_transmit`) can become **asynchronous**.
  - Function returns early; completion is signaled via callback.
  - Your buffers must remain valid until the transaction is done.
- Only **one device per bus** can use asynchronous operation.

> The async master API is marked **experimental**; very large async transactions can cause memory issues.

### 10.2 Slave Callbacks — `i2c_slave_register_event_callbacks()`

- Register callbacks in `i2c_slave_event_callbacks_t`, especially:
  - `on_recv_done` – called when a receive has completed.
- Callbacks run in **ISR context**:
  - Only use `FromISR` FreeRTOS APIs.
  - Return `true` if a higher‑priority task was woken.

Example ISR pattern:

```c
static IRAM_ATTR bool i2c_slave_rx_done_callback(
    i2c_slave_dev_handle_t dev,
    const i2c_slave_rx_done_event_data_t *edata,
    void *user_data)
{
    BaseType_t hp_task_woken = pdFALSE;
    QueueHandle_t q = (QueueHandle_t)user_data;
    xQueueSendFromISR(q, edata, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}
```

---

## 11. Power Management

- Using `I2C_CLK_SRC_XTAL` avoids installing a power‑management lock; suitable for low‑power cases as long as timing is acceptable.
- `flags.allow_pd` in bus/slave config lets the controller power‑down in light sleep (with cost in RAM).

---

## 12. IRAM‑Safe Mode

Kconfig option: `CONFIG_I2C_ISR_IRAM_SAFE`

When enabled:

1. I²C ISR can run even when flash **cache is disabled** (e.g., during flash erase/write).
2. All ISR‑used functions are linked into **IRAM**.
3. Driver objects are placed into **DRAM** (not PSRAM).

Trade‑off:

- Better real‑time behavior, but **higher IRAM usage**.

---

## 13. Thread Safety

- **Thread‑safe APIs**:
  - Factory functions: `i2c_new_master_bus()`, `i2c_new_slave_device()`.
- **Not thread‑safe**:
  - Most other public I²C APIs (transmit/receive, callbacks, etc.).
- If you need to call them from multiple tasks:
  - Protect accesses with your own mutex/lock around a bus or device handle.

---

## 14. Kconfig Options (Key Ones)

- `CONFIG_I2C_ISR_IRAM_SAFE`
  - Enable IRAM‑safe ISR mode (see section 12).
- `CONFIG_I2C_ENABLE_DEBUG_LOG`
  - Enable debug logging (increases binary size).
- `CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2`
  - Use the new slave v2 driver (recommended).

---

## 15. Minimal Usage Templates

### 15.1 Master (New Driver) — Basic Pattern

```c
#include "driver/i2c_master.h"

#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_IO     GPIO_NUM_8
#define I2C_SCL_IO     GPIO_NUM_9

void app_main(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = 0x28,      // Example address
        .scl_speed_hz    = 400000,
    };

    i2c_master_dev_handle_t dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));

    uint8_t reg = 0x00;
    uint8_t val = 0;

    // Typical register read: write reg, then read 1 byte
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 1000));

    // Cleanup when done
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
}
```

### 15.2 Slave (New Driver) — Basic Pattern

```c
#include "driver/i2c_slave.h"

#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_IO     GPIO_NUM_8
#define I2C_SCL_IO     GPIO_NUM_9

void app_main(void)
{
    i2c_slave_config_t slv_cfg = {
        .i2c_port       = I2C_PORT,
        .sda_io_num     = I2C_SDA_IO,
        .scl_io_num     = I2C_SCL_IO,
        .clk_source     = I2C_CLK_SRC_DEFAULT,
        .addr_bit_len   = I2C_ADDR_BIT_LEN_7,
        .slave_addr     = 0x28,
        .send_buf_depth = 256,
    };

    i2c_slave_dev_handle_t slv;
    ESP_ERROR_CHECK(i2c_new_slave_device(&slv_cfg, &slv));

    uint8_t tx_data[4] = {1, 2, 3, 4};
    ESP_ERROR_CHECK(i2c_slave_transmit(slv, tx_data, sizeof(tx_data), 1000));

    // Receive example would use callbacks + i2c_slave_receive()

    ESP_ERROR_CHECK(i2c_del_slave_device(slv));
}
```

---

## 16. Legacy Driver Note

- Old driver (`driver/i2c.h`) uses:
  - `i2c_config_t`, `i2c_param_config()`, `i2c_driver_install()`.
  - Command links (`i2c_cmd_link_create()`, `i2c_master_start()`, `i2c_master_write_byte()`, etc.).
- These APIs are **deprecated** in favor of `i2c_master.h` / `i2c_slave.h`.
- For new projects on ESP32‑S3, use **only the new driver**.

---

**End of I²C Summary**
