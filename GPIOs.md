# ESP32-S3 GPIO Summary

## 1. GPIO Modes
Each pin can operate as:

- **Input** → `GPIO_MODE_INPUT`  
- **Output** → `GPIO_MODE_OUTPUT`  
- **Input + Output** → `GPIO_MODE_INPUT_OUTPUT`  
- **Open-Drain Output** → `GPIO_MODE_OUTPUT_OD`  
- **Input + Open-Drain** → `GPIO_MODE_INPUT_OUTPUT_OD`  
- **Disabled** → `GPIO_MODE_DISABLE`

---

## 2. Internal Pull Resistors
Most GPIOs support:

- **Pull-Up** → `pull_up_en = true`  
- **Pull-Down** → `pull_down_en = true`  
- **None**

Used for buttons, switches, sensors that need line biasing.

---

## 3. Interrupt Configuration
GPIO interrupts can trigger on:

- **Rising edge** → `GPIO_INTR_POSEDGE`  
- **Falling edge** → `GPIO_INTR_NEGEDGE`  
- **Any edge** → `GPIO_INTR_ANYEDGE`  
- **Low level** → `GPIO_INTR_LOW_LEVEL`  
- **High level** → `GPIO_INTR_HIGH_LEVEL`  
- **Disabled**

Setup:
```c
gpio_install_isr_service(0);
gpio_isr_handler_add(pin, isr_handler, arg);
```

---

## 4. Drive Strength
Controls how strong the output can source/sink current:

- `GPIO_DRIVE_CAP_0` (weakest)  
- `GPIO_DRIVE_CAP_1`  
- `GPIO_DRIVE_CAP_2`  
- `GPIO_DRIVE_CAP_3` (strongest)

Set with:
```c
gpio_set_drive_capability(pin, cap);
```

---

## 5. Pin Hold / Power-Down Behavior
Freeze a pin’s state across deep sleep:

```c
gpio_hold_en(pin);
gpio_deep_sleep_hold_en();
```

Release:
```c
gpio_hold_dis(pin);
```

---

## 6. I/O Multiplexing (Alternate Functions)
GPIOs can be routed to internal peripherals:

- ADC  
- PWM (LEDC / MCPWM)  
- UART  
- SPI  
- I2C  
- RMT  
- PCNT  
- Touch sensor  

ESP32-S3 uses flexible pin-matrix routing.

---

## 7. Input Filtering / Debounce
Hardware filtering is basic; you can disable interrupts or rely on software:

```c
gpio_intr_disable(pin);
```

For real debouncing, use a timer or software filter.

---

## 8. Open-Drain Mode
Used for:

- I²C  
- Shared buses  
- Wired-AND signals  

```c
.mode = GPIO_MODE_OUTPUT_OD
```

Pin can only pull **LOW**; HIGH comes from a pull-up resistor.

---

## 9. Reading & Writing
Write:
```c
gpio_set_level(pin, 1);
```

Read:
```c
int val = gpio_get_level(pin);
```

---

## 10. Pin Mask Configuration
Configure multiple pins at once:

```c
.pin_bit_mask = (1ULL << GPIO_NUM_4) | (1ULL << GPIO_NUM_5)
```

---

## 11. Strapping Pins (Boot Pins)
Some pins influence boot behavior (0, 3, 45, 46).  
Avoid changing their pull configuration unless necessary.

---

## 12. Full GPIO Config Template

```c
gpio_config_t io_conf = {
    .pin_bit_mask = 1ULL << GPIO_NUM_4,
    .mode = GPIO_MODE_INPUT_OUTPUT,
    .pull_up_en = true,
    .pull_down_en = false,
    .intr_type = GPIO_INTR_ANYEDGE,
};
gpio_config(&io_conf);
```

---

## Summary
ESP32-S3 GPIOs are highly configurable: you can set mode, pulls, interrupts, drive strength, open-drain behavior, deep-sleep hold, and remap pins to peripherals. This flexibility makes them suitable for sensors, motors, communication buses, and real-time control.
