# Analog pins
this file will be to explain how to read analog in ESP-IDF from the ADC Channel 

## Table of Contents
[What is an ADC](#what-is-an-adc)
[ESP32-S3 ADC Architecture](#esp32-s3-adc-architecture)
[ADC One Shot](#adc-one-shot)
[ADC Continues](#adc-continues)
## What is an ADC

An ADC (Analog-to-Digital Converter) is the part of the microcontroller that measures analog voltages (continuous signals like 0 – 3.3 V) and converts them into a digital number the CPU can read. </br></br>

for this section and [ESP32-S3 ADC Architecture](#esp32-s3-adc-architecture) the reference is **chatGPT**

---

## ESP32-S3 ADC Architecture

| ADC Unit | Channels                | Typical Usage                                                                |
| -------- | ----------------------- | ---------------------------------------------------------------------------- |
| **ADC1** | 10 channels (CH0 → CH9) | Used for most analog inputs                                                  |
| **ADC2** | 10 channels (CH0 → CH9) | Can be used, but shared with Wi-Fi (not recommended for simple sensor reads) |

### Unit Mapping
there's mainly two ADC Units in ESP32-S3 N16R8

| ADC Unit | Channels                | Typical Usage                                                                |
| -------- | ----------------------- | ---------------------------------------------------------------------------- |
| **ADC1** | 10 channels (CH0 → CH9) | Used for most analog inputs                                                  |
| **ADC2** | 10 channels (CH0 → CH9) | Can be used, but shared with Wi-Fi (not recommended for simple sensor reads) |


### Channel Mapping
| ADC1 Channel | GPIO      | 
| ------------ | --------- | 
| CH0          | GPIO1     | 
| CH1          | GPIO2     |
...|...|
| CH9          | GPIO10    |

**Important: Not all dev boards break out every GPIO.**</br> **for example:** some pins might be used by PSRAM, flash, or peripherals.
</br>Always check your DevKitC-1 pinout diagram to make sure the pin is free.

## ADC One-Shot
**ADC One-Shot:** a mode in which the analog-to-digital converter performs a **single conversion** only **when triggered by software**, returning one digital sample per request rather than sampling continuously.
</br>
in simple words, it will read the analog value just when you write the thing in your code, so the ADC won't work unless it reads the code, which will be lighter to the CPU </br>
the other type is called [ADC Continues](#adc-continues) </br></br>

### ADC in ESP-IDF

**REF.: [ESP-IDF One-Shot](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_oneshot.html)**
</br></br>

The ADC oneshot mode driver is implemented based on ESP32 SAR ADC module. Different ESP chips might have different numbers of independent ADCs. </br>

**an ADC instance is represented by `adc_oneshot_unit_handle_t`.** </br>
ex: `adc_oneshot_unit_handle_t adc1;`

to setup the configarations for this adc instance you open `adc_oneshot_unit_init_cfg_t`

### Unit configaration

#### `unit_id`
`unit_id` selects the ADC UNIT. </br></br>

#### `clk_src`
`clk_src` selects the source clock of the ADC. If set to 0, the driver will fall back to using a default clock source.</br>

inside our ESP32-s3 n16r8 there's too many clocks
| Clock Source                       | Typical Frequency   | Description                                                                               |
| ---------------------------------- | ------------------- | ----------------------------------------------------------------------------------------- |
| **XTAL** (Crystal Oscillator)      | 40 MHz              | The main stable reference clock; almost all timing derives from this.                     |
| **PLL (Phase-Locked Loop)**        | 80 MHz – 240 MHz    | Multiplies the 40 MHz XTAL to generate higher-speed clocks for CPU, ADC, and peripherals. |
| **APB Clock (Peripheral Bus)**     | ~80 MHz             | Feeds most peripherals like UART, I2C, ADC (default digital clock source).                |
| **RTC 8 MHz Clock**                | 8 MHz (internal RC) | Low-power clock used when the main system sleeps.                                         |
| **Internal RC (150 kHz / 90 kHz)** | ~90 kHz             | Ultra-low-power reference for deep sleep timers.                                          |
| **USB / Wi-Fi / Bluetooth Clocks** | Variable            | Generated internally for high-speed communication modules.                                |

so you can choose any clock you want </br>
**Most of the time**, you just keep the default (ADC_DIGI_CLK_SRC_DEFAULT → APB clock), but for advanced control or low-power designs, you can pick another.

#### Why APB
##### What is APB
- APB = Advanced Peripheral Bus

- It’s the main peripheral clock domain used by timers, UART, I²C, SPI, and ADC.

- On the ESP32-S3, it typically runs at 80 MHz.
So the APB clock is fast, always-on when the CPU is active, and synchronized with the system clock
##### Why the ADC defaults to APB clock
TBH, You don't wanna go there, need to review communication class from scatch until go to ADC, Please don't ask tooooo much


#### `ulp_mode`
`ulp_mode` sets if the ADC will be working under ULP mode

##### What is ULP
ULP (Ultra Low Power coprocessor) is a small, independent processor built into the ESP32-S3 that runs while the main CPU cores are asleep. </br>
It’s designed for low-energy background tasks, like
- reading ADC sensors,

- monitoring GPIO inputs,

- waking the main CPU when certain conditions are met.

</br>
The ULP runs from the **RTC domain** (low-power clock, usually 8 MHz,you can see it in the table above), consumes only a few microamps, and allows the ESP32-S3 to stay in deep sleep while still “watching” the environment — making it key for ultra-low-power applications such as battery-powered sensors. </br>

**so when choosing ULP mode, the default clock will be RTC**

#### `adc_oneshot_new_unit()`
After setting up the initial configurations for the ADC, call `adc_oneshot_new_unit()` with the prepared `adc_oneshot_unit_init_cfg_t`. This function will return an ADC unit handle if the allocation is **successful**.
</br>
**EX:** </br>


```C 
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};
ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
```


### Channel Configuration

#### atten
ADC attenuation. </br>
It sets the input voltage range that the ADC can measure.

#### bitwidth
the bitwidth of the raw conversion result. </br>
how many bits the ADC will read, 9b -> 0-512, 12b -> 0-4096 
...

#### Applying the Configuration
To make these settings take effect, call `adc_oneshot_config_channel()` with the above configuration structure. You should specify an ADC channel to be configured as well. Function `adc_oneshot_config_channel()` can be called multiple times to configure different ADC channels. The Driver will save each of these channel configurations internally

### Full Example
```C
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_cfg, &adc1);

    adc_oneshot_chan_cfg_t cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11 // 0–3.3V range
    };
    adc_oneshot_config_channel(adc1, POT_PIN, &cfg);

```

### Read Conversion Result
`adc_oneshot_read()`

### More
for further info about power, safety and more, check the resource pleassssssse.

---



## ADC Continues
TBH, i Don;t think I'll continue summarizing everything so here's the source
[ESP-IDF ADC Continues](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_continuous.html)




