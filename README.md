# 🐕 IWDG module

This repository provides the **MCAL (Microcontroller Abstraction Layer)** driver for the **Independent Watchdog** (`IWDG`) used in **STM32 microcontrollers**.  
It offers a simple and safe interface to start, configure and refresh the watchdog without dealing with prescalers, reload values or register update flags.

Each STM32 family is supported in a **dedicated branch** of this repository:
- `STM32G4`
- `STM32U5`
- `STM32L4`
- `STM32H5`
- and others as needed.

---

## 📘 Overview

The **IWDG MCAL driver** abstracts the STM32 independent watchdog into a time-based interface.  
The user configures the watchdog in **milliseconds**, the module does the rest:
- Prescaler (4 - 1024) and reload value calculation from the required timeout and the LSI frequency  
- Optional **window** - refresh is allowed only within the last part of the timeout  
- Optional **early wakeup interrupt** - user callback called before the watchdog resets the MCU  
- Automatic refresh from `Iwdg_Task()` or manual refresh by `Iwdg_Set_Refresh()`  
- Watchdog state and achieved timeout readout  

Register write access (key register), register update flags and read-back verification of every configuration write are handled **internally** by the module.

> ✅ The user **does not need to include or use** any additional modules such as RCC or NVIC drivers.  
> LSI frequency is read from the RCC module and the early wakeup interrupt is configured through the NVIC module internally.

> ⚠️ Once started, the watchdog **can not be stopped** (hardware limitation). It is stopped only by a system reset,
> unless the hardware watchdog mode is selected in the option bytes. `Iwdg_Deinit()` only disables the interrupt
> and the automatic refresh - the application has to keep refreshing the watchdog.

---

## 🧩 Architecture

The architecture follows the standard MCAL layering used across all STM32 MCAL repositories:

```
┌────────────────────────────┐
│        Application         │
└────────────┬───────────────┘
             │
┌────────────▼───────────────┐
│           HAL              │
│(Hardware Abstraction Layer)│
└────────────┬───────────────┘
┌────────────▼───────────────┐
│       MCAL - Iwdg          │
│       ├── Iwdg_Port.h      │  
│       ├── Iwdg_Types.h     │  
│       └── Iwdg.c/.h        │  
└────────────┬───────────────┘
             │
┌────────────▼───────────────┐
│           RAL              │
│(Register Abstraction Layer)│
└────────────────────────────┘
```

---

## 🧠 Usage Guidelines

The user shall **only** interact with the following two public headers:

| File | Purpose |
|------|----------|
| `Iwdg_Port.h` | Contains all public API functions (init, refresh, state and timeout readout, default configuration) |
| `Iwdg_Types.h` | Contains type definitions used in the API (configuration structure, time type, callback type, request states) |

Everything else — prescaler tables, helper functions, interrupt handler — is **internal** and must not be accessed directly.

### Public API

| Function | Description |
|----------|-------------|
| `Iwdg_Get_ModuleVersion()` | Returns module SW version |
| `Iwdg_Init( config )` | Calculates the configuration, starts and configures the watchdog |
| `Iwdg_Deinit()` | Disables early wakeup interrupt and automatic refresh (watchdog keeps running) |
| `Iwdg_Task()` | Refreshes the watchdog automatically (only when the window is not used) |
| `Iwdg_Get_DefaultConfig( config )` | Fills default configuration: timeout 1 s, no window, no early wakeup interrupt |
| `Iwdg_Get_State( state )` | Returns `IWDG_FUNCTION_ACTIVE` if the watchdog is running |
| `Iwdg_Get_Timeout( timeout )` | Returns achieved timeout in ms calculated from the registers |
| `Iwdg_Set_Refresh()` | Refreshes (reloads) the watchdog counter |

### Configuration

| Item | Description |
|------|-------------|
| `Timeout` | Required timeout in ms. With 32 kHz LSI the range is approx. 1 ms - 131 s. |
| `Window` | Length of the window before timeout in ms, when refresh is allowed. `IWDG_TIME_UNUSED` disables the window. Must be shorter than `Timeout`. |
| `EarlyWakeup` | Time before timeout in ms, when the early wakeup interrupt is triggered. `IWDG_TIME_UNUSED` disables the interrupt. Must be shorter than `Timeout`. |
| `EarlyWakeupIsr` | User callback of the early wakeup interrupt, can be `NULL` |
| `IrqPriority` | Early wakeup interrupt priority (0 - highest) |

Invalid configuration (timeout out of range, window or early wakeup not shorter than timeout) is rejected by `Iwdg_Init()` **before** any register is written.

### Notes

- **Window mode** - refresh before the window resets the MCU immediately (hardware behavior), it can not be reported by software. `Iwdg_Task()` does not refresh in window mode, the application has to call `Iwdg_Set_Refresh()` inside the window.
- **Hardware watchdog mode** - if `Iwdg_Get_State()` returns `IWDG_FUNCTION_ACTIVE` before `Iwdg_Init()` after reset, the watchdog was started by the option bytes.
- **Running state (STM32U5)** - STM32U5 IWDG has no running flag. The module reports the watchdog as running, if it was started by `Iwdg_Init()` since the last reset, or if the hardware watchdog mode is selected by the option bytes (`FLASH_OPTR.IWDG_SW` cleared).
- **LSI prescaler (STM32U5)** - watchdog timing uses the LSI frequency reported by the RCC module (LSI divided by `RCC_BDCR.LSIPREDIV`, if the divider is selected).
- **Reset cause** - IWDG reset can be detected after startup by `Rcc_Get_ResetSource( RCC_RESET_SRC_IWDG, &flag )` and cleared by `Rcc_Set_ResetSourceClear()` in the RCC module.
- **Timeout accuracy** - LSI frequency varies with temperature and voltage (see datasheet), the real timeout differs from the calculated one accordingly.

---

## ⚙️ Typical Usage Example

```c
#include "Iwdg_Port.h"

static void App_WatchdogWarning( void )
{
    // Last chance to store diagnostic data before the watchdog reset
}

int main(void)
{
    iwdg_Config_t       iwdgConfig;
    iwdg_Time_ms_t      achievedTimeout = 0u;
    iwdg_RequestState_t iwdgState       = IWDG_REQUEST_ERROR;

    iwdgState = Iwdg_Get_DefaultConfig( &iwdgConfig );

    iwdgConfig.Timeout        = 500u;   // Reset after 500 ms without refresh
    iwdgConfig.EarlyWakeup    = 50u;    // Callback 50 ms before reset
    iwdgConfig.EarlyWakeupIsr = App_WatchdogWarning;
    iwdgConfig.IrqPriority    = 0u;

    if( IWDG_REQUEST_OK == iwdgState )
    {
        iwdgState = Iwdg_Init( &iwdgConfig );
    }

    if( IWDG_REQUEST_OK == iwdgState )
    {
        iwdgState = Iwdg_Get_Timeout( &achievedTimeout );
    }

    while (1)
    {
        // Refreshes the watchdog automatically (no window configured)
        Iwdg_Task();
    }
}
```

---

## 🧾 Branching Strategy

Each STM32 family has its own branch:

| Branch | Description |
|--------|--------------|
| `STM32G4` | MCAL driver for STM32G4 family |
| `STM32U5` | MCAL driver for STM32U5 family |
| `STM32L4` | MCAL driver for STM32L4 family |
| `STM32H5` | MCAL driver for STM32H5 family |

These branches contain family-specific register definitions, prescaler ranges and RCC/NVIC bindings while maintaining a common interface.

---

## 🧩 Dependencies

- **Rcc_Lib** – LSI clock frequency (`RCC_PERIPH_IWDG`)
- **Nvic_Lib** – Early wakeup interrupt handling
- **RAL (Register Abstraction Layer)** – Used internally to access low-level registers  

All mandatory RCC and NVIC configurations are handled internally.

---

## 🧱 Example Directory Structure

```
Iwdg/
├── Iwdg_Port.h
├── Iwdg_Types.h
├── Iwdg.c
├── Iwdg.h
├── CMakeLists.txt
├── LICENSE.md
└── README.md
```

---

## 🛠 CMake Integration

1. Include `Iwdg_Lib` in your CMake library.
2. Include `Iwdg_Port.h` in your project.
3. Link against the Iwdg module implementation files.
4. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** — [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.

---

## 🌐 Useful Links

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Azure DevOps](https://azure.microsoft.com/en-us/services/devops/)
- [Embedbits Github](https://github.com/Embedbits)
- [CC BY-NC 4.0 License](https://creativecommons.org/licenses/by-nc/4.0/)
