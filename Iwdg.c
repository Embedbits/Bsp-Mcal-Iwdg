/**
 * \author Mr.Nobody
 * \file Iwdg.c
 * \ingroup Iwdg
 * \brief Iwdg module common functionality
 *
 * Independent watchdog (IWDG) is clocked by Low Speed Internal (LSI) oscillator.
 * Prescaler and reload value are calculated from required timeout and LSI
 * frequency provided by RCC module. Once started, the watchdog can not be
 * stopped by software (hardware limitation), it is stopped only by system reset
 * (except hardware watchdog mode selected by option bytes).
 *
 * STM32F4 IWDG provides key, prescaler, reload and status registers only - no
 * window, no early wakeup interrupt and no watchdog running flag. Window and
 * early wakeup configuration is therefore rejected, running state is kept by
 * the module (start key written) together with hardware watchdog option byte.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Iwdg.h"                           /* Self include                   */
#include "Iwdg_Port.h"                      /* Own port file include          */
#include "Iwdg_Types.h"                     /* Module types definitions       */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
#include "Stm32_iwdg.h"                     /* IWDG peripheral functionality  */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

static iwdg_RequestState_t  Iwdg_Get_LsiClk          ( rcc_FreqHz_t * const lsiClk );
static iwdg_FunctionState_t Iwdg_Get_RunningState    ( void );
static iwdg_RequestState_t  Iwdg_Calc_Ticks          ( iwdg_Time_ms_t time, rcc_FreqHz_t lsiClk, iwdg_Divider_t divider, iwdg_Ticks_t * const ticks );
static iwdg_RequestState_t  Iwdg_Calc_Prescaler      ( iwdg_Time_ms_t timeout, rcc_FreqHz_t lsiClk, iwdg_Prescaler_t * const prescId, iwdg_Ticks_t * const ticks );

static iwdg_RequestState_t  Iwdg_Check_UpdateDone    ( void );
static iwdg_RequestState_t  Iwdg_Check_ApbClk        ( void );

static void                 Iwdg_Set_Started         ( void );
static iwdg_RequestState_t  Iwdg_Set_Prescaler       ( iwdg_Prescaler_t prescId );
static iwdg_RequestState_t  Iwdg_Set_Reload          ( uint32_t reloadVal );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define IWDG_MAJOR_VERSION              ( 1u )

/** Value of minor version of SW module */
#define IWDG_MINOR_VERSION              ( 0u )

/** Value of patch version of SW module */
#define IWDG_PATCH_VERSION              ( 0u )

/** Maximal wait time for configuration request confirmation */
#define IWDG_TIMEOUT_RAW                ( 0x84FCB )

/** Number of milliseconds in one second */
#define IWDG_MS_IN_SECOND               ( 1000u )

/** Divisor used to round integer division to the nearest value */
#define IWDG_ROUNDING_DIV               ( 2u )

/** Counter counts down to zero inclusive - number of ticks is register value + 1 */
#define IWDG_COUNTER_OFFSET             ( 1u )

/** Minimal number of watchdog counter ticks */
#define IWDG_TICKS_MIN                  ( 1u )

/** Maximal number of watchdog counter ticks (reload register width) */
#define IWDG_TICKS_MAX                  ( IWDG_RLR_RL + IWDG_COUNTER_OFFSET )

/** LL flag / state function return value signaling active state */
#define IWDG_LL_STATE_ACTIVE            ( 1u )

/** LL flag / state function return value signaling inactive state */
#define IWDG_LL_STATE_INACTIVE          ( 0u )

/** Default watchdog timeout used by \ref Iwdg_Get_DefaultConfig */
#define IWDG_DEFAULT_TIMEOUT_MS         ( 1000u )

/** Default early wakeup interrupt priority used by \ref Iwdg_Get_DefaultConfig */
#define IWDG_DEFAULT_IRQ_PRIO           ( 0u )

/** Maximal LSI (IWDG clock) frequency of STM32F4 (datasheet fLSI max) */
#define IWDG_LSI_MAX_HZ                 ( 47000u )

/** APB1 clock has to be higher than twice the IWDG clock - otherwise PVU / RVU flags are never
 *  cleared (device errata "RVU / PVU flag not cleared at low APB clock frequency") */
#define IWDG_APB_MIN_HZ                 ( 2u * IWDG_LSI_MAX_HZ )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Prescaler configuration, indexed by \ref iwdg_Prescaler_t */
static const iwdg_PrescalerConfig_t iwdg_PrescalerLut[ ] =
{
    { .RegVal = LL_IWDG_PRESCALER_4   , .Divider = 4u    }, /**< \ref IWDG_PRESCALER_DIV_4    */
    { .RegVal = LL_IWDG_PRESCALER_8   , .Divider = 8u    }, /**< \ref IWDG_PRESCALER_DIV_8    */
    { .RegVal = LL_IWDG_PRESCALER_16  , .Divider = 16u   }, /**< \ref IWDG_PRESCALER_DIV_16   */
    { .RegVal = LL_IWDG_PRESCALER_32  , .Divider = 32u   }, /**< \ref IWDG_PRESCALER_DIV_32   */
    { .RegVal = LL_IWDG_PRESCALER_64  , .Divider = 64u   }, /**< \ref IWDG_PRESCALER_DIV_64   */
    { .RegVal = LL_IWDG_PRESCALER_128 , .Divider = 128u  }, /**< \ref IWDG_PRESCALER_DIV_128  */
    { .RegVal = LL_IWDG_PRESCALER_256 , .Divider = 256u  }, /**< \ref IWDG_PRESCALER_DIV_256  */
};

_Static_assert( IWDG_PRESCALER_CNT == ( sizeof( iwdg_PrescalerLut ) / sizeof( iwdg_PrescalerConfig_t ) ), "IWDG prescaler LUT size mismatch" );

/** Automatic refresh from \ref Iwdg_Task state */
static iwdg_FunctionState_t iwdg_AutoRefreshState = IWDG_FUNCTION_INACTIVE;

/** Watchdog started by the module (start key written). STM32F4 IWDG has no
 *  running flag, the watchdog is stopped only by system reset, which also
 *  clears this state. */
static iwdg_FunctionState_t iwdg_StartedState = IWDG_FUNCTION_INACTIVE;

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
iwdg_ModuleVersion_t Iwdg_Get_ModuleVersion( void )
{
    iwdg_ModuleVersion_t retVersion;

    retVersion.Major = IWDG_MAJOR_VERSION;
    retVersion.Minor = IWDG_MINOR_VERSION;
    retVersion.Patch = IWDG_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Initializes and starts the independent watchdog.
 *
 * Prescaler and reload value are calculated from required timeout and LSI
 * frequency (smallest prescaler with the best resolution is used). Achieved
 * timeout can be read by \ref Iwdg_Get_Timeout. Register configuration follows
 * the reference manual sequence: start (LSI is enabled by HW), write access
 * enable, prescaler, reload, wait for register update, counter reload (new
 * prescaler and reload value are used from now on, write access is disabled).
 *
 * \note  Once started, the watchdog can not be stopped. \ref Iwdg_Task refreshes
 *        the watchdog automatically.
 *
 * \note  STM32F4 IWDG has no window and no early wakeup interrupt.
 *        \ref iwdg_Config_t::EarlyWakeupIsr and \ref iwdg_Config_t::IrqPriority
 *        are not used.
 *
 * \pre   Configuration shall be valid: timeout achievable with LSI clock,
 *        window and early wakeup time \ref IWDG_TIME_UNUSED (not supported by
 *        STM32F4); otherwise \ref IWDG_REQUEST_ERROR is returned and no register
 *        is changed.
 *
 * \pre   APB1 clock shall be higher than twice the maximal LSI frequency (device errata
 *        "RVU / PVU flag not cleared at low APB clock frequency" - register updates would
 *        never finish); otherwise \ref IWDG_REQUEST_ERROR is returned, the watchdog is not
 *        started. APB1 shall not be slowed below this limit while the watchdog is used.
 *
 * \param config [in]: Pointer to IWDG configuration, see \ref iwdg_Config_t. Must not be NULL.
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
iwdg_RequestState_t Iwdg_Init( iwdg_Config_t * const config )
{
    iwdg_RequestState_t  retState     = IWDG_REQUEST_ERROR;
    rcc_FreqHz_t         lsiClk       = 0u;
    iwdg_Prescaler_t     prescId      = IWDG_PRESCALER_CNT;
    iwdg_Ticks_t         timeoutTicks = 0u;

    if( ( IWDG_NULL_PTR    != config              ) &&
        ( IWDG_TIME_UNUSED == config->Window      ) &&
        ( IWDG_TIME_UNUSED == config->EarlyWakeup )    )
    {
        /*------------- Configuration calculation and validation -------------*/
        retState = Iwdg_Get_LsiClk( &lsiClk );

        if( IWDG_REQUEST_OK == retState )
        {
            retState = Iwdg_Calc_Prescaler( config->Timeout, lsiClk, &prescId, &timeoutTicks );
        }
        else
        {
            /* LSI clock is not available, keep error */
        }

        if( IWDG_REQUEST_OK == retState )
        {
            retState = Iwdg_Check_ApbClk();
        }
        else
        {
            /* Invalid configuration, keep error */
        }

        /*--------------------- Watchdog configuration -----------------------*/
        if( IWDG_REQUEST_OK == retState )
        {
            iwdg_AutoRefreshState = IWDG_FUNCTION_INACTIVE;

            Iwdg_Set_Started();

            LL_IWDG_EnableWriteAccess( IWDG );

            retState = Iwdg_Set_Prescaler( prescId );

            if( IWDG_REQUEST_OK == retState )
            {
                retState = Iwdg_Set_Reload( timeoutTicks - IWDG_COUNTER_OFFSET );
            }
            else
            {
                /* Previous step failed */
            }

            LL_IWDG_DisableWriteAccess( IWDG );
        }
        else
        {
            /* Invalid configuration, nothing is written */
        }

        if( IWDG_REQUEST_OK == retState )
        {
            /* Counter reload - new reload value is used from now on */
            LL_IWDG_ReloadCounter( IWDG );

            iwdg_AutoRefreshState = IWDG_FUNCTION_ACTIVE;
        }
        else
        {
            /* Configuration failed */
        }
    }
    else
    {
        /* NULL configuration or window / early wakeup not supported by STM32F4 */
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes module Iwdg
 *
 * Stops automatic refresh from \ref Iwdg_Task.
 *
 * \note  Running watchdog can not be stopped (HW limitation). After Deinit the
 *        application has to refresh it by \ref Iwdg_Set_Refresh, otherwise
 *        the MCU is reset after timeout.
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
iwdg_RequestState_t Iwdg_Deinit( void )
{
    iwdg_AutoRefreshState = IWDG_FUNCTION_INACTIVE;

    return ( IWDG_REQUEST_OK );
}


/**
 * \brief Main task of module Iwdg
 *
 * Refreshes the watchdog automatically, if the module was initialized (see
 * \ref Iwdg_Init). Shall be called periodically with period shorter than
 * configured timeout. After \ref Iwdg_Deinit the task does nothing and the
 * application refreshes the watchdog by \ref Iwdg_Set_Refresh.
 */
void Iwdg_Task( void )
{
    if( IWDG_FUNCTION_ACTIVE == iwdg_AutoRefreshState )
    {
        (void)Iwdg_Set_Refresh();
    }
    else
    {
        /* Automatic refresh is not active */
    }
}


/**
 * \brief Fills configuration structure with default values.
 *
 * Default configuration: timeout 1 s, window and early wakeup interrupt unused.
 *
 * \param config [out]: Pointer to store default configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
iwdg_RequestState_t Iwdg_Get_DefaultConfig( iwdg_Config_t * const config )
{
    iwdg_RequestState_t retState = IWDG_REQUEST_ERROR;

    if( IWDG_NULL_PTR != config )
    {
        config->Timeout        = IWDG_DEFAULT_TIMEOUT_MS;
        config->Window         = IWDG_TIME_UNUSED;
        config->EarlyWakeup    = IWDG_TIME_UNUSED;
        config->EarlyWakeupIsr = IWDG_NULL_PTR;
        config->IrqPriority    = IWDG_DEFAULT_IRQ_PRIO;

        retState = IWDG_REQUEST_OK;
    }
    else
    {
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns watchdog running state.
 *
 * STM32F4 IWDG has no running flag. The watchdog is running, if it was started
 * by \ref Iwdg_Init since the last reset, or if the hardware watchdog mode is
 * selected by option bytes (WDG_SW cleared - watchdog started after reset).
 *
 * \note  Watchdog is stopped by any system reset, except hardware watchdog mode
 *        (option bytes). Active state read before \ref Iwdg_Init after reset
 *        therefore signals hardware watchdog mode.
 *
 * \param state [out]: Pointer to store watchdog state, \ref IWDG_FUNCTION_ACTIVE
 *                     if the watchdog is running. Must not be NULL.
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
iwdg_RequestState_t Iwdg_Get_State( iwdg_FunctionState_t * const state )
{
    iwdg_RequestState_t retState = IWDG_REQUEST_ERROR;

    if( IWDG_NULL_PTR != state )
    {
        *state   = Iwdg_Get_RunningState();
        retState = IWDG_REQUEST_OK;
    }
    else
    {
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns configured watchdog timeout.
 *
 * Timeout is calculated from prescaler and reload registers and LSI frequency
 * provided by RCC module (rounded to the nearest ms).
 *
 * \pre   No register update may be in progress (SR update flags cleared within
 *        timeout); otherwise \ref IWDG_REQUEST_ERROR.
 *
 * \param timeout [out]: Pointer to store timeout in ms. Must not be NULL.
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
iwdg_RequestState_t Iwdg_Get_Timeout( iwdg_Time_ms_t * const timeout )
{
    iwdg_RequestState_t retState = IWDG_REQUEST_ERROR;
    rcc_FreqHz_t        lsiClk   = 0u;
    iwdg_Prescaler_t    prescId  = IWDG_PRESCALER_CNT;

    if( IWDG_NULL_PTR != timeout )
    {
        retState = Iwdg_Get_LsiClk( &lsiClk );

        if( IWDG_REQUEST_OK == retState )
        {
            retState = Iwdg_Check_UpdateDone();
        }
        else
        {
            /* LSI clock is not available */
        }

        if( IWDG_REQUEST_OK == retState )
        {
            const uint32_t     prescReg  = LL_IWDG_GetPrescaler( IWDG );
            const iwdg_Ticks_t ticks     = LL_IWDG_GetReloadCounter( IWDG ) + IWDG_COUNTER_OFFSET;

            for( iwdg_Prescaler_t searchId = IWDG_PRESCALER_DIV_4; IWDG_PRESCALER_CNT > searchId; searchId ++ )
            {
                if( iwdg_PrescalerLut[ searchId ].RegVal == prescReg )
                {
                    prescId = searchId;
                    break;
                }
                else
                {
                    /* Continue with next prescaler */
                }
            }

            if( IWDG_PRESCALER_CNT > prescId )
            {
                const uint64_t tickTime = (uint64_t)ticks * iwdg_PrescalerLut[ prescId ].Divider * IWDG_MS_IN_SECOND;

                *timeout = (iwdg_Time_ms_t)( ( tickTime + ( lsiClk / IWDG_ROUNDING_DIV ) ) / lsiClk );

                retState = IWDG_REQUEST_OK;
            }
            else
            {
                /* Unknown prescaler register value */
                retState = IWDG_REQUEST_ERROR;
            }
        }
        else
        {
            /* Register update is still in progress */
        }
    }
    else
    {
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Refreshes (reloads) the watchdog counter.
 *
 * \note  Key register is write only, the refresh can not be verified by read-back.
 *
 * \pre   Watchdog shall be running (see \ref Iwdg_Get_State); otherwise
 *        \ref IWDG_REQUEST_ERROR and no register is changed.
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
iwdg_RequestState_t Iwdg_Set_Refresh( void )
{
    iwdg_RequestState_t        retState     = IWDG_REQUEST_ERROR;
    const iwdg_FunctionState_t runningState = Iwdg_Get_RunningState();

    if( IWDG_FUNCTION_ACTIVE == runningState )
    {
        LL_IWDG_ReloadCounter( IWDG );

        retState = IWDG_REQUEST_OK;
    }
    else
    {
        /* Watchdog is not running */
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Returns IWDG kernel clock (LSI) frequency from RCC module.
 *
 * \param lsiClk [out]: Pointer to store LSI frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
static iwdg_RequestState_t Iwdg_Get_LsiClk( rcc_FreqHz_t * const lsiClk )
{
    iwdg_RequestState_t retState = IWDG_REQUEST_ERROR;

    if( IWDG_NULL_PTR != lsiClk )
    {
        const rcc_RequestState_t rccState = Rcc_Get_PeriphClk( RCC_PERIPH_IWDG, lsiClk );

        if( ( RCC_REQUEST_OK == rccState ) &&
            ( 0u              < *lsiClk  )    )
        {
            retState = IWDG_REQUEST_OK;
        }
        else
        {
            retState = IWDG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns watchdog running state.
 *
 * Watchdog is running if it was started by the module, or if the hardware
 * watchdog mode is selected by option bytes (FLASH OPTCR WDG_SW cleared).
 *
 * \return \ref IWDG_FUNCTION_ACTIVE if the watchdog is running, otherwise
 *         \ref IWDG_FUNCTION_INACTIVE.
 */
static iwdg_FunctionState_t Iwdg_Get_RunningState( void )
{
    iwdg_FunctionState_t retState = IWDG_FUNCTION_INACTIVE;
    const uint32_t       swWdg    = READ_BIT( FLASH->OPTCR, FLASH_OPTCR_WDG_SW );

    if( ( IWDG_FUNCTION_ACTIVE == iwdg_StartedState ) ||
        ( 0u                   == swWdg             )    )
    {
        retState = IWDG_FUNCTION_ACTIVE;
    }
    else
    {
        /* Software watchdog mode and watchdog not started */
        retState = IWDG_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Calculates number of watchdog counter ticks for required time.
 *
 * \param time     [in]: Required time in ms
 * \param lsiClk   [in]: LSI frequency in Hz, must be non-zero
 * \param divider  [in]: Prescaler divider value, must be non-zero
 * \param ticks   [out]: Pointer to store number of ticks (rounded to the nearest). Must not be NULL.
 *
 * \return Returns \ref IWDG_REQUEST_OK if the number of ticks is in range
 *         \ref IWDG_TICKS_MIN - \ref IWDG_TICKS_MAX. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
static iwdg_RequestState_t Iwdg_Calc_Ticks( iwdg_Time_ms_t time, rcc_FreqHz_t lsiClk, iwdg_Divider_t divider, iwdg_Ticks_t * const ticks )
{
    iwdg_RequestState_t retState = IWDG_REQUEST_ERROR;

    if( ( IWDG_NULL_PTR != ticks   ) &&
        ( 0u             < lsiClk  ) &&
        ( 0u             < divider )    )
    {
        const uint64_t tickPeriod = (uint64_t)divider * IWDG_MS_IN_SECOND;
        const uint64_t tickCnt    = ( ( (uint64_t)time * lsiClk ) + ( tickPeriod / IWDG_ROUNDING_DIV ) ) / tickPeriod;

        if( ( IWDG_TICKS_MIN <= tickCnt ) &&
            ( IWDG_TICKS_MAX >= tickCnt )    )
        {
            *ticks   = (iwdg_Ticks_t)tickCnt;
            retState = IWDG_REQUEST_OK;
        }
        else
        {
            /* Time is not achievable with this prescaler */
            retState = IWDG_REQUEST_ERROR;
        }
    }
    else
    {
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the smallest prescaler which can achieve required timeout.
 *
 * \param timeout  [in]: Required timeout in ms, must be non-zero
 * \param lsiClk   [in]: LSI frequency in Hz
 * \param prescId [out]: Pointer to store prescaler identification. Must not be NULL.
 * \param ticks   [out]: Pointer to store number of timeout ticks. Must not be NULL.
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if timeout
 *         is achievable. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
static iwdg_RequestState_t Iwdg_Calc_Prescaler( iwdg_Time_ms_t timeout, rcc_FreqHz_t lsiClk, iwdg_Prescaler_t * const prescId, iwdg_Ticks_t * const ticks )
{
    iwdg_RequestState_t retState = IWDG_REQUEST_ERROR;

    if( ( IWDG_NULL_PTR    != prescId ) &&
        ( IWDG_NULL_PTR    != ticks   ) &&
        ( IWDG_TIME_UNUSED != timeout )    )
    {
        for( iwdg_Prescaler_t searchId = IWDG_PRESCALER_DIV_4; IWDG_PRESCALER_CNT > searchId; searchId ++ )
        {
            retState = Iwdg_Calc_Ticks( timeout, lsiClk, iwdg_PrescalerLut[ searchId ].Divider, ticks );

            if( IWDG_REQUEST_OK == retState )
            {
                *prescId = searchId;
                break;
            }
            else
            {
                /* Timeout is too long for this prescaler, try next one */
            }
        }
    }
    else
    {
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Waits until all register updates are finished.
 *
 * Checks status register update flags (PVU, RVU). A register protected by an
 * update flag may be written only when the flag is cleared.
 *
 * \return Returns \ref IWDG_REQUEST_OK if all update flags are cleared within
 *         timeout. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
static iwdg_RequestState_t Iwdg_Check_UpdateDone( void )
{
    iwdg_RequestState_t retState   = IWDG_REQUEST_ERROR;
    uint32_t            readyState = IWDG_LL_STATE_INACTIVE;

    for( uint32_t iterationCnt = 0u; IWDG_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        readyState = LL_IWDG_IsReady( IWDG );

        if( IWDG_LL_STATE_ACTIVE == readyState )
        {
            retState = IWDG_REQUEST_OK;
            break;
        }
        else
        {
            /* Register update is still in progress */
            retState = IWDG_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Checks that APB1 clock is high enough for IWDG register updates.
 *
 * Device errata "RVU / PVU flag not cleared at low APB clock frequency": at APB clock lower
 * than twice the IWDG clock frequency the update flags are never cleared and the prescaler /
 * reload register can no longer be written. Maximal LSI frequency is used as IWDG clock.
 *
 * \return Returns \ref IWDG_REQUEST_OK if APB1 clock is higher than \ref IWDG_APB_MIN_HZ.
 *         Otherwise (also if APB1 clock is not available) returns \ref IWDG_REQUEST_ERROR.
 */
static iwdg_RequestState_t Iwdg_Check_ApbClk( void )
{
    iwdg_RequestState_t      retState = IWDG_REQUEST_ERROR;
    rcc_FreqHz_t             apbClk   = 0u;
    const rcc_RequestState_t rccState = Rcc_Get_ClkBusClk( RCC_CLK_BUS_APB1, &apbClk );

    if( ( RCC_REQUEST_OK  == rccState ) &&
        ( IWDG_APB_MIN_HZ <  apbClk   )    )
    {
        retState = IWDG_REQUEST_OK;
    }
    else
    {
        /* Register update flags would never be cleared */
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts the watchdog (LSI is enabled by HW).
 *
 * STM32F4 IWDG has no running flag, start can not be verified by read-back.
 * Running LSI clock domain is verified by the following register updates
 * (prescaler and reload update flags are cleared by the LSI clocked domain).
 */
static void Iwdg_Set_Started( void )
{
    LL_IWDG_Enable( IWDG );

    iwdg_StartedState = IWDG_FUNCTION_ACTIVE;
}


/**
 * \brief Configures watchdog prescaler.
 *
 * \pre   Register write access shall be enabled and no prescaler update may be
 *        in progress; otherwise \ref IWDG_REQUEST_ERROR and no register is changed.
 *
 * \param prescId [in]: Prescaler identification, value from \ref iwdg_Prescaler_t
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
static iwdg_RequestState_t Iwdg_Set_Prescaler( iwdg_Prescaler_t prescId )
{
    iwdg_RequestState_t retState   = IWDG_REQUEST_ERROR;
    uint32_t            readyState = IWDG_LL_STATE_INACTIVE;
    uint32_t            regValue   = 0u;

    if( IWDG_PRESCALER_CNT > prescId )
    {
        retState = Iwdg_Check_UpdateDone();

        if( IWDG_REQUEST_OK == retState )
        {
            LL_IWDG_SetPrescaler( IWDG, iwdg_PrescalerLut[ prescId ].RegVal );

            for( uint32_t iterationCnt = 0u; IWDG_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                readyState = LL_IWDG_IsActiveFlag_PVU( IWDG );
                regValue   = LL_IWDG_GetPrescaler( IWDG );

                if( ( IWDG_LL_STATE_INACTIVE             == readyState ) &&
                    ( iwdg_PrescalerLut[ prescId ].RegVal == regValue   )    )
                {
                    retState = IWDG_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Prescaler has not been updated yet */
                    retState = IWDG_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Previous update is in progress, nothing is written */
        }
    }
    else
    {
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures watchdog counter reload value.
 *
 * \note  New reload value is loaded to the counter by the next refresh.
 *
 * \pre   Register write access shall be enabled and no reload value update may
 *        be in progress; otherwise \ref IWDG_REQUEST_ERROR and no register is changed.
 *
 * \param reloadVal [in]: Reload register value (0 - \c IWDG_RLR_RL)
 *
 * \return Function processing state. Returns \ref IWDG_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref IWDG_REQUEST_ERROR.
 */
static iwdg_RequestState_t Iwdg_Set_Reload( uint32_t reloadVal )
{
    iwdg_RequestState_t retState   = IWDG_REQUEST_ERROR;
    uint32_t            readyState = IWDG_LL_STATE_INACTIVE;
    uint32_t            regValue   = 0u;

    if( IWDG_RLR_RL >= reloadVal )
    {
        retState = Iwdg_Check_UpdateDone();

        if( IWDG_REQUEST_OK == retState )
        {
            LL_IWDG_SetReloadCounter( IWDG, reloadVal );

            for( uint32_t iterationCnt = 0u; IWDG_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                readyState = LL_IWDG_IsActiveFlag_RVU( IWDG );
                regValue   = LL_IWDG_GetReloadCounter( IWDG );

                if( ( IWDG_LL_STATE_INACTIVE == readyState ) &&
                    ( reloadVal              == regValue   )    )
                {
                    retState = IWDG_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Reload value has not been updated yet */
                    retState = IWDG_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Previous update is in progress, nothing is written */
        }
    }
    else
    {
        retState = IWDG_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
