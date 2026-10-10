/**
 * \author Mr.Nobody
 * \file Test_Iwdg.c
 * \ingroup Iwdg
 * \brief Unit tests of Independent Watchdog (IWDG) module.
 *
 * Iwdg.c is compiled unchanged with real LL drivers. IWDG and FLASH option
 * registers are emulated by RegMem, RCC (LSI frequency) module is mocked by CMock.
 *
 * \note STM32F7 IWDG has no running flag - the module keeps the started state
 *       itself and it can not be cleared (watchdog is stopped only by system
 *       reset). ctest runs every test case in own process, the runner started
 *       without arguments runs all tests in one process - tests expecting not
 *       started watchdog are therefore placed before the first successful
 *       Iwdg_Init() (Unity runs the tests in file order).
 *
 * \note Key register is write only on HW, emulated register keeps the last
 *       written key. Update flags (SR.PVU, SR.RVU, SR.WVU) are cleared in
 *       emulated registers - register update is finished immediately.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Iwdg_Port.h"                      /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "Stm32_iwdg.h"                     /* IWDG registers definition      */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 Ut_Iwdg_EarlyWakeupCallback ( void );
static void                 Ut_Iwdg_Expect_LsiClk       ( rcc_FreqHz_t lsiClk );
static void                 Ut_Iwdg_Expect_Apb1Clk      ( rcc_FreqHz_t apb1Clk );
static iwdg_Config_t        Ut_Iwdg_Get_Config          ( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window, iwdg_Time_ms_t earlyWakeup );
static void                 Ut_Iwdg_Init_Started        ( iwdg_Time_ms_t timeout );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Nominal LSI frequency */
#define UT_IWDG_LSI_HZ                      ( 32000u )

/** APB1 clock used by tests */
#define UT_IWDG_APB1_HZ                     ( 42000000u )

/** Minimal APB1 clock - twice the maximal LSI frequency 47 kHz (device errata) */
#define UT_IWDG_APB1_MIN_HZ                 ( 94000u )

/** Timeout used by tests - 1 s with LSI 32 kHz needs prescaler 8 (4000 ticks) */
#define UT_IWDG_TIMEOUT_MS                  ( 1000u )

/** Reload value of UT_IWDG_TIMEOUT_MS (4000 ticks - 1) */
#define UT_IWDG_TIMEOUT_RELOAD              ( 3999u )

/** Window used by tests */
#define UT_IWDG_WINDOW_MS                   ( 200u )

/** Window register value of UT_IWDG_WINDOW_MS (800 ticks - 1) */
#define UT_IWDG_WINDOW_REG                  ( 799u )

/** Window register value of disabled window */
#define UT_IWDG_WINDOW_DISABLED             ( IWDG_WINR_WIN )

/** Early wakeup time used by tests (not supported by STM32F7) */
#define UT_IWDG_EWI_MS                      ( 100u )

/** Longest timeout achievable with LSI 32 kHz (prescaler 256, 4096 ticks) */
#define UT_IWDG_TIMEOUT_MAX_MS              ( 32768u )

/** Early wakeup interrupt priority used by tests */
#define UT_IWDG_IRQ_PRIO                    ( 2u )

/** Default timeout of Iwdg_Get_DefaultConfig */
#define UT_IWDG_DEFAULT_TIMEOUT_MS          ( 1000u )

/** Key register value which is not written by the module (detects no write) */
#define UT_IWDG_KEY_NONE                    ( 0x1234u )

/* ========================== LOCAL VARIABLES =============================== */

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    /* Software watchdog mode selected by option bytes (reset value IWDG_SW = 1) */
    FLASH->OPTCR = FLASH_OPTCR_IWDG_SW;

    IWDG->KR = UT_IWDG_KEY_NONE;
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Iwdg_Get_ModuleVersion() returns version of the module.
 *
 * \details Reads the module version structure.
 *
 * \par Expected results
 * - Version is 1.0.0 (Major 1, Minor 0, Patch 0).
 */
void Ut_Iwdg_Get_ModuleVersion_ReturnsVersion( void )
{
    iwdg_ModuleVersion_t version = Iwdg_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* =========================== DEFAULT CONFIG =============================== */

/**
 * \brief   Iwdg_Get_DefaultConfig() fills default configuration.
 *
 * \details Reads default configuration, then calls the function with NULL.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, timeout 1000 ms, window and early wakeup unused, no callback.
 * - NULL pointer: IWDG_REQUEST_ERROR.
 */
void Ut_Iwdg_Get_DefaultConfig_ReturnsDefaults( void )
{
    iwdg_Config_t config;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_DEFAULT_TIMEOUT_MS, config.Timeout );
    TEST_ASSERT_EQUAL_UINT32( IWDG_TIME_UNUSED, config.Window );
    TEST_ASSERT_EQUAL_UINT32( IWDG_TIME_UNUSED, config.EarlyWakeup );
    TEST_ASSERT_NULL( config.EarlyWakeupIsr );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Get_DefaultConfig( NULL ) );
}

/* ===================== NOT STARTED WATCHDOG (FIRST) ======================= */

/**
 * \brief   Iwdg_Get_State() of not started watchdog in software watchdog mode.
 *
 * \details Reads state before any Iwdg_Init() with option byte IWDG_SW set, then
 *          calls the function with NULL.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, state IWDG_FUNCTION_INACTIVE.
 * - NULL pointer: IWDG_REQUEST_ERROR.
 */
void Ut_Iwdg_Get_State_NotStarted_Inactive( void )
{
    iwdg_FunctionState_t state = IWDG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_INACTIVE, state );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Get_State( NULL ) );
}


/**
 * \brief   Iwdg_Get_State() reports hardware watchdog mode as running.
 *
 * \details Clears option byte IWDG_SW (hardware watchdog - started after reset)
 *          and reads state before any Iwdg_Init().
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, state IWDG_FUNCTION_ACTIVE.
 */
void Ut_Iwdg_Get_State_HardwareWatchdog_Active( void )
{
    iwdg_FunctionState_t state = IWDG_FUNCTION_INACTIVE;

    FLASH->OPTCR = 0u;      /* Hardware watchdog mode */

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_ACTIVE, state );
}


/**
 * \brief   Iwdg_Set_Refresh() of not running watchdog is rejected.
 *
 * \details Refreshes the watchdog before any Iwdg_Init() (software watchdog mode).
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, KR not written.
 */
void Ut_Iwdg_Set_Refresh_NotRunning_ReturnsErrorWithoutWrite( void )
{
    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Set_Refresh() );
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Set_Refresh() of hardware watchdog writes reload key.
 *
 * \details Clears option byte IWDG_SW and refreshes the watchdog before any Iwdg_Init().
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, KR = reload key.
 */
void Ut_Iwdg_Set_Refresh_HardwareWatchdog_WritesReloadKey( void )
{
    FLASH->OPTCR = 0u;      /* Hardware watchdog mode */

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_RELOAD, IWDG->KR );
}


/**
 * \brief   Iwdg_Task() of not initialized module does nothing.
 *
 * \details Calls Iwdg_Task() before any Iwdg_Init() in hardware watchdog mode.
 *
 * \par Expected results
 * - KR not written (automatic refresh is enabled only by Iwdg_Init()).
 */
void Ut_Iwdg_Task_NotInitialized_DoesNotRefresh( void )
{
    FLASH->OPTCR = 0u;      /* Hardware watchdog mode - refresh would be allowed */

    Iwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}

/* ================== INITIALIZATION ERRORS (NOT STARTED) =================== */

/**
 * \brief   Iwdg_Init() rejects timeout above maximum.
 *
 * \details Initializes timeout of maximum (32768 ms) + 100 ms.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, PR, RLR and KR not written, state stays inactive.
 */
void Ut_Iwdg_Init_TimeoutNotAchievable_ReturnsErrorWithoutAccess( void )
{
    iwdg_Config_t        config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MAX_MS + 100u, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );
    iwdg_FunctionState_t state  = IWDG_FUNCTION_ACTIVE;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->PR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->RLR );
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );     /* Watchdog not started */

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_INACTIVE, state );
}


/**
 * \brief   Iwdg_Init() rejects zero timeout.
 *
 * \details Initializes timeout IWDG_TIME_UNUSED (0).
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, watchdog not started.
 */
void Ut_Iwdg_Init_ZeroTimeout_ReturnsError( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( IWDG_TIME_UNUSED, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() rejects window equal to timeout.
 *
 * \details Initializes 1000 ms timeout with 1000 ms window.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, APB1 clock not checked, watchdog not started.
 */
void Ut_Iwdg_Init_WindowNotShorterThanTimeout_ReturnsError( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() rejects early wakeup interrupt (not supported by STM32F7).
 *
 * \details Initializes 1000 ms timeout with 100 ms early wakeup.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, RCC not called, watchdog not started.
 */
void Ut_Iwdg_Init_EarlyWakeupUsed_ReturnsErrorWithoutAccess( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, UT_IWDG_EWI_MS );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() rejects not running LSI.
 *
 * \details RCC mock returns LSI frequency 0 Hz.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, watchdog not started.
 */
void Ut_Iwdg_Init_LsiNotAvailable_ReturnsErrorWithoutAccess( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( 0u );     /* LSI not running */

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() reports error of LSI frequency read.
 *
 * \details Rcc_Get_PeriphClk() returns error.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, watchdog not started.
 */
void Ut_Iwdg_Init_RccError_ReturnsError( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_IWDG, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() rejects NULL configuration.
 *
 * \details Calls Iwdg_Init() with NULL pointer.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, RCC not called.
 */
void Ut_Iwdg_Init_NullConfig_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( NULL ) );
}

/* ============================== TIMEOUT =================================== */

/**
 * \brief   Iwdg_Get_Timeout() calculates timeout from PR, RLR and LSI.
 *
 * \details Presets PR = prescaler 8, RLR = 3999, LSI 32 kHz.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, timeout 1000 ms.
 */
void Ut_Iwdg_Get_Timeout_CalculatesFromRegisters( void )
{
    iwdg_Time_ms_t timeout = 0u;

    IWDG->PR  = LL_IWDG_PRESCALER_8;
    IWDG->RLR = UT_IWDG_TIMEOUT_RELOAD;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_MS, timeout );
}


/**
 * \brief   Iwdg_Get_Timeout() calculates the longest timeout.
 *
 * \details Presets PR = prescaler 256, RLR = maximum, LSI 32 kHz.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, timeout 32768 ms.
 */
void Ut_Iwdg_Get_Timeout_MaxRegisters_ReturnsMaxTimeout( void )
{
    iwdg_Time_ms_t timeout = 0u;

    IWDG->PR  = LL_IWDG_PRESCALER_256;
    IWDG->RLR = IWDG_RLR_RL;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_MAX_MS, timeout );
}


/**
 * \brief   Iwdg_Get_Timeout() rejects read during reload update.
 *
 * \details Presets SR.RVU (reload value update in progress).
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR is returned.
 */
void Ut_Iwdg_Get_Timeout_UpdateInProgress_ReturnsError( void )
{
    iwdg_Time_ms_t timeout = 0u;

    IWDG->SR = IWDG_SR_RVU;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Get_Timeout( &timeout ) );
}


/**
 * \brief   Iwdg_Get_Timeout() rejects NULL pointer.
 *
 * \details Calls Iwdg_Get_Timeout() with NULL pointer.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR is returned.
 */
void Ut_Iwdg_Get_Timeout_NullPtr_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Get_Timeout( NULL ) );
}

/* =================== INITIALIZATION (WATCHDOG STARTED) ==================== */

/**
 * \brief   Iwdg_Init() with timeout only configures prescaler and reload.
 *
 * \details Initializes 1000 ms timeout (LSI 32 kHz from RCC mock).
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, PR = prescaler 8, RLR = 3999 (4000 ticks), window disabled
 *   (WINR = 0xFFF - window write reloads the counter).
 * - Last key is "reload" - new configuration is loaded to the counter
 *   (write access disabled by the reload key).
 * - State reads back IWDG_FUNCTION_ACTIVE.
 */
void Ut_Iwdg_Init_TimeoutOnly_ConfiguresPrescalerAndReload( void )
{
    iwdg_Config_t        config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );
    iwdg_FunctionState_t state  = IWDG_FUNCTION_INACTIVE;

    config.EarlyWakeupIsr = NULL;   /* Default configuration of not used items */
    config.IrqPriority    = 0u;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Ut_Iwdg_Expect_Apb1Clk( UT_IWDG_APB1_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_PRESCALER_8,      IWDG->PR );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_RELOAD,  IWDG->RLR );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_WINDOW_DISABLED, IWDG->WINR );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_RELOAD,       IWDG->KR );    /* Counter reloaded at the end */

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_ACTIVE, state );
}


/**
 * \brief   Iwdg_Init() ignores early wakeup callback and priority.
 *
 * \details Initializes 1000 ms timeout with callback and interrupt priority set
 *          (window and early wakeup unused).
 *
 * \par Expected results
 * - IWDG_REQUEST_OK (STM32F7 IWDG has no interrupt, items are not used).
 */
void Ut_Iwdg_Init_CallbackWithoutEarlyWakeup_Ignored( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Ut_Iwdg_Expect_Apb1Clk( UT_IWDG_APB1_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_RELOAD, IWDG->RLR );
}


/**
 * \brief   Iwdg_Init() configures the window.
 *
 * \details Initializes 1000 ms timeout with 200 ms window (LSI 32 kHz), calls Iwdg_Task()
 *          and refreshes the watchdog manually.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, PR = prescaler 8, RLR = 3999, WINR = 799 (800 ticks - 1).
 * - Last key is "write access disable" - the counter is reloaded by the window write, no
 *   refresh key (refresh before the window would reset the MCU).
 * - Iwdg_Task() does not refresh (left to the application), Iwdg_Set_Refresh() writes the
 *   reload key.
 */
void Ut_Iwdg_Init_Window_ConfiguresWindowRegister( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, UT_IWDG_WINDOW_MS, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Ut_Iwdg_Expect_Apb1Clk( UT_IWDG_APB1_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_PRESCALER_8,       IWDG->PR );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_RELOAD,   IWDG->RLR );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_WINDOW_REG,       IWDG->WINR );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );

    Iwdg_Task();
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_RELOAD, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() selects the smallest prescaler for short timeout.
 *
 * \details Initializes 1 ms timeout (LSI 32 kHz).
 *
 * \par Expected results
 * - PR = prescaler 4, RLR = 7 (8 ticks).
 */
void Ut_Iwdg_Init_ShortTimeout_UsesSmallestPrescaler( void )
{
    Ut_Iwdg_Init_Started( 1u );

    /* 1 ms * 32 kHz / 4 = 8 ticks */
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_PRESCALER_4, IWDG->PR );
    TEST_ASSERT_EQUAL_UINT32( 7u, IWDG->RLR );
}


/**
 * \brief   Iwdg_Init() handles the longest timeout.
 *
 * \details Initializes 32768 ms timeout (prescaler 256, 4096 ticks at 32 kHz).
 *
 * \par Expected results
 * - PR = prescaler 256, RLR = maximum reload value.
 */
void Ut_Iwdg_Init_MaxTimeout_UsesLargestPrescaler( void )
{
    Ut_Iwdg_Init_Started( UT_IWDG_TIMEOUT_MAX_MS );

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_PRESCALER_256, IWDG->PR );
    TEST_ASSERT_EQUAL_UINT32( IWDG_RLR_RL, IWDG->RLR );
}


/**
 * \brief   Iwdg_Init() reports never ending prescaler update.
 *
 * \details Presets SR.PVU (previous update never finishes).
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, PR and RLR not written.
 * - Counter is not reloaded (last key "write access disable").
 * - Iwdg_Task() does not refresh the watchdog afterwards.
 */
void Ut_Iwdg_Init_PrescalerUpdateInProgress_ReturnsErrorWithoutWrite( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_PVU;     /* Previous prescaler update never finishes */

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Ut_Iwdg_Expect_Apb1Clk( UT_IWDG_APB1_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->PR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->RLR );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );

    IWDG->KR = UT_IWDG_KEY_NONE;
    Iwdg_Task();
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() rejects APB1 clock not higher than twice the maximal LSI frequency.
 *
 * \details APB1 clock 94 kHz (2 x 47 kHz), then APB1 clock read error.
 *
 * \note    Device errata bug AB#661 (ES0182 2.8.3 / 2.8.4 "RVU / PVU flag not cleared at low APB
 *          clock frequency"): at APB clock lower than twice the IWDG clock the update flags are
 *          never cleared and PR / RLR can no longer be written. The watchdog must not be started
 *          in such configuration.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, watchdog not started (KR, PR and RLR not written), state inactive.
 */
void Ut_Iwdg_Init_ApbClkTooLow_ReturnsErrorWithoutAccess( void )
{
    iwdg_Config_t        config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );
    iwdg_FunctionState_t state  = IWDG_FUNCTION_ACTIVE;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Ut_Iwdg_Expect_Apb1Clk( UT_IWDG_APB1_MIN_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Rcc_Get_ClkBusClk_ExpectAndReturn( RCC_CLK_BUS_APB1, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_ClkBusClk_IgnoreArg_clkBusFreq();

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->PR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->RLR );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_INACTIVE, state );
}

/* ========================== STATE AND REFRESH ============================= */

/**
 * \brief   Iwdg_Set_Refresh() of running watchdog writes reload key.
 *
 * \details Starts the watchdog by Iwdg_Init() and refreshes it.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, KR = reload key.
 */
void Ut_Iwdg_Set_Refresh_Running_WritesReloadKey( void )
{
    Ut_Iwdg_Init_Started( UT_IWDG_TIMEOUT_MS );

    IWDG->KR = UT_IWDG_KEY_NONE;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_RELOAD, IWDG->KR );
}


/**
 * \brief   Iwdg_Task() refreshes initialized watchdog.
 *
 * \details Initializes 1000 ms timeout and calls Iwdg_Task().
 *
 * \par Expected results
 * - KR = reload key.
 */
void Ut_Iwdg_Task_AfterInit_RefreshesAutomatically( void )
{
    Ut_Iwdg_Init_Started( UT_IWDG_TIMEOUT_MS );

    IWDG->KR = UT_IWDG_KEY_NONE;

    Iwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_RELOAD, IWDG->KR );
}


/**
 * \brief   Iwdg_Task() does not refresh after Iwdg_Deinit(), manual refresh works.
 *
 * \details Initializes the watchdog, deinitializes it and calls Iwdg_Task(),
 *          then refreshes the watchdog manually.
 *
 * \par Expected results
 * - Iwdg_Deinit() returns IWDG_REQUEST_OK, KR not written by Iwdg_Task().
 * - Watchdog keeps running: state IWDG_FUNCTION_ACTIVE, Iwdg_Set_Refresh() returns
 *   IWDG_REQUEST_OK and writes the reload key.
 */
void Ut_Iwdg_Task_AfterDeinit_DoesNotRefresh( void )
{
    iwdg_FunctionState_t state = IWDG_FUNCTION_INACTIVE;

    Ut_Iwdg_Init_Started( UT_IWDG_TIMEOUT_MS );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Deinit() );

    IWDG->KR = UT_IWDG_KEY_NONE;
    Iwdg_Task();
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_ACTIVE, state );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_RELOAD, IWDG->KR );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/** Early wakeup user callback - not called on STM32F7 (no early wakeup interrupt) */
static void Ut_Iwdg_EarlyWakeupCallback( void )
{
    TEST_FAIL_MESSAGE( "Early wakeup callback shall not be called" );
}


/** Expects LSI frequency request to RCC module */
static void Ut_Iwdg_Expect_LsiClk( rcc_FreqHz_t lsiClk )
{
    static rcc_FreqHz_t lsiClkRet;

    lsiClkRet = lsiClk;

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_IWDG, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &lsiClkRet );
}


/** Expects APB1 clock request to RCC module */
static void Ut_Iwdg_Expect_Apb1Clk( rcc_FreqHz_t apb1Clk )
{
    static rcc_FreqHz_t apb1ClkRet;

    apb1ClkRet = apb1Clk;

    Rcc_Get_ClkBusClk_ExpectAndReturn( RCC_CLK_BUS_APB1, NULL, RCC_REQUEST_OK );
    Rcc_Get_ClkBusClk_IgnoreArg_clkBusFreq();
    Rcc_Get_ClkBusClk_ReturnThruPtr_clkBusFreq( &apb1ClkRet );
}


/** Returns watchdog configuration */
static iwdg_Config_t Ut_Iwdg_Get_Config( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window, iwdg_Time_ms_t earlyWakeup )
{
    iwdg_Config_t config;

    config.Timeout        = timeout;
    config.Window         = window;
    config.EarlyWakeup    = earlyWakeup;
    config.EarlyWakeupIsr = Ut_Iwdg_EarlyWakeupCallback;
    config.IrqPriority    = UT_IWDG_IRQ_PRIO;

    return ( config );
}


/** Initializes the watchdog with timeout only, expects success */
static void Ut_Iwdg_Init_Started( iwdg_Time_ms_t timeout )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( timeout, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Ut_Iwdg_Expect_Apb1Clk( UT_IWDG_APB1_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );
}
