/**
 * \author Mr.Nobody
 * \file Test_Iwdg.c
 * \ingroup Iwdg
 * \brief Unit tests of Independent Watchdog (IWDG) module.
 *
 * Iwdg.c is compiled unchanged with real LL drivers. IWDG and FLASH option
 * registers are emulated by RegMem, RCC (LSI frequency) module is mocked by CMock.
 *
 * \note STM32G4 IWDG has no running flag - the module keeps the started state
 *       itself and it can not be cleared (watchdog is stopped only by system
 *       reset). ctest runs every test case in own process, the runner started
 *       without arguments runs all tests in one process - tests expecting not
 *       started watchdog are therefore placed before the first successful
 *       Iwdg_Init() (Unity runs the tests in file order).
 *
 * \note Key register is write only on HW, emulated register keeps the last
 *       written key. Update flags (SR.PVU, SR.RVU, SR.WVU) are cleared in
 *       emulated registers - register update is finished immediately. Reset
 *       value of emulated WINR is 0 (HW reset value is IWDG_WINR_WIN) - write
 *       of disabled window is therefore visible.
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
static iwdg_Config_t        Ut_Iwdg_Get_Config          ( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window, iwdg_Time_ms_t earlyWakeup );
static void                 Ut_Iwdg_Init_Started        ( iwdg_Time_ms_t timeout );
static void                 Ut_Iwdg_Init_Window         ( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Nominal LSI frequency */
#define UT_IWDG_LSI_HZ                      ( 32000u )

/** Timeout used by tests - 1 s with LSI 32 kHz needs prescaler 8 (4000 ticks) */
#define UT_IWDG_TIMEOUT_MS                  ( 1000u )

/** Reload value of UT_IWDG_TIMEOUT_MS (4000 ticks - 1) */
#define UT_IWDG_TIMEOUT_RELOAD              ( 3999u )

/** Window used by tests - 200 ms = 800 ticks with prescaler 8 */
#define UT_IWDG_WINDOW_MS                   ( 200u )

/** Window register value of UT_IWDG_WINDOW_MS */
#define UT_IWDG_WINDOW_REG                  ( 799u )

/** Longest window of UT_IWDG_TIMEOUT_MS - 999 ms = 3996 ticks (one tick is 0.25 ms) */
#define UT_IWDG_WINDOW_MAX_MS               ( 999u )

/** Window register value of UT_IWDG_WINDOW_MAX_MS */
#define UT_IWDG_WINDOW_MAX_REG              ( 3995u )

/** Early wakeup time used by tests (not supported by STM32G4) */
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
    FLASH->OPTR = FLASH_OPTR_IWDG_SW;

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

    FLASH->OPTR = 0u;      /* Hardware watchdog mode */

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
    FLASH->OPTR = 0u;      /* Hardware watchdog mode */

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
    FLASH->OPTR = 0u;      /* Hardware watchdog mode - refresh would be allowed */

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
 * - IWDG_REQUEST_ERROR, PR, RLR, WINR and KR not written (watchdog not started).
 */
void Ut_Iwdg_Init_WindowNotShorterThanTimeout_ReturnsErrorWithoutAccess( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->PR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->RLR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->WINR );
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() rejects window shorter than one counter tick.
 *
 * \details Initializes the longest timeout 32768 ms (prescaler 256, one tick 8 ms)
 *          with 1 ms window (rounded to 0 ticks).
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, WINR and KR not written (watchdog not started).
 */
void Ut_Iwdg_Init_WindowTooShort_ReturnsErrorWithoutAccess( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MAX_MS, 1u, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->WINR );
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() rejects early wakeup interrupt (not supported by STM32G4).
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
 * \brief   Iwdg_Get_Timeout() handles second register value of prescaler 256.
 *
 * \details Presets PR = 7 (PR[2:0] = 111 - divider 256 as 110, reference manual),
 *          RLR = maximum, LSI 32 kHz.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, timeout 32768 ms.
 */
void Ut_Iwdg_Get_Timeout_PrescalerRegAlias_UsesDivider256( void )
{
    iwdg_Time_ms_t timeout = 0u;

    IWDG->PR  = IWDG_PR_PR;
    IWDG->RLR = IWDG_RLR_RL;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_MAX_MS, timeout );
}


/**
 * \brief   Iwdg_Get_Timeout() rejects unknown prescaler register value.
 *
 * \details Presets PR = 8 (outside PR[2:0], possible only in emulated register).
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR is returned, timeout not changed.
 */
void Ut_Iwdg_Get_Timeout_UnknownPrescalerReg_ReturnsError( void )
{
    iwdg_Time_ms_t timeout = UT_IWDG_TIMEOUT_MS;

    IWDG->PR  = IWDG_PR_PR + 1u;
    IWDG->RLR = UT_IWDG_TIMEOUT_RELOAD;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_MS, timeout );
}


/**
 * \brief   Iwdg_Get_Timeout() reports error of LSI frequency read.
 *
 * \details Rcc_Get_PeriphClk() returns error.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR is returned, timeout not changed.
 */
void Ut_Iwdg_Get_Timeout_RccError_ReturnsError( void )
{
    iwdg_Time_ms_t timeout = UT_IWDG_TIMEOUT_MS;

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_IWDG, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_MS, timeout );
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
 * - IWDG_REQUEST_OK, PR = prescaler 8, RLR = 3999 (4000 ticks).
 * - WINR = maximum (window disabled) - window register write loads the new
 *   configuration to the counter.
 * - Last key is "write access disable".
 * - State reads back IWDG_FUNCTION_ACTIVE.
 */
void Ut_Iwdg_Init_TimeoutOnly_ConfiguresPrescalerAndReload( void )
{
    iwdg_Config_t        config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );
    iwdg_FunctionState_t state  = IWDG_FUNCTION_INACTIVE;

    config.EarlyWakeupIsr = NULL;   /* Default configuration of not used items */
    config.IrqPriority    = 0u;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_PRESCALER_8,           IWDG->PR );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_RELOAD,       IWDG->RLR );
    TEST_ASSERT_EQUAL_HEX32( IWDG_WINR_WIN,                 IWDG->WINR );  /* Window disabled, counter reloaded */
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );    /* Write access disabled at the end */

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
 * - IWDG_REQUEST_OK (STM32G4 IWDG has no interrupt, items are not used).
 */
void Ut_Iwdg_Init_CallbackWithoutEarlyWakeup_Ignored( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_RELOAD, IWDG->RLR );
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
 * - IWDG_REQUEST_ERROR, PR, RLR and WINR not written.
 * - Counter is not reloaded (last key "write access disable").
 * - Iwdg_Task() does not refresh the watchdog afterwards.
 */
void Ut_Iwdg_Init_PrescalerUpdateInProgress_ReturnsErrorWithoutWrite( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_PVU;     /* Previous prescaler update never finishes */

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->PR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->RLR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->WINR );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );

    IWDG->KR = UT_IWDG_KEY_NONE;
    Iwdg_Task();
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() reports never ending window update.
 *
 * \details Presets SR.WVU (previous window update never finishes) and initializes
 *          1000 ms timeout with 200 ms window.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, PR, RLR and WINR not written (all update flags are checked
 *   before the first register write).
 * - Last key "write access disable", Iwdg_Task() does not refresh the watchdog.
 */
void Ut_Iwdg_Init_WindowUpdateInProgress_ReturnsErrorWithoutWrite( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, UT_IWDG_WINDOW_MS, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_WVU;     /* Previous window update never finishes */

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->PR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->RLR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->WINR );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );

    IWDG->KR = UT_IWDG_KEY_NONE;
    Iwdg_Task();
    TEST_ASSERT_EQUAL_HEX32( UT_IWDG_KEY_NONE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() with window configures window register.
 *
 * \details Initializes 1000 ms timeout with 200 ms window (LSI 32 kHz).
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, PR = prescaler 8, RLR = 3999, WINR = 799 (800 ticks).
 * - Last key is "write access disable" (counter reloaded by the window write).
 * - State reads back IWDG_FUNCTION_ACTIVE.
 */
void Ut_Iwdg_Init_Window_ConfiguresWindowRegister( void )
{
    iwdg_FunctionState_t state = IWDG_FUNCTION_INACTIVE;

    Ut_Iwdg_Init_Window( UT_IWDG_TIMEOUT_MS, UT_IWDG_WINDOW_MS );

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_PRESCALER_8,           IWDG->PR );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_RELOAD,       IWDG->RLR );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_WINDOW_REG,           IWDG->WINR );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_ACTIVE, state );
}


/**
 * \brief   Iwdg_Init() accepts window one tick shorter than timeout.
 *
 * \details Initializes 1000 ms timeout (4000 ticks) with 999 ms window (3996 ticks).
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, WINR = 3995.
 */
void Ut_Iwdg_Init_LongestWindow_ConfiguresWindowRegister( void )
{
    Ut_Iwdg_Init_Window( UT_IWDG_TIMEOUT_MS, UT_IWDG_WINDOW_MAX_MS );

    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_WINDOW_MAX_REG, IWDG->WINR );
}


/**
 * \brief   Iwdg_Init() without window disables the window of previous configuration.
 *
 * \details Initializes 1000 ms timeout with 200 ms window, then initializes
 *          1000 ms timeout without window and calls Iwdg_Task().
 *
 * \par Expected results
 * - WINR = maximum (window disabled) after the second Iwdg_Init().
 * - Iwdg_Task() refreshes the watchdog (KR = reload key).
 */
void Ut_Iwdg_Init_ReinitWithoutWindow_DisablesWindow( void )
{
    Ut_Iwdg_Init_Window( UT_IWDG_TIMEOUT_MS, UT_IWDG_WINDOW_MS );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_WINDOW_REG, IWDG->WINR );

    Ut_Iwdg_Init_Started( UT_IWDG_TIMEOUT_MS );
    TEST_ASSERT_EQUAL_HEX32( IWDG_WINR_WIN, IWDG->WINR );

    Iwdg_Task();
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_RELOAD, IWDG->KR );
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
 * \brief   Iwdg_Task() does not refresh watchdog with window, manual refresh works.
 *
 * \details Initializes 1000 ms timeout with 200 ms window and calls Iwdg_Task(),
 *          then refreshes the watchdog manually.
 *
 * \par Expected results
 * - KR keeps "write access disable" key after Iwdg_Task() (refresh outside the
 *   window would reset the MCU - left to application).
 * - Iwdg_Set_Refresh() returns IWDG_REQUEST_OK and writes the reload key.
 */
void Ut_Iwdg_Task_WithWindow_DoesNotRefresh( void )
{
    Ut_Iwdg_Init_Window( UT_IWDG_TIMEOUT_MS, UT_IWDG_WINDOW_MS );

    Iwdg_Task();
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
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

/** Early wakeup user callback - not called on STM32G4 (no early wakeup interrupt) */
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

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );
}


/** Initializes the watchdog with timeout and window, expects success */
static void Ut_Iwdg_Init_Window( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( timeout, window, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );
}
