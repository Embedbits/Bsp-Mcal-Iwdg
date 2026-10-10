/**
 * \author Mr.Nobody
 * \file ItTest_Iwdg.c
 * \ingroup Iwdg
 * \brief Integration tests of Independent watchdog (IWDG) module on target.
 *
 * Iwdg module runs on the MCU together with real RCC module and hardware (LSI
 * clock). Tests verify behavior which cannot be verified by unit tests
 * (emulated registers): watchdog start, register update synchronization with
 * LSI clock, reset of the MCU after timeout, refresh and reconfiguration of
 * running watchdog, window violation reset.
 *
 * Tests expecting reset of the MCU use stages of the integration testing
 * (\c IntegrationTesting_Set_ResetExpected), stage 1 checks the reset source
 * through RCC module. Running watchdog is stopped by the system reset done by
 * the test runner after every test case.
 *
 * Used resources:
 * - IWDG, LSI clock
 * - DWT cycle counter - reference time base (\ref It_Iwdg_Get_Time_ms), stops
 *   counting while the core is halted by the debugger as the IWDG does
 *   (DBGMCU freeze, see ItTarget_Iwdg.c), no external wiring
 *
 * \note LSI frequency tolerance is covered by tolerances of time checks.
 *       The early wakeup interrupt exists only in the IWDG of STM32H7R / H7S,
 *       the IWDG1 of the classic STM32H7 lines rejects the configuration. The
 *       maximal timeout is 32.8 s (prescaler 256) on the classic lines and 131 s
 *       (prescaler 1024) on STM32H7R / H7S.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Iwdg_Port.h"                      /* Module under test              */
#include "Rcc_Port.h"                       /* Reset source, core clock       */
#include "Stm32_cortex.h"                   /* DWT cycle counter (CMSIS)      */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void             It_Iwdg_Init_RefTime    ( void );
static iwdg_Time_ms_t   It_Iwdg_Get_Time_ms     ( void );
static void             It_Iwdg_Restart_RefTime ( void );
static void             It_Iwdg_Wait_Until_ms   ( iwdg_Time_ms_t time_ms, iwdg_FunctionState_t taskState );
static void             It_Iwdg_Init_Watchdog   ( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window );
static void             It_Iwdg_Check_IwdgReset ( void );
#if defined(STM32H7RS)
static void             It_Iwdg_EarlyWakeupCallback( void );
#endif /* STM32H7RS */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Watchdog timeout used by tests [ms] */
#define IT_IWDG_TIMEOUT_MS                  ( 100u )

/** Long watchdog timeout used by reconfiguration test [ms] */
#define IT_IWDG_LONG_TIMEOUT_MS             ( 2000u )

/** Time without refresh after which the reset must have occurred [ms] */
#define IT_IWDG_RESET_WAIT_MS               ( 1000u )

/** Relative tolerance of watchdog times (LSI accuracy, register synchronization) [%] */
#define IT_IWDG_TIME_TOL_PCT                ( 20u )

/** Timeout above hardware maximum (4096 * 256 / LSI, approx. 32.8 s; 4096 * 1024 / LSI, approx. 131 s on STM32H7R / H7S) [ms] */
#if defined(STM32H7RS)
    #define IT_IWDG_TIMEOUT_TOO_LONG_MS     ( 150000u )
#else
    #define IT_IWDG_TIMEOUT_TOO_LONG_MS     ( 40000u )
#endif /* STM32H7RS */

/** Longest timeout achievable with nominal LSI 32 kHz [ms] */
#define IT_IWDG_TIMEOUT_MAX_MS              ( 32768u )

/** Number of milliseconds in one second */
#define IT_IWDG_MS_IN_SECOND                ( 1000u )

/* ============================== MACROS ==================================== */

/** Absolute tolerance of watchdog time [ms] */
#define IT_IWDG_TIME_TOL( time_ms )         ( ( ( time_ms ) * IT_IWDG_TIME_TOL_PCT ) / 100u )

/* ========================== LOCAL VARIABLES =============================== */

/** Core clock cycles per millisecond (reference time base) */
static uint32_t                 itIwdg_CyclesPerMs      = 0u;

#if defined(STM32H7RS)
/** Count of early wakeup interrupts (STM32H7R / H7S) */
static volatile uint32_t        itIwdg_EarlyWakeupCnt   = 0u;
#endif /* STM32H7RS */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    It_Iwdg_Init_RefTime();
}


void tearDown( void )
{
    /* Running watchdog is stopped by system reset after the test case */
}

/* =============================== TESTS ==================================== */

/*----------------------------- State and init -------------------------------*/

/**
 * \brief   Watchdog is inactive after system reset.
 *
 * \details Reads the watchdog state without initialization (system reset done by
 *          the test runner, no hardware watchdog option) and tries to refresh it.
 *
 * \par Expected results
 * - State reads back IWDG_FUNCTION_INACTIVE.
 * - Iwdg_Set_Refresh() returns IWDG_REQUEST_ERROR.
 */
void It_Iwdg_Get_State_AfterSystemReset_Inactive( void )
{
    iwdg_FunctionState_t state = IWDG_FUNCTION_ACTIVE;

    /* Watchdog is stopped by system reset (no hardware watchdog option) */
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_INACTIVE, state );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Set_Refresh() );
}


/**
 * \brief   Iwdg_Init() with default configuration starts the watchdog.
 *
 * \details Initializes the watchdog with default configuration, reads the state and
 *          the timeout (register update synchronized with LSI clock) and refreshes it.
 *
 * \par Expected results
 * - Iwdg_Init() returns IWDG_REQUEST_OK, state reads back IWDG_FUNCTION_ACTIVE.
 * - Timeout reads back the configured value +- 1 ms.
 * - Iwdg_Set_Refresh() returns IWDG_REQUEST_OK.
 */
void It_Iwdg_Init_DefaultConfig_RunningWithRequestedTimeout( void )
{
    iwdg_Config_t        config;
    iwdg_FunctionState_t state   = IWDG_FUNCTION_INACTIVE;
    iwdg_Time_ms_t       timeout = 0u;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_ACTIVE, state );

    /* Register update synchronized with LSI clock */
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_UINT32_WITHIN( 1u, config.Timeout, timeout );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
}


/**
 * \brief   Iwdg_Init() configures the longest timeout.
 *
 * \details Initializes the watchdog with timeout 32768 ms (prescaler 256, maximal
 *          reload value at nominal LSI 32 kHz) and reads the timeout back.
 *
 * \par Expected results
 * - Iwdg_Init() returns IWDG_REQUEST_OK.
 * - Timeout reads back the configured value +- 1 ms.
 */
void It_Iwdg_Init_MaxTimeout_RunningWithRequestedTimeout( void )
{
    iwdg_Time_ms_t timeout = 0u;

    It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MAX_MS, IWDG_TIME_UNUSED );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_Timeout( &timeout ) );
    TEST_ASSERT_UINT32_WITHIN( 1u, IT_IWDG_TIMEOUT_MAX_MS, timeout );
}


/**
 * \brief   Iwdg_Init() rejects timeout above the hardware maximum.
 *
 * \details Initializes the watchdog with timeout 40 s (maximum is 4096 * 256 / LSI,
 *          approx. 32.8 s).
 *
 * \par Expected results
 * - Iwdg_Init() returns IWDG_REQUEST_ERROR.
 * - Watchdog is not started (state IWDG_FUNCTION_INACTIVE).
 */
void It_Iwdg_Init_TimeoutNotAchievable_ReturnsErrorNotStarted( void )
{
    iwdg_Config_t        config;
    iwdg_FunctionState_t state = IWDG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_DefaultConfig( &config ) );
    config.Timeout = IT_IWDG_TIMEOUT_TOO_LONG_MS;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_INACTIVE, state );
}


/**
 * \brief   Iwdg_Init() rejects window not shorter than timeout.
 *
 * \details Initializes the watchdog with window equal to timeout (100 ms).
 *
 * \par Expected results
 * - Iwdg_Init() returns IWDG_REQUEST_ERROR.
 * - Watchdog is not started (state IWDG_FUNCTION_INACTIVE).
 */
void It_Iwdg_Init_WindowNotShorterThanTimeout_ReturnsErrorNotStarted( void )
{
    iwdg_Config_t        config;
    iwdg_FunctionState_t state = IWDG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_DefaultConfig( &config ) );
    config.Timeout = IT_IWDG_TIMEOUT_MS;
    config.Window  = IT_IWDG_TIMEOUT_MS;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_INACTIVE, state );
}


/**
 * \brief   Iwdg_Init() rejects early wakeup (not supported by the IWDG1 of the classic STM32H7 lines).
 *
 * \details Classic lines (ignored on STM32H7R / H7S): initializes the watchdog with
 *          200 ms timeout and early wakeup 50 ms before timeout.
 *
 * \par Expected results
 * - Iwdg_Init() returns IWDG_REQUEST_ERROR.
 * - Watchdog is not started (state IWDG_FUNCTION_INACTIVE).
 */
void It_Iwdg_Init_EarlyWakeupUsed_ReturnsErrorNotStarted( void )
{
#if !defined(STM32H7RS)
    iwdg_Config_t        config;
    iwdg_FunctionState_t state = IWDG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_DefaultConfig( &config ) );
    config.Timeout     = 2u * IT_IWDG_TIMEOUT_MS;
    config.EarlyWakeup = IT_IWDG_TIMEOUT_MS / 2u;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_INACTIVE, state );
#else
    TEST_IGNORE_MESSAGE( "IWDG of STM32H7R / H7S supports the early wakeup interrupt" );
#endif /* STM32H7RS */
}


/**
 * \brief   Early wakeup interrupt is called before the timeout (IWDG of STM32H7R / H7S).
 *
 * \details STM32H7R / H7S (ignored on the classic lines): starts the watchdog with
 *          200 ms timeout and early wakeup 100 ms before the timeout. The callback
 *          refreshes the watchdog, so the MCU is not reset. Waits without refresh from
 *          the test.
 *
 * \par Expected results
 * - No interrupt in the first 60 ms.
 * - The callback is called once until 180 ms (at ~100 ms), the MCU is not reset.
 */
void It_Iwdg_Init_EarlyWakeup_CallbackCalledBeforeTimeout( void )
{
#if defined(STM32H7RS)
    iwdg_Config_t config;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_DefaultConfig( &config ) );
    config.Timeout        = 2u * IT_IWDG_TIMEOUT_MS;
    config.EarlyWakeup    = IT_IWDG_TIMEOUT_MS;
    config.EarlyWakeupIsr = It_Iwdg_EarlyWakeupCallback;
    config.IrqPriority    = 5u;

    itIwdg_EarlyWakeupCnt = 0u;
    It_Iwdg_Restart_RefTime();
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    It_Iwdg_Wait_Until_ms( 60u, IWDG_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL_UINT32( 0u, itIwdg_EarlyWakeupCnt );

    It_Iwdg_Wait_Until_ms( 180u, IWDG_FUNCTION_INACTIVE );
    TEST_ASSERT_EQUAL_UINT32( 1u, itIwdg_EarlyWakeupCnt );
#else
    TEST_IGNORE_MESSAGE( "IWDG of the classic STM32H7 lines has no early wakeup interrupt" );
#endif /* STM32H7RS */
}

/*-------------------------------- Timeout -----------------------------------*/

/**
 * \brief   Watchdog without refresh resets the MCU after timeout.
 *
 * \details
 * 1. Stage 0: expects reset, starts the watchdog with 100 ms timeout and waits
 *    without refresh (Iwdg_Task not called) up to 1000 ms.
 * 2. Stage 1 (after reset): checks the reset source.
 *
 * \par Expected results
 * 1. No reset before timeout - tolerance (80 ms), MCU is reset before 1000 ms.
 * 2. Reset source is IWDG, software reset flag is not set.
 */
void It_Iwdg_Init_NoRefresh_ResetsMcuAfterTimeout( void )
{
    if( 0u == IntegrationTesting_Get_Stage() )
    {
        TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_ResetSourceClear() );
        IntegrationTesting_Set_ResetExpected();
        It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

        /* No reset before timeout */
        It_Iwdg_Wait_Until_ms( IT_IWDG_TIMEOUT_MS - IT_IWDG_TIME_TOL( IT_IWDG_TIMEOUT_MS ), IWDG_FUNCTION_INACTIVE );
        It_Iwdg_Wait_Until_ms( IT_IWDG_RESET_WAIT_MS, IWDG_FUNCTION_INACTIVE );

        TEST_FAIL_MESSAGE( "IWDG reset did not occur" );
    }
    else
    {
        It_Iwdg_Check_IwdgReset();
    }
}


/**
 * \brief   Reconfiguration of running watchdog applies the new timeout at once.
 *
 * \details
 * 1. Stage 0: expects reset, starts the watchdog with 2000 ms timeout, refreshes it
 *    for 300 ms, then reconfigures it to 100 ms timeout and waits without refresh
 *    up to 1000 ms.
 * 2. Stage 1 (after reset): checks the reset source.
 *
 * \par Expected results
 * 1. Reconfiguration returns IWDG_REQUEST_OK, timeout reads back 100 ms +- 1 ms.
 *    No reset before new timeout - tolerance (80 ms), MCU is reset before 1000 ms
 *    (new prescaler and reload value are loaded to the counter by Iwdg_Init()).
 * 2. Reset source is IWDG, software reset flag is not set.
 */
void It_Iwdg_Init_RunningWatchdog_NewTimeoutApplied( void )
{
    iwdg_Time_ms_t timeout = 0u;

    if( 0u == IntegrationTesting_Get_Stage() )
    {
        TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_ResetSourceClear() );
        IntegrationTesting_Set_ResetExpected();
        It_Iwdg_Init_Watchdog( IT_IWDG_LONG_TIMEOUT_MS, IWDG_TIME_UNUSED );

        It_Iwdg_Wait_Until_ms( 3u * IT_IWDG_TIMEOUT_MS, IWDG_FUNCTION_ACTIVE );

        /* Reconfiguration of running watchdog to shorter timeout */
        It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

        TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_Timeout( &timeout ) );
        TEST_ASSERT_UINT32_WITHIN( 1u, IT_IWDG_TIMEOUT_MS, timeout );

        /* No reset before new timeout */
        It_Iwdg_Wait_Until_ms( IT_IWDG_TIMEOUT_MS - IT_IWDG_TIME_TOL( IT_IWDG_TIMEOUT_MS ), IWDG_FUNCTION_INACTIVE );
        It_Iwdg_Wait_Until_ms( IT_IWDG_RESET_WAIT_MS, IWDG_FUNCTION_INACTIVE );

        TEST_FAIL_MESSAGE( "IWDG reset did not occur" );
    }
    else
    {
        It_Iwdg_Check_IwdgReset();
    }
}


/**
 * \brief   Periodic Iwdg_Task() keeps the MCU running.
 *
 * \details Starts the watchdog with 100 ms timeout and calls Iwdg_Task() continuously
 *          during 5 timeouts (500 ms).
 *
 * \par Expected results
 * - No reset of the MCU, Iwdg_Set_Refresh() returns IWDG_REQUEST_OK at the end.
 */
void It_Iwdg_Task_PeriodicCall_NoResetAfterTimeout( void )
{
    It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

    /* Automatic refresh by Iwdg_Task during 5 timeouts */
    It_Iwdg_Wait_Until_ms( 5u * IT_IWDG_TIMEOUT_MS, IWDG_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
}


/**
 * \brief   Periodic Iwdg_Set_Refresh() keeps the MCU running.
 *
 * \details Starts the watchdog with 100 ms timeout, Iwdg_Task() is not called. The
 *          watchdog is refreshed manually every 50 ms during 5 timeouts (500 ms).
 *
 * \par Expected results
 * - No reset of the MCU, every Iwdg_Set_Refresh() returns IWDG_REQUEST_OK.
 */
void It_Iwdg_Set_Refresh_PeriodicCall_NoResetAfterTimeout( void )
{
    iwdg_Time_ms_t refreshTime = 0u;

    It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

    /* Manual refresh every half of timeout during 5 timeouts */
    for( uint32_t refreshIdx = 0u; 10u > refreshIdx; refreshIdx++ )
    {
        refreshTime += IT_IWDG_TIMEOUT_MS / 2u;
        It_Iwdg_Wait_Until_ms( refreshTime, IWDG_FUNCTION_INACTIVE );

        TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
    }
}


/**
 * \brief   Iwdg_Deinit() stops automatic refresh, running watchdog resets the MCU.
 *
 * \details
 * 1. Stage 0: expects reset, starts the watchdog with 100 ms timeout, deinitializes
 *    the module and calls Iwdg_Task() continuously up to 1000 ms.
 * 2. Stage 1 (after reset): checks the reset source.
 *
 * \par Expected results
 * 1. Iwdg_Deinit() returns IWDG_REQUEST_OK, Iwdg_Task() does not refresh the watchdog
 *    (hardware watchdog cannot be stopped) - MCU is reset before 1000 ms.
 * 2. Reset source is IWDG, software reset flag is not set.
 */
void It_Iwdg_Deinit_AutoRefreshStopped_ResetsMcu( void )
{
    if( 0u == IntegrationTesting_Get_Stage() )
    {
        TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_ResetSourceClear() );
        IntegrationTesting_Set_ResetExpected();
        It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

        TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Deinit() );

        /* Iwdg_Task does not refresh after Deinit, watchdog keeps running */
        It_Iwdg_Wait_Until_ms( IT_IWDG_RESET_WAIT_MS, IWDG_FUNCTION_ACTIVE );

        TEST_FAIL_MESSAGE( "IWDG reset did not occur" );
    }
    else
    {
        It_Iwdg_Check_IwdgReset();
    }
}

/*--------------------------------- Window -----------------------------------*/

/**
 * \brief   Refresh before the window resets the MCU.
 *
 * \details
 * 1. Stage 0: expects reset, starts the watchdog with 200 ms timeout and 100 ms
 *    window (refresh allowed only in the last 100 ms), refreshes it immediately and
 *    waits 50 ms without refresh.
 * 2. Stage 1 (after reset): checks the reset source.
 *
 * \par Expected results
 * 1. Window violation resets the MCU immediately (before 50 ms).
 * 2. Reset source is IWDG, software reset flag is not set.
 */
void It_Iwdg_Set_Refresh_BeforeWindow_ResetsMcu( void )
{
    if( 0u == IntegrationTesting_Get_Stage() )
    {
        TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_ResetSourceClear() );
        IntegrationTesting_Set_ResetExpected();

        /* Refresh allowed only in the last 100 ms of 200 ms timeout */
        It_Iwdg_Init_Watchdog( 2u * IT_IWDG_TIMEOUT_MS, IT_IWDG_TIMEOUT_MS );

        (void)Iwdg_Set_Refresh();
        It_Iwdg_Wait_Until_ms( IT_IWDG_TIMEOUT_MS / 2u, IWDG_FUNCTION_INACTIVE );

        TEST_FAIL_MESSAGE( "IWDG window violation reset did not occur" );
    }
    else
    {
        It_Iwdg_Check_IwdgReset();
    }
}


/**
 * \brief   Refresh inside the window keeps the MCU running.
 *
 * \details Starts the watchdog with 200 ms timeout and 100 ms window, Iwdg_Task() is
 *          called continuously (no automatic refresh with window). The watchdog is
 *          refreshed manually at 150 ms, 300 ms and 450 ms (middle of the window).
 *
 * \par Expected results
 * - No reset of the MCU, every Iwdg_Set_Refresh() returns IWDG_REQUEST_OK.
 */
void It_Iwdg_Set_Refresh_InsideWindow_NoReset( void )
{
    /* Refresh allowed only in the last 100 ms of 200 ms timeout */
    It_Iwdg_Init_Watchdog( 2u * IT_IWDG_TIMEOUT_MS, IT_IWDG_TIMEOUT_MS );

    /* Refresh in the middle of the window (150 ms after refresh) */
    It_Iwdg_Wait_Until_ms( 150u, IWDG_FUNCTION_ACTIVE );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );

    It_Iwdg_Wait_Until_ms( 300u, IWDG_FUNCTION_ACTIVE );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );

    It_Iwdg_Wait_Until_ms( 450u, IWDG_FUNCTION_ACTIVE );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
}


/**
 * \brief   Window does not reset the MCU without refresh before timeout.
 *
 * \details
 * 1. Stage 0: expects reset, starts the watchdog with 200 ms timeout and 100 ms
 *    window, Iwdg_Task() is called continuously up to 1000 ms (no automatic refresh
 *    with window).
 * 2. Stage 1 (after reset): checks the reset source.
 *
 * \par Expected results
 * 1. No reset before timeout - tolerance (160 ms) - the window write in Iwdg_Init()
 *    is not a window violation, MCU is reset by timeout before 1000 ms.
 * 2. Reset source is IWDG, software reset flag is not set.
 */
void It_Iwdg_Task_WithWindow_ResetsMcuAfterTimeout( void )
{
    if( 0u == IntegrationTesting_Get_Stage() )
    {
        TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Set_ResetSourceClear() );
        IntegrationTesting_Set_ResetExpected();
        It_Iwdg_Init_Watchdog( 2u * IT_IWDG_TIMEOUT_MS, IT_IWDG_TIMEOUT_MS );

        /* No reset before timeout, Iwdg_Task does not refresh with window */
        It_Iwdg_Wait_Until_ms( ( 2u * IT_IWDG_TIMEOUT_MS ) - IT_IWDG_TIME_TOL( 2u * IT_IWDG_TIMEOUT_MS ), IWDG_FUNCTION_ACTIVE );
        It_Iwdg_Wait_Until_ms( IT_IWDG_RESET_WAIT_MS, IWDG_FUNCTION_ACTIVE );

        TEST_FAIL_MESSAGE( "IWDG reset did not occur" );
    }
    else
    {
        It_Iwdg_Check_IwdgReset();
    }
}


/**
 * \brief   Reconfiguration without window disables the window of running watchdog.
 *
 * \details Starts the watchdog with 200 ms timeout and 100 ms window, reconfigures it
 *          at once to 100 ms timeout without window, refreshes it immediately and then
 *          calls Iwdg_Task() continuously during 5 timeouts (500 ms).
 *
 * \par Expected results
 * - Reconfiguration returns IWDG_REQUEST_OK, immediate refresh is not a window
 *   violation (window disabled), no reset of the MCU.
 */
void It_Iwdg_Init_WindowThenNoWindow_WindowDisabled( void )
{
    It_Iwdg_Init_Watchdog( 2u * IT_IWDG_TIMEOUT_MS, IT_IWDG_TIMEOUT_MS );

    /* Reconfiguration without window */
    It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );

    /* Automatic refresh by Iwdg_Task during 5 timeouts */
    It_Iwdg_Wait_Until_ms( 5u * IT_IWDG_TIMEOUT_MS, IWDG_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes and starts reference time base (DWT cycle counter clocked
 *        by the core clock).
 */
static void It_Iwdg_Init_RefTime( void )
{
    rcc_FreqHz_t coreClk = 0u;

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphClk( RCC_PERIPH_SYSTICK, &coreClk ) );
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( IT_IWDG_MS_IN_SECOND, coreClk );

    itIwdg_CyclesPerMs = coreClk / IT_IWDG_MS_IN_SECOND;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR          = 0xC5ACCE55u;           /* Cortex-M7 DWT is locked by default */
    DWT->CYCCNT       = 0u;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}


/**
 * \brief Restarts reference time (time 0).
 */
static void It_Iwdg_Restart_RefTime( void )
{
    DWT->CYCCNT = 0u;
}


/**
 * \brief Returns reference time since \ref It_Iwdg_Restart_RefTime.
 *
 * \note  32-bit cycle counter overflows after 2^32 / core clock (25 s at 170 MHz),
 *        tests measure up to approx. 1 s.
 *
 * \return Reference time [ms]
 */
static iwdg_Time_ms_t It_Iwdg_Get_Time_ms( void )
{
    return ( DWT->CYCCNT / itIwdg_CyclesPerMs );
}


/**
 * \brief Waits until reference time is reached.
 *
 * \param time_ms   [in]: Reference time [ms]
 * \param taskState [in]: \ref IWDG_FUNCTION_ACTIVE - \ref Iwdg_Task is called during waiting
 */
static void It_Iwdg_Wait_Until_ms( iwdg_Time_ms_t time_ms, iwdg_FunctionState_t taskState )
{
    iwdg_Time_ms_t nowTime = It_Iwdg_Get_Time_ms();

    while( time_ms > nowTime )
    {
        if( IWDG_FUNCTION_ACTIVE == taskState )
        {
            Iwdg_Task();
        }
        else
        {
            /* Watchdog is not refreshed */
        }

        nowTime = It_Iwdg_Get_Time_ms();
    }
}


/**
 * \brief Initializes watchdog and restarts reference time (time 0 = watchdog start).
 *
 * \param timeout [in]: Watchdog timeout [ms]
 * \param window  [in]: Window [ms] or \ref IWDG_TIME_UNUSED
 */
static void It_Iwdg_Init_Watchdog( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window )
{
    iwdg_Config_t config;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_DefaultConfig( &config ) );
    config.Timeout = timeout;
    config.Window  = window;

    It_Iwdg_Restart_RefTime();
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );
}


/**
 * \brief Checks that the last reset was caused by the watchdog only.
 */
static void It_Iwdg_Check_IwdgReset( void )
{
    rcc_FlagState_t iwdgReset = RCC_FLAG_INACTIVE;
    rcc_FlagState_t swReset   = RCC_FLAG_ACTIVE;

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_ResetSource( RCC_RESET_SRC_IWDG, &iwdgReset ) );
    TEST_ASSERT_EQUAL_MESSAGE( RCC_FLAG_ACTIVE, iwdgReset, "Reset source is not IWDG" );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_ResetSource( RCC_RESET_SRC_SW, &swReset ) );
    TEST_ASSERT_EQUAL( RCC_FLAG_INACTIVE, swReset );
}


#if defined(STM32H7RS)
/**
 * \brief Early wakeup interrupt callback - counts calls and refreshes the watchdog
 *        (the MCU is not reset after the timeout).
 */
static void It_Iwdg_EarlyWakeupCallback( void )
{
    itIwdg_EarlyWakeupCnt++;
    (void)Iwdg_Set_Refresh();
}
#endif /* STM32H7RS */
