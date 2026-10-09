/**
 * \author Mr.Nobody
 * \file ItTest_Iwdg.c
 * \ingroup Iwdg
 * \brief Integration tests of Independent watchdog (IWDG) module on target.
 *
 * Iwdg module runs on the MCU together with real RCC and NVIC modules and
 * hardware (LSI clock). Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): watchdog start, register update synchronization
 * with LSI clock, reset of the MCU after timeout, refresh, window violation and
 * early wakeup interrupt.
 *
 * Tests expecting reset of the MCU use stages of the integration testing
 * (\c IntegrationTesting_Set_ResetExpected), stage 1 checks the reset source
 * through RCC module. Running watchdog is stopped by the system reset done by
 * the test runner after every test case.
 *
 * Used resources:
 * - IWDG, LSI clock
 * - TIM2 - reference time base 10 kHz (\ref It_Iwdg_Get_Time_ms), no external wiring
 *
 * \note LSI frequency tolerance is covered by tolerances of time checks.
 *
 * \note STM32U5 IWDG has no running flag - watchdog state is kept by the module
 *       (started by Iwdg_Init() since the last reset) together with option byte
 *       IWDG_SW (hardware watchdog mode is not selected on test boards).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Iwdg_Port.h"                      /* Module under test              */
#include "Rcc_Port.h"                       /* Reset source                   */
#include "Tim_Port.h"                       /* Reference time base            */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void             It_Iwdg_Init_RefTime    ( void );
static iwdg_Time_ms_t   It_Iwdg_Get_Time_ms     ( void );
static void             It_Iwdg_Wait_Until_ms   ( iwdg_Time_ms_t time_ms, iwdg_FunctionState_t taskState );
static void             It_Iwdg_Init_Watchdog   ( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window, iwdg_Time_ms_t earlyWakeup );
static void             It_Iwdg_Check_IwdgReset ( void );

static void             It_Iwdg_EarlyWakeupCallback( void );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Reference timer (32-bit) */
#define IT_IWDG_REF_TIM                     ( TIM_PERIPH_2 )

/** Counter frequency of the reference timer [Hz] */
#define IT_IWDG_REF_FREQ_HZ                 ( 10000u )

/** Reference timer counts per millisecond */
#define IT_IWDG_REF_COUNTS_PER_MS           ( IT_IWDG_REF_FREQ_HZ / 1000u )

/** Watchdog timeout used by tests [ms] */
#define IT_IWDG_TIMEOUT_MS                  ( 100u )

/** Time without refresh after which the reset must have occurred [ms] */
#define IT_IWDG_RESET_WAIT_MS               ( 1000u )

/** Relative tolerance of watchdog times (LSI accuracy, register synchronization) [%] */
#define IT_IWDG_TIME_TOL_PCT                ( 20u )

/* ============================== MACROS ==================================== */

/** Absolute tolerance of watchdog time [ms] */
#define IT_IWDG_TIME_TOL( time_ms )         ( ( ( time_ms ) * IT_IWDG_TIME_TOL_PCT ) / 100u )

/* ========================== LOCAL VARIABLES =============================== */

/** Count of early wakeup callback calls */
static volatile uint32_t                itIwdg_EarlyWakeupCnt   = 0u;

/** Reference time of the first early wakeup callback call [ms] */
static volatile iwdg_Time_ms_t          itIwdg_EarlyWakeupTime  = 0u;

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itIwdg_EarlyWakeupCnt  = 0u;
    itIwdg_EarlyWakeupTime = 0u;

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
 * \brief   Iwdg_Init() rejects timeout above the hardware maximum.
 *
 * \details Initializes the watchdog with timeout 200 s (maximum is 4096 * 1024 / LSI,
 *          approx. 131 s).
 *
 * \par Expected results
 * - Iwdg_Init() returns IWDG_REQUEST_ERROR.
 * - Watchdog is not started (state IWDG_FUNCTION_INACTIVE).
 */
void It_Iwdg_Init_TimeoutNotAchievable_ReturnsErrorNotStarted( void )
{
    iwdg_Config_t        config;
    iwdg_FunctionState_t state = IWDG_FUNCTION_ACTIVE;

    /* Maximal timeout is 4096 * 1024 / LSI (~131 s) */
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_DefaultConfig( &config ) );
    config.Timeout = 200000u;

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
        It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

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
    It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

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

    It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

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
        It_Iwdg_Init_Watchdog( IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

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
        It_Iwdg_Init_Watchdog( 2u * IT_IWDG_TIMEOUT_MS, IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

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
 *          called continuously. The watchdog is refreshed manually at 150 ms, 300 ms
 *          and 450 ms (middle of the window).
 *
 * \par Expected results
 * - No reset of the MCU, every Iwdg_Set_Refresh() returns IWDG_REQUEST_OK.
 */
void It_Iwdg_Set_Refresh_InsideWindow_NoReset( void )
{
    /* Refresh allowed only in the last 100 ms of 200 ms timeout */
    It_Iwdg_Init_Watchdog( 2u * IT_IWDG_TIMEOUT_MS, IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

    /* Refresh in the middle of the window (150 ms after refresh) */
    It_Iwdg_Wait_Until_ms( 150u, IWDG_FUNCTION_ACTIVE );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );

    It_Iwdg_Wait_Until_ms( 300u, IWDG_FUNCTION_ACTIVE );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );

    It_Iwdg_Wait_Until_ms( 450u, IWDG_FUNCTION_ACTIVE );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
}

/*------------------------------ Early wakeup --------------------------------*/

/**
 * \brief   Early wakeup interrupt calls the callback before timeout.
 *
 * \details Starts the watchdog with 200 ms timeout and early wakeup 50 ms before
 *          timeout, Iwdg_Task() is not called. The callback stores the time of the
 *          first call and refreshes the watchdog. Waits for the callback up to 1000 ms.
 *
 * \par Expected results
 * - Callback is called at least once, no reset of the MCU.
 * - First call at 150 ms +- 20 %.
 */
void It_Iwdg_Init_EarlyWakeup_CallbackCalledBeforeTimeout( void )
{
    /* Early wakeup interrupt 50 ms before 200 ms timeout */
    const iwdg_Time_ms_t expectedTime = ( 2u * IT_IWDG_TIMEOUT_MS ) - ( IT_IWDG_TIMEOUT_MS / 2u );

    It_Iwdg_Init_Watchdog( 2u * IT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IT_IWDG_TIMEOUT_MS / 2u );

    /* Callback refreshes the watchdog - no reset */
    while( ( 0u                    == itIwdg_EarlyWakeupCnt ) &&
           ( IT_IWDG_RESET_WAIT_MS >  It_Iwdg_Get_Time_ms() )    )
    {
        /* Wait for early wakeup interrupt */
    }

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32_MESSAGE( 1u, itIwdg_EarlyWakeupCnt, "Early wakeup callback not called" );
    TEST_ASSERT_UINT32_WITHIN( IT_IWDG_TIME_TOL( expectedTime ), expectedTime, itIwdg_EarlyWakeupTime );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes and starts reference time base (TIM2, 10 kHz, free running 32-bit).
 */
static void It_Iwdg_Init_RefTime( void )
{
    tim_PeriphConfig_t timConfig;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_DefaultConfig( &timConfig ) );
    timConfig.PeriphId         = IT_IWDG_REF_TIM;
    timConfig.TimerFrequency   = IT_IWDG_REF_FREQ_HZ;
    timConfig.RefreshFrequency = 1u;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Init( &timConfig ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Set_Period( IT_IWDG_REF_TIM, 0xFFFFFFFFu ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Start( IT_IWDG_REF_TIM ) );
}


/**
 * \brief Returns reference time since \ref It_Iwdg_Init_RefTime or \ref It_Iwdg_Init_Watchdog.
 *
 * \return Reference time [ms]
 */
static iwdg_Time_ms_t It_Iwdg_Get_Time_ms( void )
{
    tim_Counter_t counter = 0u;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_Counter( IT_IWDG_REF_TIM, &counter ) );

    return ( counter / IT_IWDG_REF_COUNTS_PER_MS );
}


/**
 * \brief Waits until reference time is reached.
 *
 * \param time_ms   [in]: Reference time [ms]
 * \param taskState [in]: \ref IWDG_FUNCTION_ACTIVE - \ref Iwdg_Task is called during waiting
 */
static void It_Iwdg_Wait_Until_ms( iwdg_Time_ms_t time_ms, iwdg_FunctionState_t taskState )
{
    while( time_ms > It_Iwdg_Get_Time_ms() )
    {
        if( IWDG_FUNCTION_ACTIVE == taskState )
        {
            Iwdg_Task();
        }
        else
        {
            /* Watchdog is not refreshed */
        }
    }
}


/**
 * \brief Initializes watchdog and restarts reference time (time 0 = watchdog start).
 *
 * \param timeout     [in]: Watchdog timeout [ms]
 * \param window      [in]: Window [ms] or \ref IWDG_TIME_UNUSED
 * \param earlyWakeup [in]: Early wakeup time before timeout [ms] or \ref IWDG_TIME_UNUSED
 */
static void It_Iwdg_Init_Watchdog( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window, iwdg_Time_ms_t earlyWakeup )
{
    iwdg_Config_t config;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_DefaultConfig( &config ) );
    config.Timeout        = timeout;
    config.Window         = window;
    config.EarlyWakeup    = earlyWakeup;
    config.EarlyWakeupIsr = It_Iwdg_EarlyWakeupCallback;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Set_Counter( IT_IWDG_REF_TIM, 0u ) );
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


/**
 * \brief Early wakeup callback - stores time of the first call and refreshes the watchdog.
 */
static void It_Iwdg_EarlyWakeupCallback( void )
{
    if( 0u == itIwdg_EarlyWakeupCnt )
    {
        itIwdg_EarlyWakeupTime = It_Iwdg_Get_Time_ms();
    }
    else
    {
        /* Time of the first call is kept */
    }

    itIwdg_EarlyWakeupCnt++;

    (void)Iwdg_Set_Refresh();
}
