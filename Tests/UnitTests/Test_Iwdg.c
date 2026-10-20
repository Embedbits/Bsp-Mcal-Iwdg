/**
 * \author Mr.Nobody
 * \file Test_Iwdg.c
 * \ingroup Iwdg
 * \brief Unit tests of Independent Watchdog (IWDG) module.
 *
 * Iwdg.c is compiled unchanged with real LL drivers. IWDG registers are emulated
 * by RegMem, RCC (LSI frequency) and NVIC modules are mocked by CMock.
 *
 * \note Watchdog running flag (SR.ONF) is set by HW after start key. Tests preset
 *       it where the module waits for it. Key register is write only on HW,
 *       emulated register keeps the last written key.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Iwdg_Port.h"                      /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "Stm32_iwdg.h"                     /* IWDG registers definition      */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  Ut_Iwdg_NvicSetHandlerStub  ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static void                 Ut_Iwdg_EarlyWakeupCallback ( void );
static void                 Ut_Iwdg_Expect_LsiClk       ( rcc_FreqHz_t lsiClk );
static iwdg_Config_t        Ut_Iwdg_Get_Config          ( iwdg_Time_ms_t timeout, iwdg_Time_ms_t window, iwdg_Time_ms_t earlyWakeup );

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

/** Early wakeup time used by tests - 100 ms = 400 ticks with prescaler 8 */
#define UT_IWDG_EWI_MS                      ( 100u )

/** Early wakeup comparator value of UT_IWDG_EWI_MS */
#define UT_IWDG_EWI_REG                     ( 399u )

/** Longest timeout achievable with LSI 32 kHz (prescaler 1024, 4096 ticks) */
#define UT_IWDG_TIMEOUT_MAX_MS              ( 131072u )

/** Early wakeup interrupt priority used by tests */
#define UT_IWDG_IRQ_PRIO                    ( 2u )

/** Default timeout of Iwdg_Get_DefaultConfig */
#define UT_IWDG_DEFAULT_TIMEOUT_MS          ( 1000u )

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC */
static nvic_IsrCallback_t   utIwdg_Isr;

/** Count of early wakeup callback calls */
static uint32_t             utIwdg_CallbackCnt;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utIwdg_Isr         = NULL;
    utIwdg_CallbackCnt = 0u;

    Nvic_Set_PeriphIrq_Handler_Stub( Ut_Iwdg_NvicSetHandlerStub );
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

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Iwdg_Init() with timeout only configures prescaler and reload.
 *
 * \details Initializes 1000 ms timeout without window and early wakeup (LSI 32 kHz
 *          from RCC mock, SR.ONF preset as after start key).
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, NVIC IRQ deactivated.
 * - PR = prescaler 8, RLR = 3999 (4000 ticks).
 * - WINR = maximum (window disabled), EWCR = 0 (early wakeup disabled).
 * - Last key is "write access disable", no ISR registered.
 */
void Ut_Iwdg_Init_TimeoutOnly_ConfiguresPrescalerAndReload( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_ONF;     /* Watchdog running after start key */

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_PRESCALER_8,     IWDG->PR );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_RELOAD, IWDG->RLR );
    TEST_ASSERT_EQUAL_HEX32( IWDG_WINR_WIN,           IWDG->WINR );    /* Window disabled */
    TEST_ASSERT_EQUAL_HEX32( 0u,                      IWDG->EWCR );    /* Early wakeup disabled */
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR ); /* Write access disabled at the end */
    TEST_ASSERT_NULL( utIwdg_Isr );
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
    iwdg_Config_t config = Ut_Iwdg_Get_Config( 1u, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_ONF;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    /* 1 ms * 32 kHz / 4 = 8 ticks */
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_PRESCALER_4, IWDG->PR );
    TEST_ASSERT_EQUAL_UINT32( 7u, IWDG->RLR );
}


/**
 * \brief   Iwdg_Init() handles the longest timeout.
 *
 * \details Initializes 131072 ms timeout (prescaler 1024, 4096 ticks at 32 kHz).
 *
 * \par Expected results
 * - PR = prescaler 1024, RLR = maximum reload value.
 */
void Ut_Iwdg_Init_MaxTimeout_UsesLargestPrescaler( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MAX_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_ONF;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_PRESCALER_1024, IWDG->PR );
    TEST_ASSERT_EQUAL_UINT32( IWDG_RLR_RL, IWDG->RLR );
}


/**
 * \brief   Iwdg_Init() configures window and early wakeup interrupt.
 *
 * \details Initializes 1000 ms timeout, 200 ms window and 100 ms early wakeup with
 *          callback. NVIC priority 2 and activation are expected.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, WINR = 799, EWCR.EWIT = 399, EWCR.EWIE set.
 * - ISR registered in NVIC.
 */
void Ut_Iwdg_Init_WindowAndEarlyWakeup_ConfiguresAllRegisters( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, UT_IWDG_WINDOW_MS, UT_IWDG_EWI_MS );

    IWDG->SR = IWDG_SR_ONF;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, UT_IWDG_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_WINDOW_REG, IWDG->WINR );
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_EWI_REG,    IWDG->EWCR & IWDG_EWCR_EWIT );
    TEST_ASSERT_BITS_HIGH( IWDG_EWCR_EWIE, IWDG->EWCR );
    TEST_ASSERT_NOT_NULL( utIwdg_Isr );
}


/**
 * \brief   Iwdg_Init() rejects timeout above maximum.
 *
 * \details Initializes timeout of maximum + 100 ms.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, PR and RLR not written, watchdog not started.
 */
void Ut_Iwdg_Init_TimeoutNotAchievable_ReturnsErrorWithoutAccess( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MAX_MS + 100u, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->PR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->RLR );
    TEST_ASSERT_NOT_EQUAL( LL_IWDG_KEY_ENABLE, IWDG->KR );     /* Watchdog not started */
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
    TEST_ASSERT_NOT_EQUAL( LL_IWDG_KEY_ENABLE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() rejects window equal to timeout.
 *
 * \details Initializes 1000 ms timeout with 1000 ms window.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, watchdog not started.
 */
void Ut_Iwdg_Init_WindowNotShorterThanTimeout_ReturnsError( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );
    TEST_ASSERT_NOT_EQUAL( LL_IWDG_KEY_ENABLE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() rejects early wakeup longer than timeout.
 *
 * \details Initializes 1000 ms timeout with 1001 ms early wakeup.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, watchdog not started.
 */
void Ut_Iwdg_Init_EarlyWakeupNotShorterThanTimeout_ReturnsError( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, UT_IWDG_TIMEOUT_MS + 1u );

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );
    TEST_ASSERT_NOT_EQUAL( LL_IWDG_KEY_ENABLE, IWDG->KR );
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
    TEST_ASSERT_NOT_EQUAL( LL_IWDG_KEY_ENABLE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() reports error of LSI frequency read.
 *
 * \details Rcc_Get_PeriphClk() returns error.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR is returned.
 */
void Ut_Iwdg_Init_RccError_ReturnsError( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    Rcc_Get_PeriphClk_ExpectAndReturn( RCC_PERIPH_IWDG, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );
}


/**
 * \brief   Iwdg_Init() rejects NULL configuration.
 *
 * \details Calls Iwdg_Init() with NULL pointer.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, RCC and NVIC not called.
 */
void Ut_Iwdg_Init_NullConfig_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( NULL ) );
}


/**
 * \brief   Iwdg_Init() reports watchdog start not confirmed by HW.
 *
 * \details SR.ONF stays 0 after start key (timeout of waiting).
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, RLR not written, write access disabled again.
 */
void Ut_Iwdg_Init_WatchdogNotStarted_ReturnsErrorWithoutConfig( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    /* SR.ONF stays 0 - start is not confirmed by HW */
    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->RLR );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );
}


/**
 * \brief   Iwdg_Init() reports never ending prescaler update.
 *
 * \details Presets SR.ONF and SR.PVU (previous update never finishes).
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, PR and RLR not written.
 */
void Ut_Iwdg_Init_PrescalerUpdateInProgress_ReturnsErrorWithoutWrite( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_ONF | IWDG_SR_PVU;   /* Previous prescaler update never finishes */

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->PR );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->RLR );
}


/**
 * \brief   Iwdg_Init() reports NVIC error, watchdog stays configured.
 *
 * \details Initializes 1000 ms timeout with early wakeup, NVIC priority returns error.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, IRQ is not activated.
 * - RLR = 3999 (watchdog is configured, it can not be stopped).
 */
void Ut_Iwdg_Init_NvicPrioError_ReturnsErrorWatchdogRunning( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, UT_IWDG_EWI_MS );

    IWDG->SR = IWDG_SR_ONF;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, UT_IWDG_IRQ_PRIO, NVIC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Init( &config ) );

    /* Watchdog is configured (can not be stopped), interrupt stays disabled in NVIC */
    TEST_ASSERT_EQUAL_UINT32( UT_IWDG_TIMEOUT_RELOAD, IWDG->RLR );
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
 * \brief   Iwdg_Get_Timeout() handles other register values of prescaler 1024.
 *
 * \details Presets PR = 1001 .. 1111 one after another (divider 1024 as 1000,
 *          reference manual "Others"), RLR = maximum, LSI 32 kHz.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, timeout 131072 ms for every register value.
 */
void Ut_Iwdg_Get_Timeout_PrescalerRegAlias_UsesDivider1024( void )
{
    for( uint32_t prescReg = LL_IWDG_PRESCALER_1024 + 1u; IWDG_PR_PR >= prescReg; prescReg++ )
    {
        iwdg_Time_ms_t timeout = 0u;

        IWDG->PR  = prescReg;
        IWDG->RLR = IWDG_RLR_RL;

        Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );

        TEST_ASSERT_EQUAL_MESSAGE( IWDG_REQUEST_OK, Iwdg_Get_Timeout( &timeout ), "Prescaler register alias value" );
        TEST_ASSERT_EQUAL_UINT32_MESSAGE( UT_IWDG_TIMEOUT_MAX_MS, timeout, "Prescaler register alias value" );
    }
}


/**
 * \brief   Iwdg_Get_Timeout() rejects unknown prescaler register value.
 *
 * \details Presets PR = 16 (outside PR[3:0], possible only in emulated register).
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

/* ========================== STATE AND REFRESH ============================= */

/**
 * \brief   Iwdg_Get_State() reports SR.ONF.
 *
 * \details Reads state with ONF cleared, with ONF set and with NULL pointer.
 *
 * \par Expected results
 * - ONF = 0: INACTIVE; ONF = 1: ACTIVE; NULL pointer: IWDG_REQUEST_ERROR.
 */
void Ut_Iwdg_Get_State_ReadsOnFlag( void )
{
    iwdg_FunctionState_t state = IWDG_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_INACTIVE, state );

    IWDG->SR = IWDG_SR_ONF;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Get_State( &state ) );
    TEST_ASSERT_EQUAL( IWDG_FUNCTION_ACTIVE, state );

    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Get_State( NULL ) );
}


/**
 * \brief   Iwdg_Set_Refresh() of running watchdog writes reload key.
 *
 * \details Presets SR.ONF and refreshes the watchdog.
 *
 * \par Expected results
 * - IWDG_REQUEST_OK, KR = reload key.
 */
void Ut_Iwdg_Set_Refresh_Running_WritesReloadKey( void )
{
    IWDG->SR = IWDG_SR_ONF;

    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Set_Refresh() );
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_RELOAD, IWDG->KR );
}


/**
 * \brief   Iwdg_Set_Refresh() of not running watchdog is rejected.
 *
 * \details Refreshes the watchdog with SR.ONF = 0.
 *
 * \par Expected results
 * - IWDG_REQUEST_ERROR, KR not written.
 */
void Ut_Iwdg_Set_Refresh_NotRunning_ReturnsErrorWithoutWrite( void )
{
    TEST_ASSERT_EQUAL( IWDG_REQUEST_ERROR, Iwdg_Set_Refresh() );
    TEST_ASSERT_EQUAL_HEX32( 0u, IWDG->KR );
}


/**
 * \brief   Iwdg_Task() refreshes watchdog without window.
 *
 * \details Initializes 1000 ms timeout without window and calls Iwdg_Task().
 *
 * \par Expected results
 * - KR = reload key.
 */
void Ut_Iwdg_Task_WithoutWindow_RefreshesAutomatically( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_ONF;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    Iwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_RELOAD, IWDG->KR );
}


/**
 * \brief   Iwdg_Task() does not refresh watchdog with window.
 *
 * \details Initializes 1000 ms timeout with 200 ms window and calls Iwdg_Task()
 *          (refresh outside the window would reset MCU - left to application).
 *
 * \par Expected results
 * - KR keeps "write access disable" key (no refresh).
 */
void Ut_Iwdg_Task_WithWindow_DoesNotRefresh( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, UT_IWDG_WINDOW_MS, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_ONF;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    Iwdg_Task();

    /* Refresh outside the window would reset MCU - left to application */
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );
}


/**
 * \brief   Iwdg_Task() does not refresh after Iwdg_Deinit().
 *
 * \details Initializes the watchdog, deinitializes it (NVIC deactivation) and
 *          calls Iwdg_Task().
 *
 * \par Expected results
 * - Iwdg_Deinit() returns IWDG_REQUEST_OK, KR is not the reload key.
 */
void Ut_Iwdg_Task_AfterDeinit_DoesNotRefresh( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, IWDG_TIME_UNUSED );

    IWDG->SR = IWDG_SR_ONF;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Deinit() );

    Iwdg_Task();

    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );
}

/* ============================ INTERRUPT ================================== */

/**
 * \brief   ISR handles early wakeup flag.
 *
 * \details Initializes the watchdog with early wakeup, presets SR.EWIF and calls
 *          captured ISR.
 *
 * \par Expected results
 * - Callback called once.
 * - EWCR.EWIC written (flag acknowledged), write access disabled again.
 */
void Ut_Iwdg_Isr_EarlyWakeupFlag_ClearsFlagAndCallsCallback( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, UT_IWDG_EWI_MS );

    IWDG->SR = IWDG_SR_ONF;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, UT_IWDG_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    IWDG->SR |= IWDG_SR_EWIF;

    utIwdg_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utIwdg_CallbackCnt );
    TEST_ASSERT_BITS_HIGH( IWDG_EWCR_EWIC, IWDG->EWCR );                  /* Flag acknowledged */
    TEST_ASSERT_EQUAL_HEX32( LL_IWDG_KEY_WR_ACCESS_DISABLE, IWDG->KR );  /* Access disabled again */
}


/**
 * \brief   ISR without early wakeup flag does nothing.
 *
 * \details Initializes the watchdog with early wakeup and calls ISR without EWIF.
 *
 * \par Expected results
 * - Callback not called, EWCR.EWIC not written.
 */
void Ut_Iwdg_Isr_NoFlag_DoesNotCallCallback( void )
{
    iwdg_Config_t config = Ut_Iwdg_Get_Config( UT_IWDG_TIMEOUT_MS, IWDG_TIME_UNUSED, UT_IWDG_EWI_MS );

    IWDG->SR = IWDG_SR_ONF;

    Ut_Iwdg_Expect_LsiClk( UT_IWDG_LSI_HZ );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, UT_IWDG_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_IWDG, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( IWDG_REQUEST_OK, Iwdg_Init( &config ) );

    utIwdg_Isr();

    TEST_ASSERT_EQUAL_UINT32( 0u, utIwdg_CallbackCnt );
    TEST_ASSERT_BITS_LOW( IWDG_EWCR_EWIC, IWDG->EWCR );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/** Stub of Nvic_Set_PeriphIrq_Handler - stores registered IWDG ISR */
static nvic_RequestState_t Ut_Iwdg_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_IWDG, irqId );

    utIwdg_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/** Early wakeup user callback - counts calls */
static void Ut_Iwdg_EarlyWakeupCallback( void )
{
    utIwdg_CallbackCnt++;
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
