/**
 * \defgroup Iwdg Iwdg
 * \brief Iwdg module
 */

/**
 * \author Mr.Nobody
 * \file Iwdg_Types.h
 * \ingroup Iwdg
 * \brief Iwdg module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 */

#ifndef IWDG_IWDG_TYPES_H
#define IWDG_IWDG_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define IWDG_NULL_PTR                        ( ( void* ) 0u )

/** Time value used to signal that optional time based function is not used
 *  (window, early wakeup interrupt) */
#define IWDG_TIME_UNUSED                     ( 0u )

/* ========================== EXPORTED MACROS =============================== */

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t iwdg_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t iwdg_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t iwdg_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    iwdg_MajorVersion_t Major; /**< Major version */
    iwdg_MinorVersion_t Minor; /**< Minor version */
    iwdg_PatchVersion_t Patch; /**< Patch version */
}   iwdg_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    IWDG_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    IWDG_FUNCTION_ACTIVE         /**< Function status is active   */
}   iwdg_FunctionState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    IWDG_REQUEST_ERROR = 0u, /**< Processing request failed  */
    IWDG_REQUEST_OK          /**< Processing request succeed */
}   iwdg_RequestState_t;


/** Flag states enumeration */
typedef enum
{
    IWDG_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    IWDG_FLAG_ACTIVE         /**< Active flag state   */
}   iwdg_FlagState_t;


/** \brief Type used to store time values in milliseconds [0.001s] */
typedef uint32_t iwdg_Time_ms_t;


/** \brief Interrupt priority type definition */
typedef uint32_t iwdg_IrqPrio_t;


/** \brief Early wakeup interrupt user callback.
 *
 * Called from interrupt context when the watchdog counter reaches the early
 * wakeup threshold. The flag is already acknowledged by the module.
 */
typedef void ( iwdg_EarlyWakeupIsrCallback_t )( void );


/** \brief Type used to store IWDG configuration */
typedef struct
{
    /** Required watchdog timeout in ms. Prescaler and reload value are
     *  calculated by the module, see \ref Iwdg_Get_Timeout for the achieved value. */
    iwdg_Time_ms_t                 Timeout;

    /** Window length in ms before timeout, when refresh is allowed. Refresh
     *  before the window resets the MCU. \ref IWDG_TIME_UNUSED disables the window.
     *  Must be shorter than \ref iwdg_Config_t::Timeout. */
    iwdg_Time_ms_t                 Window;

    /** Time in ms before timeout, when early wakeup interrupt is triggered.
     *  \ref IWDG_TIME_UNUSED disables the interrupt. Must be shorter than
     *  \ref iwdg_Config_t::Timeout. */
    iwdg_Time_ms_t                 EarlyWakeup;

    /** Early wakeup interrupt user callback, can be NULL */
    iwdg_EarlyWakeupIsrCallback_t *EarlyWakeupIsr;

    /** Early wakeup interrupt priority (0 - highest priority) */
    iwdg_IrqPrio_t                 IrqPriority;
}   iwdg_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */


#endif /* IWDG_IWDG_TYPES_H */
