/**
 * \author Mr.Nobody
 * \file Iwdg.h
 * \ingroup Iwdg
 * \brief Iwdg module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 *
 */

#ifndef IWDG_IWDG_H
#define IWDG_IWDG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Iwdg_Types.h"            /* Module types definitions       */
#include "Stm32_iwdg.h"            /* IWDG LL (prescaler values)     */
/* ============================= TYPEDEFS =================================== */

/** \brief Prescaler values enumeration */
typedef enum
{
    IWDG_PRESCALER_DIV_4 = 0u, /**< Prescaler divider 4               */
    IWDG_PRESCALER_DIV_8,      /**< Prescaler divider 8               */
    IWDG_PRESCALER_DIV_16,     /**< Prescaler divider 16              */
    IWDG_PRESCALER_DIV_32,     /**< Prescaler divider 32              */
    IWDG_PRESCALER_DIV_64,     /**< Prescaler divider 64              */
    IWDG_PRESCALER_DIV_128,    /**< Prescaler divider 128             */
    IWDG_PRESCALER_DIV_256,    /**< Prescaler divider 256             */
#if defined(LL_IWDG_PRESCALER_1024)
    IWDG_PRESCALER_DIV_512,    /**< Prescaler divider 512 (STM32H7R / H7S)  */
    IWDG_PRESCALER_DIV_1024,   /**< Prescaler divider 1024 (STM32H7R / H7S) */
#endif
    IWDG_PRESCALER_CNT         /**< Number of prescaler divider items */
}   iwdg_Prescaler_t;


/** \brief Type used to store prescaler divider value */
typedef uint32_t iwdg_Divider_t;


/** \brief Type used to store number of watchdog counter ticks */
typedef uint32_t iwdg_Ticks_t;


/** \brief Prescaler configuration item */
typedef struct
{
    uint32_t       RegVal;  /**< LL prescaler register value */
    iwdg_Divider_t Divider; /**< Prescaler divider value     */
}   iwdg_PrescalerConfig_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

#ifdef __cplusplus
}
#endif

#endif /* IWDG_IWDG_H */
