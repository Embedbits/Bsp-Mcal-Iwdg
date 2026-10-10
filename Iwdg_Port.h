/**
 * \author Mr.Nobody
 * \file Iwdg_Port.h
 * \ingroup Iwdg
 * \brief Iwdg module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef IWDG_IWDG_PORT_H
#define IWDG_IWDG_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Iwdg_Types.h"                     /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

iwdg_ModuleVersion_t        Iwdg_Get_ModuleVersion          ( void );

iwdg_RequestState_t         Iwdg_Init                       ( iwdg_Config_t * const config );
iwdg_RequestState_t         Iwdg_Deinit                     ( void );
void                        Iwdg_Task                       ( void );

iwdg_RequestState_t         Iwdg_Get_DefaultConfig          ( iwdg_Config_t * const config );

/* ------------------------------ Watchdog state ---------------------------- */

iwdg_RequestState_t         Iwdg_Get_State                  ( iwdg_FunctionState_t * const state );
iwdg_RequestState_t         Iwdg_Get_Timeout                ( iwdg_Time_ms_t * const timeout );

/* ----------------------------- Manual refresh ----------------------------- */

iwdg_RequestState_t         Iwdg_Set_Refresh                ( void );

#ifdef __cplusplus
}
#endif

#endif /* IWDG_IWDG_PORT_H */

