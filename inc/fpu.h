/*
 * fpu.h
 *
 *  Created on: 15 feb. 2025
 *      Author: Ludo
 */

#ifndef STM32G4XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32g4xx_drivers_flags.h"
#endif

#ifndef STM32G4XX_DRIVERS_DISABLE

#ifndef __FPU_H__
#define __FPU_H__

/*** FPU functions ***/

/*!******************************************************************
 * \fn void FPU_init(void)
 * \brief Enable floating point unit.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void FPU_init(void);

/*!******************************************************************
 * \fn void FPU_de_init(void)
 * \brief Disable floating point unit.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void FPU_de_init(void);

#endif /* __FPU_H__ */

#endif /* STM32G4XX_DRIVERS_DISABLE */
