/*
 * flash.h
 *
 *  Created on: Jun 11, 2019
 *      Author: cinqu
 */

#ifndef FLASH_H_
#define FLASH_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

// flash use address ( sector11 )
#define START_ADDRESS  0x80E0000 //sentor11 start address
#define END_ADDRESS  0x80FFFFF

 HAL_StatusTypeDef eeprom_enable_write(void);
 HAL_StatusTypeDef eeprom_disable_write(void);
 HAL_StatusTypeDef eeprom_write_halfword(uint32_t, uint16_t);
 HAL_StatusTypeDef eeprom_write_word(uint32_t, uint32_t);
 uint16_t eeprom_read_halfword(uint32_t);
 uint32_t eeprom_read_word(uint32_t);

#endif /* FLASH_H_ */
