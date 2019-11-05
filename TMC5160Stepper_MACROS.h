#ifndef TMC5160Stepper_MACROS_H
#define TMC5160Stepper_MACROS_H
#include "TMC5160Stepper.h"
#include "TMC5160Stepper_REGDEFS.h"

#define TMC_WRITE_REG(R) 	send5160(TMC5160_WRITE|REG_##R, &R##_sr);

#define TMC_READ_REG(R)   	send5160(TMC5160_READ|REG_##R, &R##_sr); return R##_sr;

#define TMC_READ_REG_R(R)   tmp_sr=0; send5160(TMC5160_READ|REG_##R, &tmp_sr); return tmp_sr;

#define TMC_MOD_REG(REG, SETTING) 	REG##_sr &= ~SETTING##_bm; \
								REG##_sr |= ((uint32_t)B<<SETTING##_bp)&SETTING##_bm; \
								TMC_WRITE_REG(REG);
								

#define TMC_GET_BYTE(REG, SETTING) 	return (REG()&SETTING##_bm) >> SETTING##_bp;

#define TMC_GET_BYTE_R(REG, SETTING) return (REG()&SETTING##_bm) >> SETTING##_bp;

#define TMC_GET_BIT(REG, SETTING) 	return (bool)((REG()&SETTING##_bm) >> SETTING##_bp);

#endif
