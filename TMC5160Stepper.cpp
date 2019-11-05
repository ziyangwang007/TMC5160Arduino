#include "TMC5160Stepper.h"
#include "TMC5160Stepper_MACROS.h"
#include <SPI.h>
#include "SW_SPI.h"


TMC5160Stepper::TMC5160Stepper(uint16_t pinEN, uint16_t pinDIR, uint16_t pinStep, uint16_t pinCS) :
	_pinEN(pinEN),
	_pinSTEP(pinStep),
	_pinCS(pinCS),
	_pinDIR(pinDIR),
	uses_sw_spi(false)
	{}

TMC5160Stepper::TMC5160Stepper(uint16_t pinEN, uint16_t pinDIR, uint16_t pinStep, uint16_t pinCS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK) :
	_pinEN(pinEN),
	_pinSTEP(pinStep),
	_pinCS(pinCS),
	_pinDIR(pinDIR),
	uses_sw_spi(true)
	{ TMC_SW_SPI.setPins(pinMOSI, pinMISO, pinSCK); }

void TMC5160Stepper::begin() {

	//set pins
	if (_pinEN != 0xFFFF) {
		pinMode(_pinEN, OUTPUT);
		digitalWrite(_pinEN, HIGH); //deactivate driver (LOW active)
	}
	if (_pinDIR != 0xFFFF) {
		pinMode(_pinDIR, OUTPUT);
	}
	if (_pinSTEP != 0xFFFF) {
		pinMode(_pinSTEP, OUTPUT);
		digitalWrite(_pinSTEP, LOW);
	}

	pinMode(_pinCS, OUTPUT);
	digitalWrite(_pinCS, HIGH);

	if (uses_sw_spi) TMC_SW_SPI.init();

	// Reset registers
	GCONF(GCONF_sr);
	SHORT_CONF(SHORT_CONF_sr);
	DRV_CONF(DRV_CONF_sr);
	IHOLD_IRUN(IHOLD_IRUN_sr);
	TPOWERDOWN(TPOWERDOWN_sr);
	TPWMTHRS(TPWMTHRS_sr);
	TCOOLTHRS(TCOOLTHRS_sr);
	THIGH(THIGH_sr);
	VDCMIN(VDCMIN_sr);
	CHOPCONF(CHOPCONF_sr);
	COOLCONF(COOLCONF_sr);
	PWMCONF(PWMCONF_sr);
}

uint8_t TMC5160Stepper::test_connection() {
	uint32_t drv_status = DRV_STATUS();
	switch (drv_status) {
	    case 0xFFFFFFFF: return 1;
	    case 0: return 2;
	    default: return 0;
	}
}

void TMC5160Stepper::send5160(uint8_t addressByte, uint32_t *config) {
	if (uses_sw_spi) {
		digitalWrite(_pinCS, LOW);

		status_response = TMC_SW_SPI.transfer(addressByte & 0xFF); // s =

		if (addressByte >> 7) { // Check if WRITE command
			TMC_SW_SPI.transfer((*config >> 24) & 0xFF);
			TMC_SW_SPI.transfer((*config >> 16) & 0xFF);
			TMC_SW_SPI.transfer((*config >>  8) & 0xFF);
			TMC_SW_SPI.transfer(*config & 0xFF);
		} else { // READ command
			TMC_SW_SPI.transfer16(0x0000); // Clear SPI
			TMC_SW_SPI.transfer16(0x0000);
			digitalWrite(_pinCS, HIGH);
			digitalWrite(_pinCS, LOW);

			TMC_SW_SPI.transfer(addressByte & 0xFF); // Send the address byte again
			*config = TMC_SW_SPI.transfer(0x00);
			*config <<= 8;
			*config|= TMC_SW_SPI.transfer(0x00);
			*config <<= 8;
			*config|= TMC_SW_SPI.transfer(0x00);
			*config <<= 8;
			*config|= TMC_SW_SPI.transfer(0x00);
		}

		digitalWrite(_pinCS, HIGH);
	} else {
		SPI.begin();
		SPI.beginTransaction(SPISettings(16000000/8, MSBFIRST, SPI_MODE3));
		digitalWrite(_pinCS, LOW);

		status_response = SPI.transfer(addressByte & 0xFF); 

		if (addressByte >> 7) { // Check if WRITE command
			SPI.transfer((*config >> 24) & 0xFF);
			SPI.transfer((*config >> 16) & 0xFF);
			SPI.transfer((*config >>  8) & 0xFF);
			SPI.transfer(*config & 0xFF);
		} else { // READ command
			SPI.transfer16(0x0000); // Clear SPI
			SPI.transfer16(0x0000);
			digitalWrite(_pinCS, HIGH);
			digitalWrite(_pinCS, LOW);

			SPI.transfer(addressByte & 0xFF); // Send the address byte again
			*config = SPI.transfer(0x00);
			*config <<= 8;
			*config|= SPI.transfer(0x00);
			*config <<= 8;
			*config|= SPI.transfer(0x00);
			*config <<= 8;
			*config|= SPI.transfer(0x00);
		}

		digitalWrite(_pinCS, HIGH);
		SPI.endTransaction();
	}
}

///////////////////////////////////////////////////////////////////////////////////////
// R+W: GCONF
uint32_t TMC5160Stepper::GCONF() { TMC_READ_REG(GCONF); }
void TMC5160Stepper::GCONF(uint32_t input) {
	GCONF_sr = input;
	TMC_WRITE_REG(GCONF);
}
void TMC5160Stepper::recalibrate(bool B)			{ TMC_MOD_REG(GCONF, RECALIBRATE);			}
void TMC5160Stepper::faststandstill(bool B)			{ TMC_MOD_REG(GCONF, FASTSTANDSTILL);		}
void TMC5160Stepper::en_pwm_mode(bool B)			{ TMC_MOD_REG(GCONF, EN_PWM_MODE);			}
void TMC5160Stepper::multistep_filt(bool B)			{ TMC_MOD_REG(GCONF, MULTISTEP_FILT);		}
void TMC5160Stepper::shaft(bool B) 					{ TMC_MOD_REG(GCONF, SHAFT);				}
void TMC5160Stepper::diag0_error(bool B) 			{ TMC_MOD_REG(GCONF, DIAG0_ERROR);			}
void TMC5160Stepper::diag0_otpw(bool B) 			{ TMC_MOD_REG(GCONF, DIAG0_OTPW);			}
void TMC5160Stepper::diag0_stall(bool B) 			{ TMC_MOD_REG(GCONF, DIAG0_STALL);			}
void TMC5160Stepper::diag1_stall(bool B) 			{ TMC_MOD_REG(GCONF, DIAG1_STALL);			}
void TMC5160Stepper::diag1_index(bool B) 			{ TMC_MOD_REG(GCONF, DIAG1_INDEX);			}
void TMC5160Stepper::diag1_onstate(bool B) 			{ TMC_MOD_REG(GCONF, DIAG1_ONSTATE);		}
void TMC5160Stepper::diag1_steps_skipped(bool B) 	{ TMC_MOD_REG(GCONF, DIAG1_STEPS_SKIPPED);	}
void TMC5160Stepper::diag0_int_pushpull(bool B) 	{ TMC_MOD_REG(GCONF, DIAG0_INT_PUSHPULL);	}
void TMC5160Stepper::diag1_poscomp_pushpull(bool B) { TMC_MOD_REG(GCONF, DIAG1_POSCOMP_PUSHPULL);}
void TMC5160Stepper::small_hysteresis(bool B) 		{ TMC_MOD_REG(GCONF, SMALL_HYSTERESIS);		}
void TMC5160Stepper::stop_enable(bool B) 			{ TMC_MOD_REG(GCONF, STOP_ENABLE);			}
void TMC5160Stepper::direct_mode(bool B) 			{ TMC_MOD_REG(GCONF, DIRECT_MODE);			}
void TMC5160Stepper::test_mode(bool B) 				{ TMC_MOD_REG(GCONF, TEST_MODE);			}
bool TMC5160Stepper::recalibrate()					{ TMC_GET_BYTE(GCONF, RECALIBRATE);			}
bool TMC5160Stepper::faststandstill()				{ TMC_GET_BYTE(GCONF, FASTSTANDSTILL);		}
bool TMC5160Stepper::en_pwm_mode()					{ TMC_GET_BYTE(GCONF, EN_PWM_MODE);			}
bool TMC5160Stepper::multistep_filt()				{ TMC_GET_BYTE(GCONF, MULTISTEP_FILT);		}
bool TMC5160Stepper::shaft() 						{ TMC_GET_BYTE(GCONF, SHAFT);				}
bool TMC5160Stepper::diag0_error() 					{ TMC_GET_BYTE(GCONF, DIAG0_ERROR);			}
bool TMC5160Stepper::diag0_otpw() 					{ TMC_GET_BYTE(GCONF, DIAG0_OTPW);			}
bool TMC5160Stepper::diag0_stall() 					{ TMC_GET_BYTE(GCONF, DIAG0_STALL);			}
bool TMC5160Stepper::diag1_stall() 					{ TMC_GET_BYTE(GCONF, DIAG1_STALL);			}
bool TMC5160Stepper::diag1_index() 					{ TMC_GET_BYTE(GCONF, DIAG1_INDEX);			}
bool TMC5160Stepper::diag1_onstate() 				{ TMC_GET_BYTE(GCONF, DIAG1_ONSTATE);		}
bool TMC5160Stepper::diag1_steps_skipped() 			{ TMC_GET_BYTE(GCONF, DIAG1_STEPS_SKIPPED);	}
bool TMC5160Stepper::diag0_int_pushpull() 			{ TMC_GET_BYTE(GCONF, DIAG0_INT_PUSHPULL);	}
bool TMC5160Stepper::diag1_poscomp_pushpull() 		{ TMC_GET_BYTE(GCONF, DIAG1_POSCOMP_PUSHPULL);}
bool TMC5160Stepper::small_hysteresis() 			{ TMC_GET_BYTE(GCONF, SMALL_HYSTERESIS);	}
bool TMC5160Stepper::stop_enable() 					{ TMC_GET_BYTE(GCONF, STOP_ENABLE);			}
bool TMC5160Stepper::direct_mode() 					{ TMC_GET_BYTE(GCONF, DIRECT_MODE);			}
bool TMC5160Stepper::test_mode() 					{ TMC_GET_BYTE(GCONF, TEST_MODE);			}
///////////////////////////////////////////////////////////////////////////////////////
// R+C: GSTAT
void 	TMC5160Stepper::GSTAT(uint8_t input){
	GSTAT_sr = input;
	TMC_WRITE_REG(GSTAT);
}
uint8_t TMC5160Stepper::GSTAT()			 	{ TMC_READ_REG_R(GSTAT); 		}
bool 	TMC5160Stepper::reset()				{ TMC_GET_BYTE(GSTAT, RESET);	}
bool 	TMC5160Stepper::drv_err()			{ TMC_GET_BYTE(GSTAT, DRV_ERR);	}
bool 	TMC5160Stepper::uv_cp()				{ TMC_GET_BYTE(GSTAT, UV_CP);	}
///////////////////////////////////////////////////////////////////////////////////////
// IFCNT
///////////////////////////////////////////////////////////////////////////////////////
// SLAVECONF
///////////////////////////////////////////////////////////////////////////////////////
// R: IOIN
uint32_t 	TMC5160Stepper::IOIN() 			{ TMC_READ_REG_R(IOIN); 				}
bool 		TMC5160Stepper::step()			{ TMC_GET_BYTE_R(IOIN, STEP);			}
bool 		TMC5160Stepper::dir()			{ TMC_GET_BYTE_R(IOIN, DIR);			}
bool 		TMC5160Stepper::dcen_cfg4()		{ TMC_GET_BYTE_R(IOIN, DCEN_CFG4);		}
bool 		TMC5160Stepper::dcin_cfg5()		{ TMC_GET_BYTE_R(IOIN, DCIN_CFG5);		}
bool 		TMC5160Stepper::drv_enn_cfg6()	{ TMC_GET_BYTE_R(IOIN, DRV_ENN_CFG6);	}
bool 		TMC5160Stepper::dco()			{ TMC_GET_BYTE_R(IOIN, DCO);			}
uint8_t 	TMC5160Stepper::version() 		{ TMC_GET_BYTE_R(IOIN, VERSION);		}
///////////////////////////////////////////////////////////////////////////////////////
// OUTPUT
///////////////////////////////////////////////////////////////////////////////////////
// X_COMPARE
///////////////////////////////////////////////////////////////////////////////////////
// OTP_PROG
///////////////////////////////////////////////////////////////////////////////////////
// OTP_READ
///////////////////////////////////////////////////////////////////////////////////////
// FACTORY_CONF
///////////////////////////////////////////////////////////////////////////////////////
// W: SHORT_CONF
uint32_t TMC5160Stepper::SHORT_CONF() { return SHORT_CONF_sr; }
void TMC5160Stepper::SHORT_CONF(uint32_t input) {
	SHORT_CONF_sr = input;
	TMC_WRITE_REG(SHORT_CONF);
}
void TMC5160Stepper::s2vs_level(	uint8_t B )	{ TMC_MOD_REG(SHORT_CONF, S2VS_LEVEL);		}
void TMC5160Stepper::s2g_level(		uint8_t B )	{ TMC_MOD_REG(SHORT_CONF, S2G_LEVEL);		}
void TMC5160Stepper::shortfilter(	uint8_t B )	{ TMC_MOD_REG(SHORT_CONF, SHORTFILTER);		}
void TMC5160Stepper::shortdelay(	uint8_t B )	{ TMC_MOD_REG(SHORT_CONF, SHORTDELAY);		}
///////////////////////////////////////////////////////////////////////////////////////
// W: DRV_CONF
uint32_t TMC5160Stepper::DRV_CONF() { return DRV_CONF_sr; }
void TMC5160Stepper::DRV_CONF(uint32_t input) {
	DRV_CONF_sr = input;
	TMC_WRITE_REG(DRV_CONF);
}
void TMC5160Stepper::bbmtime(	uint8_t B )		{ TMC_MOD_REG(DRV_CONF, BBMTIME);		}
void TMC5160Stepper::bbmclks(	uint8_t B )		{ TMC_MOD_REG(DRV_CONF, BBMCLKS);		}
void TMC5160Stepper::otselect(	uint8_t B )		{ TMC_MOD_REG(DRV_CONF, OTSELECT);		}
void TMC5160Stepper::drvstrength(	uint8_t B )	{ TMC_MOD_REG(DRV_CONF, DRVSTRENGTH);	}
void TMC5160Stepper::filt_isense(	uint8_t B )	{ TMC_MOD_REG(DRV_CONF, FILT_ISENSE);	}
///////////////////////////////////////////////////////////////////////////////////////
// W: GLOBALSCALER
uint32_t TMC5160Stepper::GLOBALSCALER() { return GLOBALSCALER_sr; }
void TMC5160Stepper::GLOBALSCALER(uint8_t input) {
	GLOBALSCALER_sr = input;
	TMC_WRITE_REG(GLOBALSCALER);
}
///////////////////////////////////////////////////////////////////////////////////////// W:IHOLD_IRUN
// IHOLD_IRUN
void TMC5160Stepper::IHOLD_IRUN(uint32_t input) {
	IHOLD_IRUN_sr = input;
	TMC_WRITE_REG(IHOLD_IRUN);
}
uint32_t TMC5160Stepper::IHOLD_IRUN() { return IHOLD_IRUN_sr; }

void 	TMC5160Stepper::ihold(uint8_t B) 		{ TMC_MOD_REG(IHOLD_IRUN, IHOLD);		}
void 	TMC5160Stepper::irun(uint8_t B)  		{ TMC_MOD_REG(IHOLD_IRUN, IRUN); 		}
void 	TMC5160Stepper::iholddelay(uint8_t B)	{ TMC_MOD_REG(IHOLD_IRUN, IHOLDDELAY); 	}
uint8_t TMC5160Stepper::ihold() 				{ TMC_GET_BYTE(IHOLD_IRUN, IHOLD);		}
uint8_t TMC5160Stepper::irun()  				{ TMC_GET_BYTE(IHOLD_IRUN, IRUN); 		}
uint8_t TMC5160Stepper::iholddelay()  			{ TMC_GET_BYTE(IHOLD_IRUN, IHOLDDELAY);	}
///////////////////////////////////////////////////////////////////////////////////////
// W: TPOWERDOWN
uint8_t TMC5160Stepper::TPOWERDOWN() { return TPOWERDOWN_sr; }
void TMC5160Stepper::TPOWERDOWN(uint8_t input) {
	TPOWERDOWN_sr = input;
	TMC_WRITE_REG(TPOWERDOWN);
}
///////////////////////////////////////////////////////////////////////////////////////
// R: TSTEP
uint32_t TMC5160Stepper::TSTEP() { TMC_READ_REG_R(TSTEP); }
///////////////////////////////////////////////////////////////////////////////////////
// W: TPWMTHRS
uint32_t TMC5160Stepper::TPWMTHRS() { return TPWMTHRS_sr; }
void TMC5160Stepper::TPWMTHRS(uint32_t input) {
	TPWMTHRS_sr = input;
	TMC_WRITE_REG(TPWMTHRS);
}
///////////////////////////////////////////////////////////////////////////////////////
// W: TCOOLTHRS
uint32_t TMC5160Stepper::TCOOLTHRS() { return TCOOLTHRS_sr; }
void TMC5160Stepper::TCOOLTHRS(uint32_t input) {
	TCOOLTHRS_sr = input;
	TMC_WRITE_REG(TCOOLTHRS);
}
///////////////////////////////////////////////////////////////////////////////////////
// W: THIGH
uint32_t TMC5160Stepper::THIGH() { return THIGH_sr; }
void TMC5160Stepper::THIGH(uint32_t input) {
	THIGH_sr = input;
	TMC_WRITE_REG(THIGH);
}
///////////////////////////////////////////////////////////////////////////////////////
// W: VDCMIN
uint32_t TMC5160Stepper::VDCMIN() { return VDCMIN_sr; }
void TMC5160Stepper::VDCMIN(uint32_t input) {
	VDCMIN_sr = input;
	TMC_WRITE_REG(VDCMIN);
}
///////////////////////////////////////////////////////////////////////////////////////
// R+W: CHOPCONF
uint32_t TMC5160Stepper::CHOPCONF() { TMC_READ_REG(CHOPCONF); }
void TMC5160Stepper::CHOPCONF(uint32_t input) {
	CHOPCONF_sr = input;
	TMC_WRITE_REG(CHOPCONF);
}

void TMC5160Stepper::toff(		uint8_t B )	{ TMC_MOD_REG(CHOPCONF, TOFF);		}
void TMC5160Stepper::hstrt(		uint8_t B )	{ TMC_MOD_REG(CHOPCONF, HSTRT);		}
void TMC5160Stepper::hend(		uint8_t B )	{ TMC_MOD_REG(CHOPCONF, HEND);		}
void TMC5160Stepper::fd3(		bool    B )	{ TMC_MOD_REG(CHOPCONF, FD3);		}
void TMC5160Stepper::disfdcc(	bool 	B )	{ TMC_MOD_REG(CHOPCONF, DISFDCC);	}
void TMC5160Stepper::chm(		bool 	B )	{ TMC_MOD_REG(CHOPCONF, CHM);		}
void TMC5160Stepper::tbl(		uint8_t B )	{ TMC_MOD_REG(CHOPCONF, TBL);		}
void TMC5160Stepper::vhighfs(	bool 	B )	{ TMC_MOD_REG(CHOPCONF, VHIGHFS);	}
void TMC5160Stepper::vhighchm(	bool 	B )	{ TMC_MOD_REG(CHOPCONF, VHIGHCHM);	}
void TMC5160Stepper::tpfd(		uint8_t B )	{ TMC_MOD_REG(CHOPCONF, TPFD);		}
void TMC5160Stepper::mres(		uint8_t B )	{ TMC_MOD_REG(CHOPCONF, MRES);		}
void TMC5160Stepper::intpol(	bool 	B )	{ TMC_MOD_REG(CHOPCONF, INTPOL);	}
void TMC5160Stepper::dedge(		bool 	B )	{ TMC_MOD_REG(CHOPCONF, DEDGE);		}
void TMC5160Stepper::diss2g(	bool 	B )	{ TMC_MOD_REG(CHOPCONF, DISS2G);	}
void TMC5160Stepper::diss2vs(	bool 	B )	{ TMC_MOD_REG(CHOPCONF, DISS2VS);	}
uint8_t TMC5160Stepper::toff()		{ TMC_GET_BYTE(CHOPCONF, TOFF);		}
uint8_t TMC5160Stepper::hstrt()		{ TMC_GET_BYTE(CHOPCONF, HSTRT);	}
uint8_t TMC5160Stepper::hend()		{ TMC_GET_BYTE(CHOPCONF, HEND);		}
uint8_t TMC5160Stepper::fd3()		{ TMC_GET_BYTE(CHOPCONF, FD3);		}
bool 	TMC5160Stepper::disfdcc()	{ TMC_GET_BYTE(CHOPCONF, DISFDCC);	}
bool 	TMC5160Stepper::chm()		{ TMC_GET_BYTE(CHOPCONF, CHM);		}
uint8_t TMC5160Stepper::tbl()		{ TMC_GET_BYTE(CHOPCONF, TBL);		}
bool 	TMC5160Stepper::vhighfs()	{ TMC_GET_BYTE(CHOPCONF, VHIGHFS);	}
bool 	TMC5160Stepper::vhighchm()	{ TMC_GET_BYTE(CHOPCONF, VHIGHCHM);	}
uint8_t TMC5160Stepper::tpfd()		{ TMC_GET_BYTE(CHOPCONF, TPFD);		}
uint8_t TMC5160Stepper::mres()		{ TMC_GET_BYTE(CHOPCONF, MRES);		}
bool 	TMC5160Stepper::intpol()	{ TMC_GET_BYTE(CHOPCONF, INTPOL);	}
bool 	TMC5160Stepper::dedge()		{ TMC_GET_BYTE(CHOPCONF, DEDGE);	}
bool 	TMC5160Stepper::diss2g()	{ TMC_GET_BYTE(CHOPCONF, DISS2G);	}
bool 	TMC5160Stepper::diss2vs()	{ TMC_GET_BYTE(CHOPCONF, DISS2VS);	}

void TMC5160Stepper::microsteps(uint16_t ms) {
	switch(ms) {
		case 256: mres(0); break;
		case 128: mres(1); break;
		case  64: mres(2); break;
		case  32: mres(3); break;
		case  16: mres(4); break;
		case   8: mres(5); break;
		case   4: mres(6); break;
		case   2: mres(7); break;
		case   0: mres(8); break;
		default: break;
				}
	}
///////////////////////////////////////////////////////////////////////////////////////
// W: COOLCONF
uint32_t TMC5160Stepper::COOLCONF() { return COOLCONF_sr; }
void TMC5160Stepper::COOLCONF(uint32_t input) {
	COOLCONF_sr = input;
	TMC_WRITE_REG(COOLCONF);
}
void TMC5160Stepper::semin(		uint8_t B )	{ TMC_MOD_REG(COOLCONF, SEMIN);		}
void TMC5160Stepper::seup(		uint8_t B )	{ TMC_MOD_REG(COOLCONF, SEUP);		}
void TMC5160Stepper::semax(		uint8_t B )	{ TMC_MOD_REG(COOLCONF, SEMAX);		}
void TMC5160Stepper::sedn(		uint8_t B )	{ TMC_MOD_REG(COOLCONF, SEDN);		}
void TMC5160Stepper::seimin(	bool 	B )	{ TMC_MOD_REG(COOLCONF, SEIMIN);	}
void TMC5160Stepper::sgt(		int8_t  B )	{ TMC_MOD_REG(COOLCONF, SGT);		}
void TMC5160Stepper::sfilt(		bool 	B )	{ TMC_MOD_REG(COOLCONF, SFILT);		}

uint8_t TMC5160Stepper::semin()	{ TMC_GET_BYTE(COOLCONF, SEMIN);	}
uint8_t TMC5160Stepper::seup()	{ TMC_GET_BYTE(COOLCONF, SEUP);		}
uint8_t TMC5160Stepper::semax()	{ TMC_GET_BYTE(COOLCONF, SEMAX);	}
uint8_t TMC5160Stepper::sedn()	{ TMC_GET_BYTE(COOLCONF, SEDN);		}
bool TMC5160Stepper::seimin()	{ TMC_GET_BYTE(COOLCONF, SEIMIN);	}
bool TMC5160Stepper::sfilt()	{ TMC_GET_BYTE(COOLCONF, SFILT);	}
int8_t TMC5160Stepper::sgt() {
	// Two's complement in a 7bit value
	int8_t val = (COOLCONF()&SGT_bm) >> SGT_bp;
	return val <= 63 ? val : val - 128;
}
///////////////////////////////////////////////////////////////////////////////////////
// R: DRV_STATUS
uint32_t TMC5160Stepper::DRV_STATUS() { TMC_READ_REG_R(DRV_STATUS); 			}
uint16_t TMC5160Stepper::sg_result(){ TMC_GET_BYTE_R(DRV_STATUS, SG_RESULT); 	}
bool TMC5160Stepper::s2vsa()		{ TMC_GET_BYTE_R(DRV_STATUS, S2VSA); 		}
bool TMC5160Stepper::s2vsb()		{ TMC_GET_BYTE_R(DRV_STATUS, S2VSB); 		}
bool TMC5160Stepper::stealth()		{ TMC_GET_BYTE_R(DRV_STATUS, STEALTH); 		}
bool TMC5160Stepper::fsactive()		{ TMC_GET_BYTE_R(DRV_STATUS, FSACTIVE); 	}
uint8_t TMC5160Stepper::cs_actual()	{ TMC_GET_BYTE_R(DRV_STATUS, CS_ACTUAL); 	}
bool TMC5160Stepper::stallguard()	{ TMC_GET_BYTE_R(DRV_STATUS, STALLGUARD); 	}
bool TMC5160Stepper::ot()			{ TMC_GET_BYTE_R(DRV_STATUS, OT); 			}
bool TMC5160Stepper::otpw()			{ TMC_GET_BYTE_R(DRV_STATUS, OTPW); 		}
bool TMC5160Stepper::s2ga()			{ TMC_GET_BYTE_R(DRV_STATUS, S2GA); 		}
bool TMC5160Stepper::s2gb()			{ TMC_GET_BYTE_R(DRV_STATUS, S2GB); 		}
bool TMC5160Stepper::ola()			{ TMC_GET_BYTE_R(DRV_STATUS, OLA); 			}
bool TMC5160Stepper::olb()			{ TMC_GET_BYTE_R(DRV_STATUS, OLB); 			}
bool TMC5160Stepper::stst()			{ TMC_GET_BYTE_R(DRV_STATUS, STST); 		}
//////////////////////////////////////////////////////////////////////////////////////
// W: PWMCONF
uint32_t TMC5160Stepper::PWMCONF() { return PWMCONF_sr; }
void TMC5160Stepper::PWMCONF(uint32_t input) {
	PWMCONF_sr = input;
	TMC_WRITE_REG(PWMCONF);
}
void TMC5160Stepper::pwm_ofs(		uint8_t B )	{ TMC_MOD_REG(PWMCONF, PWM_OFS);		}
void TMC5160Stepper::pwm_grad(		uint8_t B )	{ TMC_MOD_REG(PWMCONF, PWM_GRAD);		}
void TMC5160Stepper::pwm_freq(		uint8_t B )	{ TMC_MOD_REG(PWMCONF, PWM_FREQ);		}
void TMC5160Stepper::pwm_autoscale(	bool 	B )	{ TMC_MOD_REG(PWMCONF, PWM_AUTOSCALE);	}
void TMC5160Stepper::pwm_autograd(	bool 	B )		{ TMC_MOD_REG(PWMCONF, PWM_AUTOGRAD);	}
void TMC5160Stepper::freewheel(		uint8_t B )	{ TMC_MOD_REG(PWMCONF, FREEWHEEL);		}
void TMC5160Stepper::pwm_reg(		uint8_t B )	{ TMC_MOD_REG(PWMCONF, PWM_REG);		}
void TMC5160Stepper::pwm_lim(		uint8_t B )	{ TMC_MOD_REG(PWMCONF, PWM_LIM);		}
uint8_t TMC5160Stepper::pwm_ofs()		{ TMC_GET_BYTE(PWMCONF, PWM_OFS);		}
uint8_t TMC5160Stepper::pwm_grad()		{ TMC_GET_BYTE(PWMCONF, PWM_GRAD);		}
uint8_t TMC5160Stepper::pwm_freq()		{ TMC_GET_BYTE(PWMCONF, PWM_FREQ);		}
bool 	TMC5160Stepper::pwm_autoscale()	{ TMC_GET_BYTE(PWMCONF, PWM_AUTOSCALE);	}
bool 	TMC5160Stepper::pwm_autograd()		{ TMC_GET_BYTE(PWMCONF, PWM_REG);		}
uint8_t TMC5160Stepper::freewheel()		{ TMC_GET_BYTE(PWMCONF, PWM_LIM);		}
uint8_t TMC5160Stepper::pwm_reg()		{ TMC_GET_BYTE(PWMCONF, PWM_REG);		}
uint8_t TMC5160Stepper::pwm_lim()		{ TMC_GET_BYTE(PWMCONF, PWM_LIM);		}
///////////////////////////////////////////////////////////////////////////////////////
// R: PWM_SCALE
uint32_t TMC5160Stepper::PWM_SCALE() 	{ TMC_READ_REG_R(PWM_SCALE); 			}
///////////////////////////////////////////////////////////////////////////////////////
// R: PWM_AUTO
uint32_t TMC5160Stepper::PWM_AUTO() 	{ TMC_READ_REG_R(PWM_AUTO); 			}
///////////////////////////////////////////////////////////////////////////////////////
// R: LOST_STEPS
uint32_t TMC5160Stepper::LOST_STEPS() 	{ TMC_READ_REG_R(LOST_STEPS); 			}
