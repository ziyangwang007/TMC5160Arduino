#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

class TMC5160Stepper {
	public:
		TMC5160Stepper(uint16_t pinEN, uint16_t pinDIR, uint16_t pinStep, uint16_t pinCS);
		TMC5160Stepper(uint16_t pinEN, uint16_t pinDIR, uint16_t pinStep, uint16_t pinCS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK);
		void begin();
		uint8_t test_connection();



		// GCONF
		uint32_t GCONF();
		void GCONF(							uint32_t value);
		void recalibrate(					bool B);
		void faststandstill(				bool B);
		void en_pwm_mode(					bool B);
		void multistep_filt(				bool B);
		void shaft(							bool B);
		void diag0_error(					bool B);
		void diag0_otpw(					bool B);
		void diag0_stall(					bool B);
		void diag1_stall(					bool B);
		void diag1_index(					bool B);
		void diag1_onstate(					bool B);
		void diag1_steps_skipped(			bool B);
		void diag0_int_pushpull(			bool B);
		void diag1_poscomp_pushpull(		bool B);
		void small_hysteresis(				bool B);
		void stop_enable(					bool B);
		void direct_mode(					bool B);
		void test_mode(						bool B);
		bool recalibrate();
		bool faststandstill();
		bool en_pwm_mode();
		bool multistep_filt();
		bool shaft();
		bool diag0_error();
		bool diag0_otpw();
		bool diag0_stall();
		bool diag1_stall();
		bool diag1_index();
		bool diag1_onstate();
		bool diag1_steps_skipped();
		bool diag0_int_pushpull();
		bool diag1_poscomp_pushpull();
		bool small_hysteresis();
		bool stop_enable();
		bool direct_mode();
		bool test_mode();
		// GSTAT
		void 	GSTAT(							uint8_t input);
		uint8_t GSTAT();
		bool 	reset();
		bool 	drv_err();
		bool 	uv_cp();
		// IFCNT
		// SLAVECONF
		// IOIN
		uint32_t IOIN();
		bool 	step();
		bool 	dir();
		bool 	dcen_cfg4();
		bool 	dcin_cfg5();
		bool 	drv_enn_cfg6();
		bool 	dco();
		uint8_t version();
		// OUTPUT
		// X_COMPARE 
		// OTP_PROG 
		// OTP_READ
		// FACTORY_CONF
		// SHORT_CONF
		void SHORT_CONF(	uint32_t value);
		uint32_t SHORT_CONF();
		void s2vs_level(	uint8_t B );
		void s2g_level(		uint8_t B );
		void shortfilter(	uint8_t B );
		void shortdelay(	uint8_t B );
		// DRV_CONF
		void DRV_CONF(		uint32_t value);
		uint32_t DRV_CONF();
		void bbmtime(		uint8_t B);
		void bbmclks(		uint8_t B);
		void otselect(		uint8_t B);
		void drvstrength(	uint8_t B);
		void filt_isense(	uint8_t B);
		// GLOBALSCALER
		uint32_t GLOBALSCALER();
		void GLOBALSCALER(						uint8_t B);
		// OFFSET_READ
		// IHOLD_IRUN
		void 		IHOLD_IRUN(					uint32_t input);
		uint32_t 	IHOLD_IRUN();
		void 		ihold(						uint8_t B);
		void 		irun(						uint8_t B);
		void 		iholddelay(					uint8_t B);
		uint8_t 	ihold();
		uint8_t 	irun();
		uint8_t 	iholddelay();
		// TPOWERDOWN
		uint8_t TPOWERDOWN();
		void TPOWERDOWN(						uint8_t input);
		// TSTEP
		uint32_t TSTEP();
		// TPWMTHRS
		uint32_t TPWMTHRS();
		void TPWMTHRS(							uint32_t input);
		// TCOOLTHRS
		uint32_t TCOOLTHRS();
		void TCOOLTHRS(							uint32_t input);
		// THIGH
		uint32_t THIGH();
		void THIGH(								uint32_t input);
		// RAMPMODE
		// XACTUAL
		// VACTUAL
		// VSTART
		// A1
		// V1
		// AMAX
		// VMAX 
		// DMAX 
		// D1
		// VSTOP
		// TZEROWAIT 
		// XTARGET
		// VDCMIN
		uint32_t VDCMIN();
		void VDCMIN(						uint32_t input);
		// SW_MODE
		// RAMP_STAT
		// XLATCH
		// ENCMODE
		// X_ENC
		// ENC_CONST
		// ENC_STATUS
		// ENC_LATCH
		// ENC_DEVIATION
		// CHOPCONF
		uint32_t CHOPCONF();
		void CHOPCONF(							uint32_t value);
		void toff(								uint8_t B);
		void hstrt(								uint8_t B);
		void hend(								uint8_t B);
		void fd3(								bool    B);
		void disfdcc(							bool 	B);
		void chm(								bool 	B);
		void tbl(								uint8_t B);
		void vhighfs(							bool 	B);
		void vhighchm(							bool 	B);
		void tpfd(								uint8_t B);
		void mres(								uint8_t B);
		void microsteps(						uint16_t ms);
		void intpol(							bool 	B);
		void dedge(								bool 	B);
		void diss2g(							bool 	B);
		void diss2vs(							bool 	B);
		uint8_t toff();
		uint8_t hstrt();
		uint8_t hend();
		uint8_t fd3();
		bool 	disfdcc();
		bool 	chm();
		uint8_t tbl();
		bool 	vhighfs();
		bool 	vhighchm();
		uint8_t tpfd();
		uint8_t mres();
		bool 	intpol();
		bool 	dedge();
		bool 	diss2g();
		bool 	diss2vs();
		// COOLCONF
		void COOLCONF(uint32_t value);
		uint32_t COOLCONF();
		void semin(							uint8_t B);
		void seup(							uint8_t B);
		void semax(							uint8_t B);
		void sedn(							uint8_t B);
		void seimin(						bool B);
		void sgt(							int8_t B);
		void sfilt(							bool B);
		uint8_t semin();
		uint8_t seup();
		uint8_t semax();
		uint8_t sedn();
		bool seimin();
		int8_t sgt();
		bool sfilt();
		// DCCTRL
		// DRV_STATUS
		uint32_t DRV_STATUS();
		uint16_t sg_result();
		bool s2vsa();
		bool s2vsb();
		bool stealth();
		bool fsactive();
		uint8_t cs_actual();
		bool stallguard();
		bool ot();
		bool otpw();
		bool s2ga();
		bool s2gb();
		bool ola();
		bool olb();
		bool stst();
		// PWMCONF
		void PWMCONF(						uint32_t value);
		uint32_t PWMCONF();
		void pwm_ofs(						uint8_t B);
		void pwm_grad(						uint8_t B);
		void pwm_freq(						uint8_t B);
		void pwm_autoscale(					bool	B);
		void pwm_autograd(					bool	B);
		void freewheel(						uint8_t B);
		void pwm_reg(						uint8_t B);
		void pwm_lim(						uint8_t B);
		uint8_t pwm_ofs();
		uint8_t pwm_grad();
		uint8_t pwm_freq();
		bool 	pwm_autoscale();
		bool 	pwm_autograd();
		uint8_t freewheel();
		uint8_t pwm_reg();
		uint8_t pwm_lim();
		// PWM_SCALE
		uint32_t PWM_SCALE();
		// PWM_AUTO
		uint32_t PWM_AUTO();
		// LOST_STEPS
		uint32_t LOST_STEPS();

		uint8_t status_response;
		uint32_t drv_status;

	private:
		const uint16_t _pinEN        = 0xFFFF;
		const uint16_t _pinSTEP      = 0xFFFF;
		const uint16_t _pinCS        = 0xFFFF;
		const uint16_t _pinDIR       = 0xFFFF;

		// Shadow registers
		uint32_t 	        GCONF_sr 		= 0x00000000UL,
							GSTAT_sr		= 0x00000000UL,
							IOIN_sr 		= 0x00000000UL,
							SHORT_CONF_sr	= 0x00010606UL,
							DRV_CONF_sr		= 0x00080400UL,
							GLOBALSCALER_sr = 0x00000080UL,        // 128
							IHOLD_IRUN_sr 	= 0x00000000UL,
							TPOWERDOWN_sr 	= 0x00000000UL,
							VDCMIN_sr 		= 0x00000000UL,
							CHOPCONF_sr 	= 0x10410150UL,
							COOLCONF_sr 	= 0x00000000UL,
							TSTEP_sr 		= 0x00000000UL,
							TPWMTHRS_sr 	= 0x00000000UL,
							TCOOLTHRS_sr 	= 0x00000000UL,
							THIGH_sr 		= 0x00000000UL,
							XDIRECT_sr 		= 0x00000000UL,
							PWMCONF_sr 		= 0xC40C001EUL,

							tmp_sr 			= 0x00000000UL;

		void send5160(uint8_t addressByte, uint32_t *config);

		const bool uses_sw_spi;
};
