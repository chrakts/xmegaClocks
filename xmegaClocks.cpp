#include <avr/io.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "xmegaClocks.h"

void init_clock(int sysclk, int pll, bool doAutocalibration, uint8_t calibrationValue)
{
	CLK_t *mein_clock;
	OSC_t *mein_osc;
	mein_clock = &CLK;
	mein_osc = &OSC;
	switch(sysclk)
	{
		case QUARZ:
			mein_osc->XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;
//			mein_osc->XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCPWR_bm | OSC_XOSCSEL_XTAL_16KCLK_gc;
			mein_osc->CTRL = OSC_XOSCEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 32 MHz-Clock ein

			while((mein_osc->STATUS & OSC_XOSCRDY_bm) == 0)			// wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)		// wartet bis diese stabil
			;

			if ( (pll>0) & (pll<16) )
			{
				mein_osc->PLLCTRL = OSC_PLLSRC_XOSC_gc | pll;
				mein_osc->CTRL = OSC_PLLEN_bm | OSC_XOSCEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet zusätzlich die PLL ein

				while((mein_osc->STATUS & OSC_PLLRDY_bm) == 0)		// wartet bis diese stabil
				;
				CCP = CCP_IOREG_gc;										// geschuetztes Register freigeben
				mein_clock->CTRL = CLK_SCLKSEL_PLL_gc;					// umschalten auf PLL-Clock
				mein_osc->CTRL = OSC_PLLEN_bm | OSC_XOSCEN_bm | OSC_RC32KEN_bm;
			}
			else
			{
				CCP = CCP_IOREG_gc;										// geschuetztes Register freigeben
				mein_clock->CTRL = CLK_SCLKSEL_XOSC_gc;					// umschalten auf XOSC-Clock
				mein_osc->CTRL = OSC_XOSCEN_bm | OSC_RC32KEN_bm;
			}
		break; // QUARZ
		case CLK2M:
			mein_osc->CTRL = OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 2 MHz-Clock ein
			while((mein_osc->STATUS & OSC_RC2MRDY_bm) == 0)  // wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)  // wartet bis diese stabil
			;
			CCP = CCP_IOREG_gc;								// geschuetztes Register freigeben
			mein_clock->CTRL = CLK_SCLKSEL_RC2M_gc;		// umschalten auf 2 MHz-Clock
      if(doAutocalibration==true)
      {
        OSC_RC32KCAL = calibrationValue;
        #if defined (__AVR_ATxmega32E5__)

        #else
          CLKSYS_AutoCalibration_Enable(OSC_RC2MCREF_RC32K_gc,false); // OSC_RC32MCREF_bm
        #endif // defined
      }
		break;
		case CLK32M:
			mein_osc->CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm; // schaltet die 32 MHz-Clock ein
			while((mein_osc->STATUS & OSC_RC32MRDY_bm) == 0)  // wartet bis diese stabil
			;
			while((mein_osc->STATUS & OSC_RC32KRDY_bm) == 0)  // wartet bis diese stabil
			;
			CCP = CCP_IOREG_gc;								// geschuetztes Register freigeben
			mein_clock->CTRL = CLK_SCLKSEL_RC32M_gc;		// umschalten auf 32 MHz-Clock
			mein_osc->CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm;		// abschalten der 2 MHz-Clock
      if(doAutocalibration==true)
      {
        OSC_RC32KCAL = calibrationValue;
        CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_RC32K_gc,false); // OSC_RC32MCREF_bm
      }

		break;
	}
}


/*! \brief This function enables automatic calibration of the selected internal
 *         oscillator.
 *
 *  Either the internal 32kHz RC oscillator or an external 32kHz
 *  crystal can be used as a calibration reference. The user must make sure
 *  that the selected reference is ready and running.
 *
 *  \param  clkSource    Clock source to calibrate, either OSC_RC2MCREF_bm or
 *                       OSC_RC32MCREF_bm.
 *  \param  bextReference True if external crystal should be used as reference.
 */

#if defined (__AVR_ATxmega32E5__)
void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, bool extReference )
{/*
	OSC.DFLLCTRL = ( OSC.DFLLCTRL & ~clkSource ) |
	               ( extReference ? clkSource : 0 );
	if (clkSource == OSC_RC2MCREF_bm) {
		DFLLRC2M.CTRL |= DFLL_ENABLE_bm;
	} else if (clkSource == OSC_RC32MCREF_RC32K_gc) {   // OSC_RC32MCREF_bm
		DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
	}*/
}
#else
/*! \brief This function enables automatic calibration of the selected internal
 *         oscillator.
 *
 *  Either the internal 32kHz RC oscillator or an external 32kHz
 *  crystal can be used as a calibration reference. The user must make sure
 *  that the selected reference is ready and running.
 *
 *  \param  clkSource    Clock source to calibrate, either OSC_RC2MCREF_bm or
 *                       OSC_RC32MCREF_bm.
 *  \param  bextReference True if external crystal should be used as reference.
 */
void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, bool extReference )
{
	OSC.DFLLCTRL = ( OSC.DFLLCTRL & ~clkSource ) |
	               ( extReference ? clkSource : 0 );
	if (clkSource == OSC_RC2MCREF_bm) {
		DFLLRC2M.CTRL |= DFLL_ENABLE_bm;
	} else if (clkSource == OSC_RC32MCREF_RC32K_gc) {   // OSC_RC32MCREF_bm
		DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
	}
}
#endif // defined
