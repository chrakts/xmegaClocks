#ifndef XMEGACLOCKS_H_INCLUDED
#define XMEGACLOCKS_H_INCLUDED

enum{QUARZ,CLK2M,CLK32M};

void init_clock(int sysclk, int pll, bool doAutocalibration, uint8_t calibrationValue);
void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, bool extReference);

#endif // XMEGACLOCKS_H_INCLUDED
