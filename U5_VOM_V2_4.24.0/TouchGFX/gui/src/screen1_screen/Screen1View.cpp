#include <gui/screen1_screen/Screen1View.hpp>
#include <texts/TextKeysAndLanguages.hpp>
#ifndef SIMULATOR
extern "C" {
#include "main.h"
}
#endif
Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}
#ifndef SIMULATOR
void Screen1View::myUpdateGauge()
{
	int val_min, val_max;
	gauge1.getRange(val_min, val_max);
	if(voltageValue < val_max)
	{
		gauge1.setValue(voltageValue);
	}
}

void Screen1View::updateVOM(uint32_t voltage_mV)
{

	Unicode::snprintf(textArea1Buffer,TEXTAREA1_SIZE, "%s", "");
	textArea1.invalidate();
	Unicode::snprintf(textArea1Buffer,TEXTAREA1_SIZE, "%d", voltage_mV);
	textArea1.resizeToCurrentText();
	textArea1.invalidate();

	uint32_t powerW = voltage_mV*1500/1000;
	Unicode::snprintf(textArea3Buffer,TEXTAREA3_SIZE, "%s", "");
	textArea3.invalidate();
	Unicode::snprintf(textArea3Buffer,TEXTAREA3_SIZE, "%d", powerW);
	textArea3.invalidate();

	voltageValue = voltage_mV*100/9999;
	myUpdateGauge();

}//
#endif
