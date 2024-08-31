#include <gui/screen1_screen/Screen1View.hpp>

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
void Screen1View::myUpdateGauge()
{
	int val = gauge1.getValue();
	int val_min, val_max;
	gauge1.getRange(val_min, val_max);
	val = val+1;
	if(val > val_max)
	{
		val=val_min;
	}
	gauge1.setValue(val);
}
