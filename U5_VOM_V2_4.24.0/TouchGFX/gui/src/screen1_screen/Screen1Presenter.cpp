#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{

}
#ifndef SIMULATOR
void Screen1Presenter::myUpdate()
{
	view.myUpdateGauge();
}
void Screen1Presenter::updateVOM(uint32_t voltage_mV, unsigned int fps)
{
	view.updateVOM(voltage_mV, fps);
}
#endif
