#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>
#ifndef SIMULATOR
extern "C"
{
#include "main.h"
}
#endif
class Screen1View : public Screen1ViewBase
{
public:
	int voltageValue = 0;
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
#ifndef SIMULATOR
    virtual void myUpdateGauge();
    virtual void updateVOM(uint32_t voltage_mV);
#endif
protected:
};

#endif // SCREEN1VIEW_HPP
