#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#ifndef SIMULATOR
#include "main.h"
extern "C"
{

	extern uint8_t update_ui;
}
#endif
Model::Model() : modelListener(0)
{

}

void Model::tick()
{
#ifndef SIMULATOR
	if(update_ui){
		modelListener->myUpdate();
		update_ui = 0;
	}
#endif
}
