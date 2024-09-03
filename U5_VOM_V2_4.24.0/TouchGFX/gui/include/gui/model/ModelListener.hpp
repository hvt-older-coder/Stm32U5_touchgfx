#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>
#ifndef SIMULATOR
extern "C" {
#include "main.h"
}
#endif
class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }
#ifndef SIMULATOR
    virtual void myUpdate(){}
    virtual void updateVOM(uint32_t voltage_mV, unsigned int fps){}
#endif

protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
