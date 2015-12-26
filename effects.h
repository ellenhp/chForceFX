#ifndef _EFFECTS_
#define _EFFECTS_

#include <stdint.h>
#include <ffb.h>

typedef struct
{
    int8_t effectResponseX;
    int8_t effectResponseY;
} EffectResponse_t;

EffectResponse_t ProcessAllEffects(volatile TEffectState* effects, uint8_t len, int8_t xAxis, int8_t yAxis);

EffectResponse_t ProcessEffect_Constant(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_Ramp(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_Square(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_Sine(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_Triangle(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_SawtoothDown(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_SawtoothUp(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_Spring(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_Damper(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_Inertia(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);
EffectResponse_t ProcessEffect_Friction(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis);

#endif // _EFFECTS_
