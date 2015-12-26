#include "effects.h"
#include "ffb.h"

#include <avr/wdt.h>

#define min(A, B) ((A < B) ? A : B)
#define max(A, B) ((A > B) ? A : B)
#define clamp(val, lower, upper) min(upper, max(lower, val))

EffectResponse_t ProcessAllEffects(volatile TEffectState* effects, uint8_t len, int8_t xAxis, int8_t yAxis)
{
    int16_t xTotalForce = 0;
    int16_t yTotalForce = 0;
    for (int i = 0; i < len; i++)
    {
        if (effects[i].state == MEffectState_Playing)
        {
            EffectResponse_t effectResponse;
            effectResponse.effectResponseX = 0;
            effectResponse.effectResponseY = 0;
            switch (effects[i].type)
            {
            case USB_EFFECT_CONSTANT:
                effectResponse = ProcessEffect_Constant(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_RAMP:
                effectResponse = ProcessEffect_Ramp(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_SQUARE:
                effectResponse = ProcessEffect_Square(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_SINE:
                effectResponse = ProcessEffect_Sine(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_TRIANGLE:
                effectResponse = ProcessEffect_Triangle(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_SAWTOOTHDOWN:
                effectResponse = ProcessEffect_SawtoothDown(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_SAWTOOTHUP:
                effectResponse = ProcessEffect_SawtoothUp(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_SPRING:
                effectResponse = ProcessEffect_Spring(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_DAMPER:
                effectResponse = ProcessEffect_Damper(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_INERTIA:
                effectResponse = ProcessEffect_Inertia(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_FRICTION:
                effectResponse = ProcessEffect_Friction(&effects[i], xAxis, yAxis);
                break;
            case USB_EFFECT_CUSTOM:
                // Do nothing
                break;
            }
            xTotalForce += effectResponse.effectResponseX;
            yTotalForce += effectResponse.effectResponseY;
        }
    }
    EffectResponse_t effectResponse;
    effectResponse.effectResponseX = (int8_t)clamp(xTotalForce, -127, 127);
    effectResponse.effectResponseY = (int8_t)clamp(yTotalForce, -127, 127);

    return effectResponse;
}

EffectResponse_t ProcessEffect_Constant(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}

EffectResponse_t ProcessEffect_Ramp(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}

EffectResponse_t ProcessEffect_Square(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}

EffectResponse_t ProcessEffect_Sine(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}

EffectResponse_t ProcessEffect_Triangle(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}

EffectResponse_t ProcessEffect_SawtoothDown(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}

EffectResponse_t ProcessEffect_SawtoothUp(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}

EffectResponse_t ProcessEffect_Spring(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    const int16_t deadzone = 10;
    const int16_t stiction = 60;

    static int8_t lastX = 0;
    static int8_t lastY = 0;
    static int8_t firstRun = 1;

    static int16_t xLastVel = 0;
    static int16_t yLastVel = 0;

    static int16_t centerP = 10;
    static int16_t centerD = 30;

    if (firstRun)
    {
        lastX = xAxis;
        lastY = yAxis;
        firstRun = 0;
    }

    wdt_reset();

    int16_t xVel = (xAxis - lastX) / 2 + xLastVel / 2;
    int16_t yVel = (yAxis - lastY) / 2 + yLastVel / 2;

    int16_t xError = 0 - xAxis; //TODO actually make this dynamic
    int16_t yError = 0 - yAxis;

    //Aggressive value = 16
    int16_t xForceP = clamp(xError * centerP / 64, -90, 90);
    int16_t yForceP = clamp(yError * centerP / 64, -90, 90);

    //Aggressive value = 60ish
    int16_t xForceD = clamp(xVel * centerD / 8, -127, 127);
    int16_t yForceD = clamp(yVel * centerD / 8, -127, 127);

    int16_t xTotalForce = xForceP - xForceD;
    int16_t yTotalForce = yForceP - yForceD;

    if (xTotalForce > deadzone)
    {
        xTotalForce += stiction;
    }
    if (xTotalForce < -deadzone)
    {
        xTotalForce -= stiction;
    }

    if (yTotalForce > deadzone)
    {
        yTotalForce += stiction;
    }
    if (yTotalForce < -deadzone)
    {
        yTotalForce -= stiction;
    }

    EffectResponse_t springResponse;
    springResponse.effectResponseX = (int8_t)clamp(xTotalForce, -127, 127);
    springResponse.effectResponseY = (int8_t)clamp(yTotalForce, -127, 127);

    xLastVel = xVel;
    yLastVel = yVel;

    lastX = xAxis;
    lastY = yAxis;

    return springResponse;
}

EffectResponse_t ProcessEffect_Damper(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}

EffectResponse_t ProcessEffect_Inertia(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}

EffectResponse_t ProcessEffect_Friction(volatile TEffectState* effect, int8_t xAxis, int8_t yAxis)
{
    EffectResponse_t response;
    response.effectResponseX = 0;
    response.effectResponseY = 0;
    return response;
}
