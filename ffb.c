#include "ffb.h"
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define wdt_set_interrupt_only(value)   \
    __asm__ __volatile__ (  \
        "in __tmp_reg__,__SREG__" "\n\t"    \
        "cli" "\n\t"    \
        "wdr" "\n\t"    \
        "sts %0,%1" "\n\t"  \
        "out __SREG__,__tmp_reg__" "\n\t"   \
        "sts %0,%2" \
        : /* no outputs */  \
        : "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
        "r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
			   "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) | \
					 _BV(WDIE) | (value & 0x07)) ) \
        : "r0"  \
    )

#define min(A, B) ((A < B) ? A : B)
#define max(A, B) ((A > B) ? A : B)
#define clamp(val, lower, upper) min(upper, max(lower, val))

static int16_t centerP = 0;
static int16_t centerI = 0;
static int16_t centerD = 0;

static int8_t xCenter = 0;
static int8_t yCenter = 0;

static int16_t xIntegral = 0;
static int16_t yIntegral = 0;

static volatile TEffectState gEffectStates[MAX_EFFECTS+1];	// one for each effect (array index 0 is unused to simplify things)

void FFB_SetForceX(int8_t signedSpeed);
void FFB_SetForceY(int8_t signedSpeed);

void FFB_Init(void)
{
    DDRB |= (1 << PB5) | (1 << PB6) | (1 << PB4); // ADC X and Y, one direction enable for X
    DDRE |= (1 << PE6); // Other direction enable for X

    DDRD |= (1 << PD7); // One direction enable for Y
    DDRC |= (1 << PC6); // Other direction enable for Y

    PORTB &= ~(1 << PB4);
    PORTE &= ~(1 << PE6);

    PORTD &= ~(1 << PD7);
    PORTC &= ~(1 << PC6);

    wdt_set_interrupt_only(WDTO_30MS);
    FFB_Enable();
}

void FFB_Disable(void)
{
    FFB_SetForceX(0);
    FFB_SetForceY(0);

    //Disable the timers
    TCCR1A = 0;
    TCCR1B = 0;
}

void FFB_Enable(void)
{
    wdt_reset();

    TCCR1A = _BV(COM1A0) | _BV(COM1A1) | _BV(COM1B0) | _BV(COM1B1) | _BV(WGM10);
    TCCR1B = _BV(CS00);

    FFB_SetForceX(0);
    FFB_SetForceY(0);
}

void FFB_SetForceY(int8_t signedSpeed)
{
    uint8_t direction = (signedSpeed >= 0) ? 1 : 0; //TODO: Does C99 guarantee the true condition here will result in 1 or just nonzero?
    uint8_t notDirection = (signedSpeed >= 0) ? 0 : 1; //TODO: Does C99 guarantee !0 == 1 or just nonzero?
    uint8_t pwmState;

    pwmState = (uint8_t)abs(signedSpeed);
    pwmState = 255 - (pwmState << 1);

    uint8_t portBState = PORTB & ~(1 << PB4);
    PORTB = portBState | (direction << PB4);

    uint8_t portEState = PORTE & ~(1 << PE6);
    PORTE = portEState | (notDirection << PE6);

    OCR1A = pwmState;
}

void FFB_SetForceX(int8_t signedSpeed)
{
    uint8_t direction = (signedSpeed >= 0) ? 1 : 0; //TODO: See above
    uint8_t notDirection = (signedSpeed >= 0) ? 0 : 1; //TODO: See above
    uint8_t pwmState;

    pwmState = (uint8_t)abs(signedSpeed);
    pwmState = 255 - (pwmState << 1);

    uint8_t portCState = PORTC & ~(1 << PC6);
    PORTC = portCState | (direction << PC6);

    uint8_t portDState = PORTD & ~(1 << PD7);
    PORTD = portDState | (notDirection << PD7);

    OCR1B = pwmState;
}

void FFB_SetPID(int16_t p, int16_t i, int16_t d)
{
    if (centerI != i)
    {
        xIntegral = 0;
        yIntegral = 0;
    }
    centerP = p;
    centerI = i;
    centerD = d;
}

void FFB_SetCenter(int8_t xCenterIn, int8_t yCenterIn)
{
    xCenter = xCenterIn;
    yCenter = yCenterIn;
}

void FFB_Update(int8_t xAxis, int8_t yAxis)
{
    const int16_t deadzone = 10;
    const int16_t stiction = 60;

    static int8_t lastX = 0;
    static int8_t lastY = 0;
    static int8_t firstRun = 1;

    static int16_t xLastVel = 0;
    static int16_t yLastVel = 0;

    if (firstRun)
    {
        lastX = xAxis;
        lastY = yAxis;
        firstRun = 0;
    }

    wdt_reset();

    int16_t xVel = (xAxis - lastX) / 2 + xLastVel / 2;
    int16_t yVel = (yAxis - lastY) / 2 + yLastVel / 2;

    int16_t xError = xCenter - xAxis;
    int16_t yError = yCenter - yAxis;

    xIntegral += xError;
    yIntegral += yError;

    //Aggressive value = 16
    int16_t xForceP = clamp(xError * centerP / 64, -90, 90);
    int16_t yForceP = clamp(yError * centerP / 64, -90, 90);

    int16_t xForceI = clamp(xIntegral * centerI / 256, -127, 127);
    int16_t yForceI = clamp(yIntegral * centerI / 256, -127, 127);

    //Aggressive value = 5ish
    int16_t xForceD = clamp(xVel * centerD / 8, -127, 127);
    int16_t yForceD = clamp(yVel * centerD / 8, -127, 127);

    int16_t xTotalForce = xForceP + xForceI - xForceD;
    int16_t yTotalForce = yForceP + yForceI - yForceD;

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

    FFB_SetForceX((int8_t)clamp(xTotalForce, -127, 127));
    FFB_SetForceY((int8_t)clamp(yTotalForce, -127, 127));

    xLastVel = xVel;
    yLastVel = yVel;

    lastX = xAxis;
    lastY = yAxis;
}

void FFB_StopAllEffects()
{
    for (int i = 2; i < MAX_EFFECTS; i++)
    {
        if (gEffectStates[i].state == MEffectState_Playing)
        {
            gEffectStates[i].state = MEffectState_Allocated;
        }
    }
}

uint8_t FFB_GetNextFreeEffect()
{
    for (uint8_t i = 2; i < MAX_EFFECTS; i++)
    {
        if (gEffectStates[i].state == MEffectState_Free)
        {
            gEffectStates[i].state = MEffectState_Allocated;
            return i;
        }
    }
    return 0;
}

void FFB_CreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData)
{
	outData->reportId = 6;
	outData->effectBlockIndex = FFB_GetNextFreeEffect();

	if (outData->effectBlockIndex == 0) {
		outData->loadStatus = 2; // 1=Success,2=Full,3=Error
	} else {
		outData->loadStatus = 1; // 1=Success,2=Full,3=Error

		volatile TEffectState* effect = &gEffectStates[outData->effectBlockIndex];

		effect->usb_duration = USB_DURATION_INFINITE;
		effect->usb_fadeTime = USB_DURATION_INFINITE;
		effect->usb_gain = 0xFF;
		effect->usb_offset = 0;
		effect->usb_attackLevel = 0xFF;
		effect->usb_fadeLevel = 0xFF;
	}

	outData->ramPoolAvailable = 0xFFFF;	//Tell the driver we have plenty of memory left. TODO what should really be here?
}

void FFB_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data)
	{
	uint8_t eid = data->effectBlockIndex;

	if (eid == 0xFF)
    {
        // TODO might not be to spec, but I don't think it's a good idea to start EVERYTHING at once
        // Stopping makes sense though.
        if (data->operation == 3)
        {   // Stop all
            FFB_StopAllEffects();
        }
    }

	if (data->operation == 1)
	{	// Start
		gEffectStates[eid].state = MEffectState_Playing;
    }
	else if (data->operation == 2)
	{	// StartSolo
		FFB_StopAllEffects();
		gEffectStates[eid].state = MEffectState_Playing;
	}
	else if (data->operation == 3)
	{	// Stop
        gEffectStates[eid].state = MEffectState_Allocated;
	}
}


void FFB_SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data)
{
    volatile TEffectState* effect = &gEffectStates[data->effectBlockIndex];

    effect->state = MEffectState_Playing;
}

void FFB_HandleUSBMessage(uint8_t *data, uint16_t len)
{
	uint8_t effectId = data[1]; // effectBlockIndex is always the second byte.

	switch (data[0])	// reportID
	{
	case 1:
		FFB_SetEffect((USB_FFBReport_SetEffect_Output_Data_t *) data);
		break;
	case 2:
		// ffb->SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*) data, &gEffectStates[effectId]);
		break;
	case 3:
		// ffb->SetCondition((USB_FFBReport_SetCondition_Output_Data_t*) data, &gEffectStates[effectId]);
		break;
	case 4:
		// ffb->SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*) data, &gEffectStates[effectId]);
		break;
	case 5:
		// ffb->SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*) data, &gEffectStates[effectId]);
		break;
	case 6:
		// ffb->SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*)data, &gEffectStates[effectId]);
		break;
	case 7:
		// FfbHandle_SetCustomForceData((USB_FFBReport_SetCustomForceData_Output_Data_t*) data);
		break;
	case 8:
		// FfbHandle_SetDownloadForceSample((USB_FFBReport_SetDownloadForceSample_Output_Data_t*) data);
		break;
	case 9:
		break;
	case 10:
		FFB_EffectOperation((USB_FFBReport_EffectOperation_Output_Data_t*) data);
		break;
	case 11:
		// FfbHandle_BlockFree((USB_FFBReport_BlockFree_Output_Data_t *) data);
		break;
	case 12:
		// FfbHandle_DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*) data);
		break;
	case 13:
		// FfbHandle_DeviceGain((USB_FFBReport_DeviceGain_Output_Data_t*) data);
		break;
	case 14:
		// FfbHandle_SetCustomForce((USB_FFBReport_SetCustomForce_Output_Data_t*) data);
		break;
	default:
		break;
	};
}

ISR(WDT_vect)
{
    //WDT interrupt
    FFB_SetForceX(0);
    FFB_SetForceY(0);
}
