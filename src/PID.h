#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

	/* РљРѕСЌС„С„РёС†РёРµРЅС‚С‹ СЂРµРіСѓР»СЏС†РёРё */
	float Kp;
	float Ki;
	float Kd;

	/* РўР°Сѓ Р¤РќР§ */
	float tau;

	/* РџСЂРµРґРµР»С‹ СЃРёРіРЅР°Р»Р° РЅР° РІС‹С…РѕРґРµ */
	float limMin;
	float limMax;

	/* РџСЂРµРґРµР»С‹ СЂР°Р±РѕС‚С‹ РёРЅС‚РµРіСЂРёСЂСѓСЋС‰РµРіРѕ РєРѕРјРїРѕРЅРµРЅС‚Р° */
	float limMinInt;
	float limMaxInt;

	/* РІСЂРµРјСЏ РјРµР¶ СЃРЅСЏС‚РёР№ РЅР°РїСЂСЏР¶РµРЅРёСЏ */
	float T;

	/* РљРІР°Р·Рё РїР°РјСЏС‚СЊ СЂРµРіСѓР»СЏС‚РѕСЂР° */
	float integrator;
	float prevError;			/* РўСЂРµР±СѓРµС‚СЃСЏ РґР»СЏ РёРЅС‚РµРіСЂР°С‚РѕСЂР° */
	float differentiator;
	float prevMeasurement;		/* РўСЂРµР±СѓРµС‚СЃСЏ РґР»СЏ РґРёС„С„РµСЂРµРЅС†РёР°С‚РѕСЂР° */

	/* Р’С‹С…РѕРґ СЂРµРіСѓР»СЏС‚РѕСЂР° */
	float out;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif
