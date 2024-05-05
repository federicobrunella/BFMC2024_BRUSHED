#include "PID.h"
#include <stdio.h>

void init_PID(PID* p, float Tc, float u_max, float u_min, float offset){
	p->Tc = Tc;
	p->u_max = u_max;
	p->u_min = u_min;
	p->Iterm = 0;
	p->e_old = 0;
	p->offset = offset;
}

void tune_PID(PID*p, float Kp, float Ki, float Kd, float Kb){
	p->Kp = Kp;
	p->Ki = Ki;
	p->Kd = Kd;
	p->Kb = Kb;
}

void resetPID(PID* p){
	p->Iterm = 0;
	p->e_old = 0;
}

float PID_controller(PID* p , float y, float r){
	float u;
	float newIterm;
	float e = 0;

	e = r-y;

	if (isinf(p->Iterm) || isnan(p->Iterm)) {
		p->Iterm = 0;
		p->e_old = 0;
	}


	float Pterm = p->Kp*e;
	newIterm = p->Iterm + (p->Ki)*p->Tc*p->e_old;
	float Dterm = (p->Kd/p->Tc)*(e - p->e_old);

	p->e_old = e;


	u = Pterm + newIterm + Dterm + p->offset;

	if(p->offset == 0){
		// ANTI-WINDUP DEL TERMINE INTEGRALE
		if(newIterm > p->u_max){
			newIterm = p->u_max;
		}
		else if(newIterm < p->u_min){
			newIterm = p->u_min;
		}
	}

		// saturazione con back-calculation
		float saturated_u = u;

		if(saturated_u > p->u_max){
			saturated_u = p->u_max;
		}
		else if(saturated_u < p->u_min){
			saturated_u = p->u_min;
		}

		float correction = p->Kb * (saturated_u - u) * p->Ki * p->Tc;
		p->Iterm = newIterm + correction;

		u = saturated_u;
	/*
	else{
		if(u > p->u_max){
			u = p->u_max;
		}
		else if(u < p->u_min){
			u = p->u_min;
		} else {
			p->Iterm = newIterm;
		}
	}
*/

	if(p->offset == 0){
		//printf("%f;%f;%f\r\n", u, p->Iterm, correction);
	}

	return u;
}



