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

void tune_PID(PID* p, float Kp, float Ki, float Kd, float Kb){
	p->Kp = Kp;
	p->Ki = Ki;
	p->Kd = Kd;
	p->Kb = Kb;
}

float PID_controller(PID* p , float y, float r){
	float u;
	float newIterm;
	float e = 0;

	e = r-y;


	float Pterm = p->Kp*e;
	newIterm = p->Iterm + (p->Ki)*p->Tc*p->e_old;
	float Dterm = (p->Kd/p->Tc)*(e - p->e_old);

	p->e_old = e;

	u = Pterm + newIterm + Dterm + p->offset;


	// VECCHIA SATURAZIONE
	//-----------------------------------------
	//Verifico la saturazione generale del PID
	/*if(u > p->u_max){
		u = p->u_max;
	}
	else if(u < p->u_min){
		u = p->u_min;
	}
	else{
		p->Iterm = newIterm;
	}
	*/
	//-----------------------------------------


	//NUOVA IMPLEMENTAZIONE CON BACK-CALCULATION
	//-----------------------------------------
	/*float saturated_u = u;

	if (saturated_u > p->u_max)
		saturated_u = p->u_max;
	else if (saturated_u < p->u_min)
		saturated_u = p->u_min;

	// Correggo l'integrale
	float correction = p->Kb * (saturated_u - u) * p->Ki * p->Tc;
	p->Iterm = newIterm + correction;

	u = saturated_u;*/
	//-----------------------------------------
	if(newIterm > p->u_max){
			newIterm = p->u_max;
		}
		else if(newIterm < p->u_min){
			newIterm = p->u_min;
		}

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


	//Print per il PID di sterzo
	if(p->offset == 0){
		//printf("errore: %.2f, y: %.2f, r: %.2f, u: %.2f \r\n", e, y, r, u);
		//printf("%f\r\n", u);
	}

	return u;
}



