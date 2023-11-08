#include <stdio.h>

double statusPWM(double duty, double th, double* prev)
{	
	if((duty*duty < th*th) || (duty*duty > 1))
	{
		return 2;
	}
	
	if(duty * *prev < 0)
	{
		*prev = 0;
		return 3;
	}
	
	*prev = duty;
	return duty;
}

double setPIN_and_Delay(double* A, double* B, double* C, double* D, double CDuty)
{
	if (CDuty == 2)
	{
		*A = 0;
		*B = 0;
		*C = 0;
		*D = 0;
		return 0;
	}
	
	else
	{
		if (CDuty == 3)
		{
			*A = 0;
			*B = 0;
			*C = 0;
			*D = 0;
			return 0.0001;
		}
		
		else
		{
			if (CDuty > 0)
			{
				*A = 1;
				*B = 0;
				*C = 0;
				*D = CDuty;
				return 0;
			}
			
			if (CDuty < 0)
			{
				*A = 0;
				*B = -CDuty;
				*C = 1;
				*D = 0;
				return 0;
			}
		}
		
	}
}



int
main (int argc, char *argv[])
{
	double A, B, C, D, duty, prev;
	
	prev = 0;
	
	//Testing status
	/*
	printf("%f\n", statusPWM(4, 0.3, &prev));
	printf("%f\n", statusPWM(0.2, 0.3, &prev));
	printf("%f and %f\n", statusPWM(0.5, 0.3, &prev), prev);
	printf("%f and %f\n", statusPWM(0.5, 0.3, &prev), prev);
	printf("%f and %f\n", statusPWM(-0.5, 0.3, &prev), prev);
	printf("%f and %f\n", statusPWM(-0.5, 0.3, &prev), prev);
	printf("%f and %f\n", statusPWM(0.5, 0.3, &prev), prev);
	*/
	//OK
	
	printf("\n\n\n");
	
	//Testing set
	prev = 0;
	printf("Duty out of bound\n");
	duty = statusPWM(4, 0.3, &prev);
	printf("Delay: %f\n", setPIN_and_Delay(&A, &B, &C, &D, duty));
	printf("%f %f %f %f", A, B, C, D);
	
	printf("\n\n");
	printf("Duty less than th\n");
	duty = statusPWM(0.2, 0.3, &prev);
	printf("Delay: %f\n", setPIN_and_Delay(&A, &B, &C, &D, duty));
	printf("%f %f %f %f", A, B, C, D);
	
	printf("\n\n");
	printf("Positive duty\n");
	duty = statusPWM(0.5, 0.3, &prev);
	printf("Delay: %f\n", setPIN_and_Delay(&A, &B, &C, &D, duty));
	printf("%f %f %f %f", A, B, C, D);
	
	printf("\n\n");
	printf("Positive duty again\n");
	duty = statusPWM(0.4, 0.3, &prev);
	printf("Delay: %f\n", setPIN_and_Delay(&A, &B, &C, &D, duty));
	printf("%f %f %f %f", A, B, C, D);
	
	printf("\n\n");
	printf("Negative duty --> Commutation needed\n");
	duty = statusPWM(-0.5, 0.3, &prev);
	printf("Delay: %f\n", setPIN_and_Delay(&A, &B, &C, &D, duty));
	printf("%f %f %f %f", A, B, C, D);
	printf("\n%f", prev);
	
	printf("\n\n");
	printf("Same duty\n");
	duty = statusPWM(-0.5, 0.3, &prev);
	printf("Delay: %f\n", setPIN_and_Delay(&A, &B, &C, &D, duty));
	printf("%f %f %f %f", A, B, C, D);
	
	printf("\n\n");
	printf("Negative duty again\n");
	duty = statusPWM(-0.4, 0.3, &prev);
	printf("Delay: %f\n", setPIN_and_Delay(&A, &B, &C, &D, duty));
	printf("%f %f %f %f", A, B, C, D);
	
	printf("\n\n");
	printf("Positive duty --> Commutation needed\n");
	duty = statusPWM(0.5, 0.3, &prev);
	printf("Delay: %f\n", setPIN_and_Delay(&A, &B, &C, &D, duty));
	printf("%f %f %f %f", A, B, C, D);
	//OK
	return 0;
}
