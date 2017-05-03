#include <Cortex_M3_LPC1768_definicije_registara.h>
#include <DWIN_UART_LCM.h>
#include <CAN_komunikacija_Loshmi_protokol.h>
#include <ozivljavanje_kontroleraTMM75.h>
#include <upaljac.h>

struct{
	unsigned RI:1;
	unsigned TI1:1;
	unsigned EI:1;
	unsigned DOI:1;
	unsigned WUI:1;
	unsigned EPI:1;
	unsigned ALI:1;
	unsigned BEI:1;
	unsigned IDI:1;
	unsigned TI2:1;
	unsigned TI3:1;
	unsigned :5;
	unsigned ERR_LOCATION:5;
	unsigned ERR_DIRECTION:1;
	unsigned ERR_TYPE:2;
	unsigned ALC:8;
}CAN1ICRbits,CAN2ICRbits;

unsigned char bajt1, bajt2;

void UART_LCM_prekid(void)
{	
	primi_odgovor_LCMa(UART_LCM_RBR);
}

void UART0_prekid(void)
{
	if(U0LSRbits.RDR)
	{
		bajt1=U0RBR;
		U0THR=++bajt1;
	}
}

void UART1_prekid(void)
{
	if(U1LSRbits.RDR)
	{
		bajt2=U1RBR;
		U1THR=++bajt2;
	}
}

void CAN_prekid(void)
{
	*(unsigned int *)&CAN2ICRbits=CAN2ICR;

	if(CAN2ICRbits.RI)
	{
		primi_CAN_LP_poruku();
		
		CAN2CMRbits.RRB=1;		//Брише Receive Buffer Status бит у CAN2GSR регистру
	}
	if(CAN2ICRbits.TI1)
		posalji_CAN_LP_poruku();
	
	if(CAN2ICRbits.BEI)
	{
		CAN2MODbits.RM=1;		//Уђи у ресет мод
		CAN2GSRbits.RXERR=0;
		CAN2GSRbits.TXERR=0;
		CAN2MODbits.RM=0;		//Изађи из ресет мода

		pauza_CAN_LP_posle_reseta();
	}
}

void Timer0_prekid(void)
{
	TTC_upaljac_trenutno=T0TC;

	if(T0IRbits.MR0)
	{
		vreme_upaljaca+=interval_provere_komparatora;
			
		if(!DogUp.upaljac_se_ne_zagreva)
		{
			if(!DogUp.ne_azuriraj_PWM_zagrevanja)
			{
				trenutak_azuriranja_zagrevanja=vreme_upaljaca;
				broj_Timer0_prekida_za_upaljac_PWM++;
				if(broj_Timer0_prekida_za_upaljac_PWM>broj_Timer0_prekida_za_upaljac_PWM_max)
					broj_Timer0_prekida_za_upaljac_PWM=broj_Timer0_prekida_za_upaljac_PWM_max;
				DogUp.ne_azuriraj_PWM_zagrevanja=1;
			}

			if(vreme_upaljaca>trenutak_azuriranja_zagrevanja+vreme_zagrevanja_pri_konstantnom_PWMu)
				DogUp.ne_azuriraj_PWM_zagrevanja=0;
			
			if(brojac_Timer0_prekida_za_upaljac<broj_Timer0_prekida_za_upaljac_PWM)
			{
				PWM1_upaljaca_SET=1;
				brojac_Timer0_prekida_za_upaljac++;
			}
			else if(brojac_Timer0_prekida_za_upaljac<=broj_Timer0_prekida_za_hladjenje_upaljaca)
			{
				PWM1_upaljaca_CLR=1;
				brojac_Timer0_prekida_za_upaljac++;
				if(brojac_Timer0_prekida_za_upaljac>broj_Timer0_prekida_za_hladjenje_upaljaca)
					brojac_Timer0_prekida_za_upaljac=0;
			}
			else
				brojac_Timer0_prekida_za_upaljac=0;

			if(upaljac_zagrejan)
			{
				DogUp.upaljac_se_ne_zagreva=1;
				DogUp.ne_azuriraj_PWM_zagrevanja=0;
				broj_Timer0_prekida_za_upaljac_PWM=0;
				brojac_Timer0_prekida_za_upaljac=0;
				PWM1_upaljaca_CLR=1;
				DogUp.upaljac_se_hladi=1;
			}
		}
		else
		{
			if(!DogUp.upaljac_se_hladi)
			{
				if(upaljac_zagrejan)
				{
					PWM1_upaljaca_CLR=1;
					DogUp.upaljac_se_hladi=1;
					brojac_Timer0_prekida_za_hladjenje_upaljaca++;
				}
			}
			else
			{
				if(++brojac_Timer0_prekida_za_hladjenje_upaljaca>broj_Timer0_prekida_za_hladjenje_upaljaca)
				{
					PWM1_upaljaca_SET=1;
					DogUp.upaljac_se_hladi=0;
					brojac_Timer0_prekida_za_hladjenje_upaljaca=0;
				}
			}
		}

		TMR_upaljac_pom=TTC_upaljac_trenutno+interval_provere_komparatora*frekvencija_T0;
		if(TMR_upaljac_pom>vrednost_prekoracenja_T0)
			T0MR0=TMR_upaljac_pom-vrednost_prekoracenja_T0;
		else
			T0MR0=TMR_upaljac_pom;
		
		T0IR=MR0_flag;
	}

	if(T0IRbits.MR3)
	{
		if(brojac_prekoracenja_w_motora<255)
			brojac_prekoracenja_w_motora++;
		if(brojac_prekoracenja_protok_goriva<255)
			brojac_prekoracenja_protok_goriva++;
		T0IR=MR3_flag;
	}
}

void EINT3_prekid(void)
{
	TTC_w_motora_trenutno=TTC_w_motora;
	TTC_protok_goriva_trenutno=TTC_protok_goriva;

	if(IOIRPbits_w_motora.P_w_motora)
	{
		if(T0IRbits.MR3)
		{
			brojac_prekoracenja_w_motora++;
			brojac_prekoracenja_protok_goriva++;
			T3IR=MR3_flag;												//да не би узео прекорачење два пута
		}
		delta_t_w_motora=TTC_w_motora_trenutno-TTC_w_motora_prethodno+brojac_prekoracenja_w_motora*vrednost_prekoracenja_T0;
		delta_t_w_motora=delta_t_w_motora/frekvencija_T0;
		brojac_delta_t_w_motora++;
		if (delta_t_w_motora<delta_t_w_motora_min)
			brojac_delta_t_w_motora--;
		else
			delta_t_w_motora_sum+=delta_t_w_motora;
		brojac_prekoracenja_w_motora=0;
		TTC_w_motora_prethodno=TTC_w_motora_trenutno;

		IOICbits_w_motora.P_w_motora=1;
	}

	if(IOIRPbits_protok_goriva.P_protok_goriva)
	{
		if(T0IRbits.MR3)
		{
			brojac_prekoracenja_w_motora++;
			brojac_prekoracenja_protok_goriva++;
			T0IR=MR3_flag;										//да не би узео прекорачење два пута
		}
		delta_t_protok_goriva=TTC_protok_goriva_trenutno-TTC_protok_goriva_prethodno+
			brojac_prekoracenja_protok_goriva*vrednost_prekoracenja_T0;
		delta_t_protok_goriva=delta_t_protok_goriva/frekvencija_T0;
		brojac_delta_t_protok_goriva++;
		if (delta_t_protok_goriva<delta_t_protok_goriva_min)
			brojac_delta_t_protok_goriva--;
		else
			delta_t_protok_goriva_sum+=delta_t_protok_goriva;
		brojac_prekoracenja_protok_goriva=0;
		TTC_protok_goriva_prethodno=TTC_protok_goriva_trenutno;

		IOICbits_protok_goriva.P_protok_goriva=1;
	}
}

