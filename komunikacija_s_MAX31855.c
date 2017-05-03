#include <Cortex_M3_LPC1768_definicije_registara.h>
#include <komunikacija_s_MAX31855.h>

struct T_MAX31855_struct T_MAX31855={0,0,0,0};

union{
	struct{
		unsigned termopar_prekinut:1;
		unsigned termopar_spojen_s_masom:1;
		unsigned termopar_spojen_s_Vcc:1;
		unsigned :1;
		unsigned T_hladnog_spoja:11;
		unsigned znak_T_hladnog_spoja:1;
		unsigned greska:1;
		unsigned :1;
		unsigned T:13;
		unsigned znak_T:1;
	};
	struct{
		unsigned tip_greske:3;
	};
	unsigned short int niz_T[2];
}T_MAX31855_uputstvo;

void iscitaj_MAX31855(void)
{
	short int pom;
	
	while(SSP0SRbits.RNE)
		pom=SSP0DR;
	
//	CON9_P0_SET=1;
	while(!SSP0SRbits.TFE);
	SSP0DR=0;
	SSP0DR=0;

	while(SSP0SRbits.BSY);
//	CON9_P0_CLR=1;
	T_MAX31855_uputstvo.niz_T[1]=SSP0DR;
	T_MAX31855_uputstvo.niz_T[0]=SSP0DR;

	T_MAX31855.greska=T_MAX31855_uputstvo.greska;	
	T_MAX31855.tip_greske=T_MAX31855_uputstvo.tip_greske;
	pom=0;
	if(T_MAX31855_uputstvo.znak_T_hladnog_spoja)
	{
		pom=T_MAX31855_uputstvo.T_hladnog_spoja | 0xF800;
		T_MAX31855.T_hladnog_spoja=0.0625*pom;
	}
	else
		T_MAX31855.T_hladnog_spoja=0.0625*T_MAX31855_uputstvo.T_hladnog_spoja;
	
	if(T_MAX31855_uputstvo.znak_T)
	{
		pom=T_MAX31855_uputstvo.T | 0xE000;
		T_MAX31855.T=0.25*pom;
	}
	else
		T_MAX31855.T=0.25*T_MAX31855_uputstvo.T;
}

void citaj_MAX31855(unsigned char koja_temperatura)
{
	switch(koja_temperatura)
	{
		case T1_MAX31855:
			CS_T1_CLR=1;
			iscitaj_MAX31855();
			CS_T1_SET=1;
			break;
		case T2_MAX31855:
			CS_T2_CLR=1;
			iscitaj_MAX31855();
			CS_T2_SET=1;
			break;
		case T3_MAX31855:
			CS_T3_CLR=1;
			iscitaj_MAX31855();
			CS_T3_SET=1;
			break;
		case T4_MAX31855:
			CS_T4_CLR=1;
			iscitaj_MAX31855();
			CS_T4_SET=1;
			break;
	}
}
