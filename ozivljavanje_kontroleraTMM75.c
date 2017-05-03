#include <Cortex_M3_LPC1768_definicije_registara.h>
#define __USE_C99_MATH
#include <arm_math.h>												//Укључује core_cm3.h, math.h и string.h

#pragma diag_suppress  177, 550							//не приказује упозорења о декларисаним променљивaмa које нису више нигде наведене (177) или
																						//су наведене али нису коришћене (550)

#include <takt_rada.h>
#include <pauze.h>
#include <konverzije.h>
#include <DWIN_UART_LCM.h>
#include <komunikacija_s_MAX31855.h>
#include <CAN_komunikacija_Loshmi_protokol.h>
#include <ozivljavanje_kontroleraTMM75.h>
#include <upaljac.h>

#define Van0_1	0.0		//[V]
#define Van0_2	3.3		//[V]

#define AN0_1	4
#define AN0_2	4090

#define AN3_1	390
#define AN3_2	2930	//Вулетов џојстик

unsigned char str[20];

// extern unsigned int __heap_base, Heap_Mem, __heap_limit;
// unsigned int intg;

/************************************************ Адресиране променљиве *******************************************************************/
/************************************************ Адресиране променљиве - контрола мотора ***********************************************/
#pragma arm section rwdata = ".ARM.__at_0x10001000"
	unsigned char ID_kontrole_motora, odakle_nominala;
__packed	short int AN0_pumpa_PWM, broj_hladnih_startova, broj_uspesnih_paljenja, broj_rada_u_radnom_rezimu, broj_regularnih_gasenja;
__packed	float vreme_kontrole_motora, omega_motora, omega_motora_N, omega_motora_N_joystick, pumpa_PWM,	T_lezaja, T_izduvnih_gasova, T_hladnog_spoja,
								sila, H, P0, vreme_rada_motora_u_toku_hladnog_starta, vreme_rada_motora_u_toku_paljenja,
								vreme_rada_motora_u_toku_dovodjenja_u_radni_rezim, vreme_rada_motora_u_radnom_rezimu,
								vreme_rada_motora_u_toku_zaustavljanja;
								
__packed struct{
	unsigned starter:1;
	unsigned upaljac:1;
	unsigned gorivni_ventil:1;
	unsigned gorivni_ventil_upaljaca:1;
	unsigned pumpu:1;
	unsigned uljni_ventil:1;
}aktiviraj;

#pragma arm section rwdata

/************************************************ Променљиве за мерење угаоне брзине мотора ***********************************************/
unsigned char brojac_delta_t_w_motora=0, brojac_prekoracenja_w_motora=0;
const float delta_obrta_motora=prenosni_odnos/broj_zuba;
unsigned int TTC_w_motora_trenutno,TTC_w_motora_prethodno;
float delta_t_w_motora, delta_t_w_motora_prethodno=100000.0, delta_t_w_motora_sum=0.0,
			omega_motora_prethodno=0.0, omega_motora_abs;

/************************************************ Променљиве за упаљач ********************************************************************/
unsigned char brojac_Timer0_prekida_za_upaljac=0, broj_Timer0_prekida_za_upaljac_PWM=0,
							brojac_Timer0_prekida_za_hladjenje_upaljaca=0;
const unsigned char broj_Timer0_prekida_za_hladjenje_upaljaca=Vnu*Vnu/Vnun/Vnun-1;
unsigned int TTC_protok_goriva_trenutno,TTC_protok_goriva_prethodno;
unsigned int TTC_upaljac_trenutno, TMR_upaljac_pom;
float vreme_upaljaca=0, trenutak_azuriranja_zagrevanja;
struct DogUp_struct DogUp={0,0,0,0};

/************************************************ Променљиве за проток ********************************************************************/
unsigned char brojac_delta_t_protok_goriva=0, brojac_prekoracenja_protok_goriva=0;
const float delta_mase_goriva=ro_goriva/impulsa_po_litru_goriva;
float delta_t_protok_goriva, delta_t_protok_goriva_prethodno=100000.0, delta_t_protok_goriva_sum=0.0,
			protok_goriva, protok_goriva_prethodno=0.0;

unsigned short int AN0,AN1,AN2,AN3;
unsigned int vrednost_prekoracenja_T0;
float frekvencija_T0,
			AN0_napon,AN1_napon,AN2_napon,AN3_napon,
			T1,T2,T3,T4,T_hs1,T_hs2,T_hs3,T_hs4,
			gorivna_pumpa_PWM, uljna_pumpa_PWM, upaljac_PWM;

extern unsigned char bajt1, bajt2;

const float KU_AN0_napon=(Van0_2-Van0_1)/(AN0_2-AN0_1),
						OU_AN0_napon=Van0_1-(Van0_2-Van0_1)/(AN0_2-AN0_1)*AN0_1,
						KU_gorivna_pumpa_PWM=(gorivna_pumpa_PWM_max-gorivna_pumpa_PWM_min)/(AN3_2-AN3_1),
						OU_gorivna_pumpa_PWM=gorivna_pumpa_PWM_min-(gorivna_pumpa_PWM_max-gorivna_pumpa_PWM_min)/(AN3_2-AN3_1)*AN3_1,
						KU_uljna_pumpa_PWM=(uljna_pumpa_PWM_max-uljna_pumpa_PWM_min)/(AN3_2-AN3_1),
						OU_uljna_pumpa_PWM=uljna_pumpa_PWM_min-(uljna_pumpa_PWM_max-uljna_pumpa_PWM_min)/(AN3_2-AN3_1)*AN3_1,
						K_PWM_gorivna_pumpa_PWM=PWM_clock/(PWM_frekvencija*100.0),
						K_PWM_uljna_pumpa_PWM=PWM_clock/(PWM_frekvencija*100.0),
						K_PWM_upaljac_PWM=PWM_clock/(PWM_frekvencija*100.0);

void SysTick_prekid(void)
{
	vreme_kontrole_motora+=takt_rada;
	
	ADCRbits.SEL=AD0;
	ADCRbits.START=1;
	while(!ADGDRbits.DONE);
	ADCRbits.START=0;
	AN0=ADGDRbits.RESULT;
	
	AN0_napon=AN0;//KU_AN0_napon*AN0+OU_AN0_napon;
	
	ADCRbits.SEL=AD1;
	ADCRbits.START=1;
	while(!ADGDRbits.DONE);
	ADCRbits.START=0;
	AN1=ADGDRbits.RESULT;
	
	AN1_napon=AN1;//KU_AN1_napon*AN1+OU_AN1_napon;
	
	ADCRbits.SEL=AD2;
	ADCRbits.START=1;
	while(!ADGDRbits.DONE);
	ADCRbits.START=0;
	AN2=ADGDRbits.RESULT;
	
	AN2_napon=AN2;//KU_AN2_napon*AN2+OU_AN2_napon;
	
	ADCRbits.SEL=AD3;
	ADCRbits.START=1;
	while(!ADGDRbits.DONE);
	ADCRbits.START=0;
	AN3=ADGDRbits.RESULT;
	
	AN3_napon=AN3;//KU_AN3_napon*AN3+OU_AN3_napon;
	
	gorivna_pumpa_PWM=KU_gorivna_pumpa_PWM*AN3+OU_gorivna_pumpa_PWM;
	uljna_pumpa_PWM=gorivna_pumpa_PWM;
	upaljac_PWM=uljna_pumpa_PWM;

	if(AN3<1000)
	{
		GVU_CLR=1;
		GV_CLR=1;
		VVisak_CLR=1;
		UV_CLR=1;
		aktiviraj.upaljac=0;
	}
	else if(AN3<2000)
	{
		GVU_SET=1;
		GV_CLR=1;
		VVisak_SET=1;
		UV_CLR=1;
		aktiviraj.upaljac=0;
	}
	else
	{
		GVU_SET=1;
		GV_SET=1;
		VVisak_SET=1;
		UV_SET=1;
		aktiviraj.upaljac=1;
	}

	if(PORT1Bbits.P7)
	{
		PORT1B_CLRbits.P7=1;
		PORT1B_CLRbits.P6=1;
		PORT1A_CLRbits.P0=1;
		PORT1A_CLRbits.P1=1;
		PORT1A_CLRbits.P4=1;
		PORT1B_CLRbits.P0=1;
		PORT1B_CLRbits.P1=1;
		PORT1B_CLRbits.P2=1;
	}
	else
	{
		PORT1B_SETbits.P7=1;
		PORT1B_SETbits.P6=1;
		PORT1A_SETbits.P0=1;
		PORT1A_SETbits.P1=1;
		PORT1A_SETbits.P4=1;
		PORT1B_SETbits.P0=1;
		PORT1B_SETbits.P1=1;
		PORT1B_SETbits.P2=1;
	}
	
	citaj_MAX31855(T1_MAX31855);
	if(T_MAX31855.greska)
	{
		if(T_MAX31855.tip_greske)											//Ово ме је јебало док нисам схватио да може да има грешку
			T1=10000*T_MAX31855.tip_greske;							//нултог типа.
	}
	else
	{
		T1=T_MAX31855.T;
// 		T_lezaja=filter_po_nagibu(T_MAX31855.T,T_lezaja_prethodno,delta_T_lezaja_max);
// 		T_lezaja_prethodno=T_lezaja;
	}
	T_hs1=T_MAX31855.T_hladnog_spoja;
	
	citaj_MAX31855(T2_MAX31855);
	if(T_MAX31855.greska)
	{
		if(T_MAX31855.tip_greske)											//Ово ме је јебало док нисам схватио да може да има грешку
			T2=10000*T_MAX31855.tip_greske;							//нултог типа.
	}
	else
	{
		T2=T_MAX31855.T;
// 		T_lezaja=filter_po_nagibu(T_MAX31855.T,T_lezaja_prethodno,delta_T_lezaja_max);
// 		T_lezaja_prethodno=T_lezaja;
	}
	T_hs2=T_MAX31855.T_hladnog_spoja;

	citaj_MAX31855(T3_MAX31855);
	if(T_MAX31855.greska)
	{
		if(T_MAX31855.tip_greske)											//Ово ме је јебало док нисам схватио да може да има грешку
			T3=10000*T_MAX31855.tip_greske;							//нултог типа.
	}
	else
	{
		T3=T_MAX31855.T;
// 		T_lezaja=filter_po_nagibu(T_MAX31855.T,T_lezaja_prethodno,delta_T_lezaja_max);
// 		T_lezaja_prethodno=T_lezaja;
	}
	T_hs3=T_MAX31855.T_hladnog_spoja;
	
	citaj_MAX31855(T4_MAX31855);
	if(T_MAX31855.greska)
	{
		if(T_MAX31855.tip_greske)											//Ово ме је јебало док нисам схватио да може да има грешку
			T4=10000*T_MAX31855.tip_greske;							//нултог типа.
	}
	else
	{
		T4=T_MAX31855.T;
// 		T_lezaja=filter_po_nagibu(T_MAX31855.T,T_lezaja_prethodno,delta_T_lezaja_max);
// 		T_lezaja_prethodno=T_lezaja;
	}
	T_hs4=T_MAX31855.T_hladnog_spoja;
	
	/********************************************** Срачунавање угаоне брзине ***************************************************************/
	{
		unsigned char brojac_delta_t_pom;
		float delta_t_sum_pom, delta_t_avg_raw, omega_motora_raw;
		
		IOIEbits_w_motora.P_w_motora=0;
		T0MCRbits.MR3I=0;
		delta_t_sum_pom=delta_t_w_motora_sum;		//рачунамо с помоћним величинама јер Capture прекид може да промени стварне
		brojac_delta_t_pom=brojac_delta_t_w_motora;
		delta_t_w_motora_sum=0.0;
		brojac_delta_t_w_motora=0;
		IOIEbits_w_motora.P_w_motora=1;
		T0MCRbits.MR3I=1;
	
		if (brojac_delta_t_pom)
			delta_t_avg_raw=delta_t_sum_pom/brojac_delta_t_pom;
		else
			if(brojac_prekoracenja_w_motora>=broj_overflowa_tajmera_za_merenje_ugaone_brzine_motora)
				delta_t_avg_raw=100000.0;
			else
				delta_t_avg_raw=delta_t_w_motora_prethodno;
	
		omega_motora_raw=delta_obrta_motora/delta_t_avg_raw;

//		omega_motora=filter_po_nagibu(omega_motora_raw,omega_motora_prethodno,delta_omega_motora_max);			
		omega_motora=omega_motora_raw;	
		omega_motora_prethodno=omega_motora;
	
		omega_motora_abs=fabs(omega_motora);
		delta_t_w_motora_prethodno=delta_obrta_motora/omega_motora_abs;
	}
	
/********************************************** Срачунавање протока горива ****************************************************************/
	{
		unsigned char brojac_delta_t_pom;
		float delta_t_sum_pom, delta_t_avg_raw, protok_goriva_raw;
		
		IOIEbits_protok_goriva.P_protok_goriva=0;
		T0MCRbits.MR3I=0;
		delta_t_sum_pom=delta_t_protok_goriva_sum;		//рачунамо с помоћним величинама јер Capture прекид може да промени стварне
		brojac_delta_t_pom=brojac_delta_t_protok_goriva;
		delta_t_protok_goriva_sum=0.0;
		brojac_delta_t_protok_goriva=0;
		IOIEbits_protok_goriva.P_protok_goriva=1;
		T0MCRbits.MR3I=1;
	
		if (brojac_delta_t_pom)
			delta_t_avg_raw=delta_t_sum_pom/brojac_delta_t_pom;
		else
			if(brojac_prekoracenja_protok_goriva>=broj_overflowa_tajmera_za_merenje_protoka_goriva)
				delta_t_avg_raw=100000.0;
			else
				delta_t_avg_raw=delta_t_protok_goriva_prethodno;
	
		protok_goriva_raw=1/delta_t_avg_raw;

//		omega_motora=filter_po_nagibu(omega_motora_raw,omega_motora_prethodno,delta_omega_motora_max);			
		protok_goriva=protok_goriva_raw;	
		protok_goriva_prethodno=protok_goriva;
			
		delta_t_protok_goriva_prethodno=1/protok_goriva;
	}
	
	PWM1MR4=K_PWM_gorivna_pumpa_PWM*gorivna_pumpa_PWM;
	PWM1LERbits.M4L=1;
	
	PWM1MR6=K_PWM_uljna_pumpa_PWM*uljna_pumpa_PWM;
	PWM1LERbits.M6L=1;
	
// 	PWM1MR3=K_PWM_upaljac_PWM*upaljac_PWM;
// 	PWM1LERbits.M3L=1;

	if(aktiviraj.upaljac)
	{
		if(!DogUp.azurirao_sam_tajmer_za_upaljac)
		{
			TTC_upaljac_trenutno=T0TC;
			
			TMR_upaljac_pom=TTC_upaljac_trenutno+interval_provere_komparatora*frekvencija_T0;
			if(TMR_upaljac_pom>vrednost_prekoracenja_T0)
				T0MR0=TMR_upaljac_pom-vrednost_prekoracenja_T0;
			else
				T0MR0=TMR_upaljac_pom;
			
			T0MCRbits.MR0I=1;
			DogUp.azurirao_sam_tajmer_za_upaljac=1;
		}
	}
	else
	{
		T0MCRbits.MR0I=0;
		PWM1_upaljaca_CLR=1;
		vreme_upaljaca=0;
		DogUp.upaljac_se_ne_zagreva=0;
		DogUp.ne_azuriraj_PWM_zagrevanja=0;
		DogUp.azurirao_sam_tajmer_za_upaljac=0;
		brojac_Timer0_prekida_za_upaljac=0;
		broj_Timer0_prekida_za_upaljac_PWM=0;
	}	

// 	if(CS_T1)
// 		CS_T1_CLR=1;
// 	else
// 		CS_T1_SET=1;
// 	if(CS_T2)
// 		CS_T2_CLR=1;
// 	else
// 		CS_T2_SET=1;
// 	if(CS_T3)
// 		CS_T3_CLR=1;
// 	else
// 		CS_T3_SET=1;
// 	if(CS_T4)
// 		CS_T4_CLR=1;
// 	else
// 		CS_T4_SET=1;	
}

void glavna(void)
{
	static pauzaSysTick_tip pauzaSysTick=pauzaSysTick_pocetno;
	unsigned char string[12],i;
	static unsigned char j,k;
	struct{
		unsigned short int X;
		unsigned short int Y;
	}poz[50];
	
// 	intg=__heap_base+1;
	
	#include <reset.h>
	#include <sistemska_podesavanja.h>						//овде искључујем све прекиде
	#include <oscilator.h>
	#include <kontrola_napajanja.h>				
	#include <funkcije_pinova.h>	
 	#include <pull_up_otpornici.h>
 	#include <portovi.h>
	#include <SysTick_tajmer_za_takt_rada.h>
	#include <spoljni_prekidi.h>
	#include <uart.h>
	#include <AD_pretvarac.h>
	#include <tajmeri.h>
	#include <CAN.h>
//	#include <ethernet.h>
	#include <PWM1.h>
//	#include <MCPWM.h>
	#include <SSP.h>

	SSP0CR1bits.SSPE=1;		//Омогући SSP0	
	inicijalizuj_CAN_LP();
	
	*(unsigned char *)&aktiviraj=0;
	
	frekvencija_T0=Fosc/PCLKSEL_TIMER0/(T0PR+1);
	vrednost_prekoracenja_T0=vrednost_prekoracenja_T0_u_sekundama*frekvencija_T0;
	T0MR3=vrednost_prekoracenja_T0;
	
	PWM1TCRbits.PWM_ENABLE=1;			//Омогући PWM
	
	#include <prekidi.h>

//	sekunde_u_h_min_sec(7564.0,str);

//Одради handshake - Тouch screen-у треба 0.5-2 секунде да се иницијализује по укључењу
	brojac_handshake_LCM=0;
	while(!DogLCM.uspostavio_sam_komunikaciju_s_LCMom)
	{
		brojac_handshake_LCM++;
		HandshakeLCM();
		cekaj_da_stigne_odgovor_na_komandu();
		if(!DogLCM.odgovor_na_komandu_je_neispravan)
			if(odgovor_LCMa_na_komandu[1]=='O' && odgovor_LCMa_na_komandu[2]=='K')
				DogLCM.uspostavio_sam_komunikaciju_s_LCMom=1;
	}
	IskljuciKursorLCM();
	ObrisiEkranLCM();
	KonfiguracijaRadnogRezimaLCM(baud_115200,ne_pamti_konfiguraciju_LCMa);
	while(!isteklo_vreme(500,(pauzaSysTick_tip *)&pauzaSysTick));
 	PodesiBojeLCM(zuta,crna);
	ObrisiEkranLCM();
	
	//CrtajMrezuZaFontLCM(font_12x24);
	CrtajMrezuZaFontLCM(font_8x16);

	i=0;j=1;k=1;
	PisiTekstLCM("UART0=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("UART1=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	j=1;k++;
	PisiTekstLCM("AN0_n=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;	
	PisiTekstLCM("AN1_n=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;	
	PisiTekstLCM("AN2_n=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;	
	PisiTekstLCM("AN3_n=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;

	PisiTekstLCM("T1=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("T2=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("T3=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("T4=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("T_hs1=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("T_hs2=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("T_hs3=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("T_hs4=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	
	PisiTekstLCM("t=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("w=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("Ths=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;	
	
	j=1;k++;
	PisiTekstLCM("GP_PWM=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("UP_PWM=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("Upa_PWM=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("upaljac=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;	
	PisiTekstLCM("up_zagrejan=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("up_se_ne_za=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("t_up=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	PisiTekstLCM("protok=",j,k,font_8x16);sledeca_pozicija_teksta(&j,&k,14,59);
	poz[i].X=pozicija_kursora[0];poz[i].Y=pozicija_kursora[1];i+=1;
	
	ZapamtiPrikazLCM(lejer_za_prikaz);
	
	while(!isteklo_vreme(50,(pauzaSysTick_tip *)&pauzaSysTick));

	while(1)
	{
		i=0;

		int_u_string(bajt1,string);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,3,font_8x16,lejer_za_prikaz,1);i+=1;

		int_u_string(bajt2,string);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,3,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(AN0_napon,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(AN1_napon,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(AN2_napon,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(AN3_napon,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(T1,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(T2,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(T3,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(T4,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(T_hs1,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(T_hs2,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(T_hs3,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(T_hs4,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(vreme_kontrole_motora,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(omega_motora,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(T_hladnog_spoja,string,2);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,7,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(gorivna_pumpa_PWM,string,1);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,5,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(uljna_pumpa_PWM,string,1);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,5,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(upaljac_PWM,string,1);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,5,font_8x16,lejer_za_prikaz,1);i+=1;
		
		int_u_string(aktiviraj.upaljac,string);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,3,font_8x16,lejer_za_prikaz,1);i+=1;

		int_u_string(upaljac_zagrejan,string);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,2,font_8x16,lejer_za_prikaz,1);i+=1;
		
		int_u_string(DogUp.upaljac_se_ne_zagreva,string);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,2,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(vreme_upaljaca,string,1);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,5,font_8x16,lejer_za_prikaz,1);i+=1;
		
		float_u_string(protok_goriva,string,1);
		PrikaziStringBrojaLCM(string,poz[i].X,poz[i].Y,5,font_8x16,lejer_za_prikaz,1);i+=1;

// 		PORT0A_SETbits.P2=1;
// 		
// 		while(!isteklo_vreme(2.5,(pauzaSysTick_tip *)&pauzaSysTick));
// 		
// 		PORT0A_CLRbits.P2=1;
//

		while(!isteklo_vreme(50,(pauzaSysTick_tip *)&pauzaSysTick));
	}
}
