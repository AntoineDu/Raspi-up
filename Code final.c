//************************************************************************************
//** PIN A5 TRIAC impulsion
//** PIN E1 TRIAC puissance
//** PIN D0 LED 1
//** PIN D1 LED 2
//** PIN C6 Tx
//** PIN C7 Rx
//** PIN B2 RECEPTEUR RF
//** PIN D7 EMETTEUR RF
//** PIN C1 PWM
//************************************************************************************

#include <18F4550.h>

#fuses NOBROWNOUT,NOLVP,HS,NOPROTECT,NODEBUG //,NOPROTECT,NOLVP,NODEBUG,USBDIV,VREGEN,ECPLL, ECPIO

#use delay(clock=24000000,crystal=20000000)
#use rs232(baud=19200, xmit=PIN_C6, rcv=PIN_C7, stream=RS232)
#USE PWM(OUTPUT=PIN_C1, FREQUENCY=10kHz, DUTY=80, stream=pwm1)
#byte INTCON2 = 0xFF1
#bit  RBPU = INTCON2.7

//#include "..\include\LCD420_S3.c"

// Variables :

const int16 PosCharge = 4000, PosZero = 48000, PosUn = 49000, PosDeux = 50000, PosTrois = 51000, EcartRxInf = 100, EcartRxSup = 1100;

const int8 BUFFER_SIZE = 16;

const int16 Puissance_Triac[9] = {40000, 35000, 30000, 25000, 20000, 15000}; 

int16 Timer0_Rx = 0, TimerTx[20], PWM_Duty = 500;

int8 DeuxBitsTx = 0, N_AlternanceTx = 0, N_AlternanceTxMax = 16, N_AlternanceRx = 0, N_AlternanceRxPrec = 0, N_AlternanceRxMax = 16, VarBoucleTx = 0, Charge_Valeur = 0;

int8 buffer[BUFFER_SIZE], next_in = 0, next_out = 0, SerialChar[5], N_Alternance = 1, MessageTx[5], N_MessageTx = 1, DeuxBitsRx = 0, N_DeuxBitsRx = 0, MessageRx[5], N_MessageRx = 1, AdresseModule = 228;

int1 FlagBouton = 0, FlagRx = 0, FlagRxAlternance = 0, StartTx = 0, TxOn = 0, RxOn = 0, Flag_Positif = 0, Flag_WtD_P = 0,Flag_TriacVar = 0, Flag_Triac = 0, Tx_Ack = 0, Wait_Ack = 0;

int16 NbrImpEXT2 = 0;
int8 FlamingoTrame[25] = {2,2,2,1,2,2,3,2,2,2,2,1,2,2,2,2,3,2,2,2,2,2,2,2,2};
int8 On[4] = {2,2,1,3}, Off[4] = {2,2,2,2}, OnAll[4] = {2,1,2,3}, OffAll[4] = {2,1,3,2};
int8 AdresseA[4] = {2,2,1,2}, AdresseB[4] = {2,1,3,1}, AdresseC[4] = {2,1,2,2}, AdresseD[4] = {1,3,2,1};
int8 AdresseR[4], EtatR[4];
int8 impulsionTOT;
int8 x=0;
int8 tableau[33];
int1 synchro = 0;	
int8 impulsion = 0;
int8 tableautest[33] = {2,2,2,1,2,2,3,2,2,2,2,1,2,2,2,2,3,2,2,2,2,2,2,2,2,2,2,1,3,2,2,1,2};
int16 Valeur_Timer0 = 0;
int1 synchro_timer=0;
int16 Compteur_ActivationRF = 0;
int1 Autorisation_Gachette = 0;

#define bkbhit (next_in != next_out)

/*void Acknowledge();
void ReceptionRF();*/
void SetupEnvoiWireless(int8 Etat[],int8 TailleEtat, int8 Adresse[],int8 TailleAdresse);
void EnvoiWireless(int8 Trame[], int8 data);
void initialisation()  // Routine d'initialisation
{
    set_tris_a(0b00000001);
    output_a(0);
    set_tris_b(255);
    RBPU = 0;   // Active le pull up sur les IO du port B mis en entrée
    set_tris_d(0b00000000);
    output_d(0);    // Tous les pins à 0
    setup_ccp1(CCP_CAPTURE_RE);     // Configure CCP1 sur flanc montant
	setup_ccp2(CCP_PWM);
	//set_pwm2_duty((int16)0);
	//pwm_on(pwm1);
//	pwm_set_duty_percent(pwm1,250);
    setup_comparator(A0_VR_A1_VR);      // Active les comparateurs avec référence interne
    setup_vref(VREF_LOW | 6); // Règle la référence à 5V / 4
    setup_timer_0(RTCC_DIV_1|RTCC_INTERNAL);    // Timer 0 clock interne et prédiviseur de 1
    setup_timer_1(T1_DIV_BY_1|T1_INTERNAL);     // Timer 1 clock interne et prédiviseur de 1
    enable_interrupts(INT_CCP1);    // Active l'interruption sur CCP1
    enable_interrupts(INT_COMP);    // Active l'interruption sur le comparateur
	clear_interrupt(INT_COMP);
    clear_interrupt(INT_RDA);
    enable_interrupts(INT_RDA);    // Active l'interruption sur le port série
//	clear_interrupt(INT_EXT2);		
//	enable_interrupts(INT_EXT2);	//Active interruption sur l'entrée RC6 ou 7 pour réception RF
//	ext_int_edge(2, L_TO_H);
	disable_interrupts(INT_EXT2);
    enable_interrupts(GLOBAL);    // Active l'interruption globale
	write_eeprom(1, 2);
    AdresseModule = read_eeprom(1);
	//write_eeprom(2, 1);
    Flag_Triac = read_eeprom(2);
    if(Flag_Triac) 
	{	
		Charge_Valeur = 8;		
		output_high(PIN_E1);
	}
    printf("%u ",AdresseModule);
}

int8 bgetc() // lecture du buffer serie
{						
   int8 c;
   while(!bkbhit) ;				// tant que le buffer n'est pas vide
   c=buffer[next_out];				// lecture du buffer
   next_out=(next_out+1) % BUFFER_SIZE;		// mise a jour du pointeur
   return(c);
}

void SetupTransmission(int8 AdresseRecepteur, int8 Commande)    // Remplis le tableau des valeurs pour Timer1 en fonction des données à envoyer
{	
	/*if(AdresseRecepteur == 0 && Commande == 0)	//Acquittement
	{
		Tx_Ack = 1;	
		TxOn = 0;
        StartTx = 1;
		Transmission();
	}*/
    if(!TxOn && !StartTx)
    {
        N_MessageTx = 1;
        MessageTx[1] = 150;		//2 1 1 2 (impulsion de "synchro")
        MessageTx[2] = AdresseRecepteur;
        MessageTx[3] = AdresseModule;
        MessageTx[4] = Commande;
		//MessageTx[5] = 20;		//nombre d'impulsion à envoyer pour vérification (4* 5 octets)

        for(VarBoucleTx = 1 ; VarBoucleTx <= N_AlternanceTxMax ; VarBoucleTx++)		//transforme binaire en impulsion
        {
            if(VarBoucleTx == 1 || VarBoucleTx == 5 || VarBoucleTx == 9 || VarBoucleTx == 13) DeuxBitsTx = MessageTx[N_MessageTx] & 3;

            if(VarBoucleTx == 2 || VarBoucleTx == 6 || VarBoucleTx == 10 || VarBoucleTx == 14) DeuxBitsTx = MessageTx[N_MessageTx] >> 2 & 3;

            if(VarBoucleTx == 3 || VarBoucleTx == 7 || VarBoucleTx == 11 || VarBoucleTx == 15) DeuxBitsTx = MessageTx[N_MessageTx] >> 4 & 3;

            if(VarBoucleTx == 4 || VarBoucleTx == 8 || VarBoucleTx == 12 || VarBoucleTx == 16)
            {
                DeuxBitsTx = MessageTx[N_MessageTx] >> 6 & 3;
                N_MessageTx = N_MessageTx + 1;		//octet à transmettre transformer
				/*if(N_MessageTx == 4)
				{
					Wait_Ack = 1;
				}*/
            }

            switch(DeuxBitsTx)		//position de l'impulsion fonction des 2 bits 
            {
                case 0 :
                    TimerTx[VarBoucleTx] = 65535 - PosZero;
                break;
                case 1 :
                    TimerTx[VarBoucleTx] = 65535 - PosUn;
                break;
                case 2 :
                    TimerTx[VarBoucleTx] = 65535 - PosDeux;
                break;
                case 3 :
                    TimerTx[VarBoucleTx] = 65535 - PosTrois;
                break;
            }
        }
        TxOn = 0;
        StartTx = 1;
    }
}

void Transmission()    // Règle le timer à chaque alternance si besoin
{
    if(TxOn && !StartTx)
    {
	/*	if(Tx_Ack)
		{
			Acknowledge();
			return;
		}*/
        if(0 < N_AlternanceTx && N_AlternanceTx <= N_AlternanceTxMax)
        {
            set_timer1(TimerTx[N_AlternanceTx]);
        }
        else
        {
            TxOn = 0;
            StartTx = 0;
            disable_interrupts(INT_TIMER1);    // Desactive l'interruption sur Timer1
            clear_interrupt(INT_TIMER1);
            enable_interrupts(INT_COMP);    // Active l'interruption sur le comparateur
            clear_interrupt(INT_COMP);
        }
    }

	
    if(StartTx && !TxOn) // On active l'interruption sur timer1 et on le règle pour une impulsion de charge du condensateur
    {
        set_timer1(65535 - PosCharge);
        disable_interrupts(INT_COMP);    // Desactive l'interruption sur le comparateur
        enable_interrupts(INT_TIMER1);    // Active l'interruption sur Timer1
        clear_interrupt(INT_TIMER1);
        N_AlternanceTx = 0;   // Met le compteur d'alternance à 0
    }
}

void Reception()    // Analyse les impulsions détectées par le comparateur pour reconstituer le message
{
    /*if(get_timer0() > 40000)
    {
        enable_interrupts(INT_COMP);
		clear_interrupt(INT_COMP);
    }*/
    if(FlagRx)
    {
        output_high(PIN_D1);

        if(48000 < Timer0_Rx < 52000 && !FlagRxAlternance)
        {

            // On regarde quelle position d'impulsion on a reçue :
            if(PosZero + EcartRxInf < Timer0_Rx && Timer0_Rx < PosZero + EcartRxSup) // Position 0 = 00
            {
                DeuxBitsRx = 0;
                FlagRxAlternance = 1;
//				disable_interrupts(INT_COMP);
            }
            if(PosUn + EcartRxInf < Timer0_Rx && Timer0_Rx < PosUn + EcartRxSup) // Position 1 = 01
            {
                DeuxBitsRx = 1;
                FlagRxAlternance = 1;
//				disable_interrupts(INT_COMP);
            }
            if(PosDeux + EcartRxInf < Timer0_Rx && Timer0_Rx < PosDeux + EcartRxSup) // Position 2 = 10
            {
                DeuxBitsRx = 2;
                FlagRxAlternance = 1;
//				disable_interrupts(INT_COMP);
            }
            if(PosTrois + EcartRxInf < Timer0_Rx && Timer0_Rx < PosTrois + EcartRxSup) // Position 3 = 11
            {
                DeuxBitsRx = 3;
                FlagRxAlternance = 1;
//				disable_interrupts(INT_COMP);
            }

            // Si on a pas reçu de position valide on quitte :
            if(!FlagRxAlternance)
            {
                FlagRx = 0;
                output_low(PIN_D1);
                return;
            }

            if(!RxOn)		//si impulsion ratée ou impulsion synchro pas bonne
            {
                RxOn = 1;
                N_AlternanceRx = 1;
                N_AlternanceRxPrec = 0;
                N_DeuxBitsRx = 1;
                N_MessageRx = 1;
            }
            else
            {
                N_DeuxBitsRx++;
				//AJOUTER COMPTEUR IMPULSION POUR VERIFICATION FULL OCTET???
            }

            // On place les deux bits obtenus au bon endroit :
            if((N_AlternanceRxPrec + 1) == N_AlternanceRx) // Si on a pas raté d'impulsion
            {
                switch(N_DeuxBitsRx)		//entre autre transforme impulsion en binaire
                {
                    case 1 : // Bits 0 et 1 :
                        MessageRx[N_MessageRx] = 0;
                        MessageRx[N_MessageRx] = MessageRx[N_MessageRx] + DeuxBitsRx;
						//if(N_MessageRx)Acknowledge();
                        if(N_MessageRx == 1 && DeuxBitsRx != 2) RxOn = 0; // Vérifie que la première impulsion du premier octet est 10 sinon on recommence
                        break;
                    case 2 : // Bits 2 et 3 :
                        MessageRx[N_MessageRx] = MessageRx[N_MessageRx] + (DeuxBitsRx << 2);
                        if(N_MessageRx == 1 && DeuxBitsRx != 1) RxOn = 0; // Vérifie que la deuxième impulsion du premier octet est 01 sinon on recommence
                        break;
                    case 3 : // Bits 4 et 5 :
                        MessageRx[N_MessageRx] = MessageRx[N_MessageRx] + (DeuxBitsRx << 4);
                        if(N_MessageRx == 1 && DeuxBitsRx != 1) RxOn = 0; // Vérifie que la troisème impulsion du premier octet est 01 sinon on recommence
                        break;
                    case 4 : // Bits 6 et 7 :
                        MessageRx[N_MessageRx] = MessageRx[N_MessageRx] + (DeuxBitsRx << 6);
                        if(N_MessageRx == 1 && DeuxBitsRx != 2) RxOn = 0; // Vérifie que la quatrième impulsion du premier octet est 10 sinon on recommence
                        N_MessageRx++;
                        N_DeuxBitsRx = 0;
                        break;
                }
                if(N_AlternanceRx == N_AlternanceRxMax)
                {
//                    printf("%u %u %u ",MessageRx[2],MessageRx[3],MessageRx[4]);
					//SetupTransmission(0,0);
                    Flag_WtD_P = 1;
                    RxOn = 0;
                }
                N_AlternanceRxPrec++;
			/*///	if(N_AlternanceRxPrec == MessageRx[5]) 
				{
					Tx_Ack = 1;
				    TxOn = 0;
				    StartTx = 1;
				}*/
            }
            else RxOn = 0; // On a raté une impulsion on redémarre la réception
        }
        FlagRx = 0;
        output_low(PIN_D1);
    }
}
/*
void Acknowledge()
{	
	if(Wait_Ack)
	{
		if(MessageRx[1] == 3)
		{
			printf("ACK");			
			N_DeuxBitsRx = 0;
            N_AlternanceRx = 1;
            N_AlternanceRxPrec = 0;
			FlagRxAlternance = 0;
		}
		else
		{
			SetupTransmission(SerialChar[2],SerialChar[3]);	
		}
		Wait_Ack = 0;
	}
	if(Tx_Ack)
	{
		set_timer1(65535 - PosTrois);
	}
}*/
void WhatToDoPowerline()    // Fonction qui décide de quoi faire en fonction de la commande reçue par courannt porteur
{
    if(Flag_WtD_P /*&& !Tx_Ack*/)
    {		
        int8 AdresseSource = 0;
        if(MessageRx[2] == AdresseModule)		//en reception "AdresseRecepteur" devient "AdresseModule"
        {
            AdresseSource = MessageRx[3];
			//putc(AdresseModule);
            switch(MessageRx[4]) // On regarde quelle commande on a reçue
            {
                case 0 : // Demande de confirmation de sa présence
                    delay_ms(50);
                    SetupTransmission(AdresseSource,1); // Réponse
                break;
                case 1 : // Confirme la présence d'un autre module après une demande
                    printf("1");
                break;

                case 20 : // Allume le triac de puissance
                    output_high(PIN_E1);
                    Flag_Triac = 1;
					Flag_TriacVar = 0;
                    write_eeprom(2,1);
					Charge_Valeur = 8;
                break;
                case 21 : // Eteint le triac de puissance
                    output_low(PIN_E1);
                    Flag_Triac = 0;
					Flag_TriacVar = 0;
                    write_eeprom(2,0);
					Charge_Valeur = 0;
                break;

 /*               case 22 :
					//output_low(PIN_E1);
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur += 1;
                    if(Charge_Valeur >= 8) 
					{
						Charge_Valeur = 8;
						Flag_TriacVar = 0;
						output_high(PIN_E1);
					}                                               
                break;
                case 23 :
					if(Charge_valeur == 8)output_low(PIN_E1);
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur -= 1; 
                    if(Charge_Valeur <= 0)
					{	
						output_low(PIN_E1);
						Flag_Triac = 0;
						Flag_TriacVar = 0;
						Charge_Valeur = 0;
						write_eeprom(2,0);                
					}                
                break;*/

                case 22 :
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur = 0;                                  
                break;
                case 23 :
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur = 1;                                  
                break;
                case 24 :
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur = 2;                                  
                break;

                case 25 :
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur = 3;                                  
                break;

                case 26 :
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur = 4;                               
                break;

                case 27 :
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur = 5;                                 
                break;
         /*       case 28 :
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur = 6;                                  
                break;
                case 29 :
                    Flag_Triac = 1;
                    write_eeprom(2,1);                    
                    Flag_TriacVar = 1;
					Charge_Valeur = 7;                                  
                break;*/

				case 30 :
					SetupEnvoiWireless(On, 4, AdresseA,4);
				break;
				case 31 :
					SetupEnvoiWireless(Off, 4, AdresseA,4);
				break;
				case 32 :
					SetupEnvoiWireless(On, 4, AdresseB,4);
				break;
				case 33 :
					SetupEnvoiWireless(Off, 4, AdresseB,4);
				break;
				case 34 :
					SetupEnvoiWireless(On, 4, AdresseC,4);
				break;
				case 35 :
					SetupEnvoiWireless(Off, 4, AdresseC,4);
				break;
				case 36 :
					SetupEnvoiWireless(On, 4, AdresseD,4);
				break;
				case 37 :
					SetupEnvoiWireless(Off, 4, AdresseD,4);
				break;
				case 38 :
					SetupEnvoiWireless(OnAll, 4, AdresseA,4);
				break;
				case 39 :
					SetupEnvoiWireless(OffAll, 4, AdresseA,4);
				break;


                case 15 : // Demande de l'état du triac de puissance
                    delay_ms(50);
                    if(Flag_Triac) SetupTransmission(AdresseSource,12); // Réponse si allumé
                    else SetupTransmission(AdresseSource,13); // Réponse si éteint
                break;

                case 12 : // Confirme l'état ON du triac d'un autre module après une demande
                    printf("1");
                break;
                case 13 : // Confirme l'état OFF du triac d'un autre module après une demande
                    printf("0");
                break;
            }
			
        }
        Flag_WtD_P = 0;
    }
}

void WhatToDoPSerial()    // Fonction qui décide de quoi faire en fonction de la commande reçue par liaison série
{
    while(bkbhit)
    {
        SerialChar[0] = bgetc();
        if(SerialChar[0] == '$')
        {
            SerialChar[1] = bgetc();
            SerialChar[2] = bgetc();
            SerialChar[3] = bgetc();

            switch(SerialChar[1])
            {
                case 'A' : // 65 = A = Adresse. On écrit l'adresse dans le registre 1 de l'eeprom interne
                    write_eeprom(1,SerialChar[3]);
                    AdresseModule = read_eeprom(1);
                    printf("AC_OK %d ",AdresseModule);
                break;

                case 'C' : // 67 = C = Commande. On envoie une commande, C2 est l'adresse où envoyer et C3 la commande.
//                    printf("A : %u C : %u ",SerialChar[2],SerialChar[3]);
                    SetupTransmission(SerialChar[2],SerialChar[3]);
                break;
			
				/*case 'E' : //69 = E = Commande RF
					switch(SerialChar[2])
					{
						int8 AdresseSendRF[4];
						case 1 :
							AdresseSendRF = AdresseA;
						break;
						case 2 :
							AdresseSendRF = AdresseB;
						break;
						case 3 :
							AdresseSendRF = AdresseC;
						break;
						case 4 :
							AdresseSendRF = AdresseD;
						break;
						case 5 :
							AdresseSendRF = AdresseE;
						break;
					}
					switch(SerialChar[2])
					{	
						int8 EtatSendRF[4];
						case 1 :
							EtatSendRF = AdresseA;
						break;
						case 2 :
							EtatSendRF = AdresseB;
						break;
						case 3 :
							EtatSendRF = AdresseC;
						break;
						case 4 :
							EtatSendRF = AdresseD;
						break;
					}					*/
				//	SetupEnvoiWireless(EtatSendRF[4],AdresseSendRF[4]);
            }
        }
        else
        {
            while(bkbhit) bgetc();    // vidage buffer si non OK
        }
    }    
}
/*
void ReceptionRF()
{	
	disable_interrupts(INT_EXT2);	
	int8 OkOn = 0, OkOff = 0, OkOnAll = 0, OkOffAll = 0;
	int8 EtatModule = 0;
	int8 i=0;
	int8 j=0;
	for(i = 0; i<25; i++)
	{	
		if(tableau[i] != FlamingoTrame[i])
		{
			enable_interrupts(INT_EXT2);
			return;
		}
	}
	for(i = 29; i<33; i++)
	{	
		AdresseR[j]=tableau[i];
		if(AdresseR[j] != AdresseB[j]) 
		{
			enable_interrupts(INT_EXT2);
			return;
		}
		j++;
	}	
	j = 0;
	
	for(i = 25; i<29; i++)
	{	
		EtatR[j]=tableau[i];
		if(EtatR[j] != On[j]) //OkOn++;
		{
			enable_interrupts(INT_EXT2);
			return;
		}
		else 
		{
			if(EtatR[j] == On[j]) OkOn++;	
			if(EtatR[j] == Off[j]) OkOff++;
			if(EtatR[j] == OnAll[j]) OkOnAll++;
			if(EtatR[j] == OffAll[j]) OkOffAll++;		
	
			if((OkOn || OkOff || OkOnAll || OkOffAll) == 4)
			{
				if(OkOn == 4) EtatModule = 1;	
				if(OkOff == 4) EtatModule = 2;	
				if(OkOnAll == 4) EtatModule = 3;	
				if(OkOffAll == 4) EtatModule = 4;	
			//	output_high(PIN_D1);
			}	
		}		
		j++;
	} 
	//SetupEnvoiWireless(EtatR,4, AdresseR, 4);
	
	switch(EtatModule)
	{
		case 1 :
			output_high(PIN_E1);
			write_eeprom(2,1);
			Flag_Triac = 1;
		break;
		
		case 2 :
			output_low(PIN_E1);
			write_eeprom(2,0);
			Flag_Triac = 0;
		break;	
	
		case 3 :
			output_high(PIN_E1);
			write_eeprom(2,1);
			Flag_Triac = 1;
		break;
		
		case 4 :
			output_low(PIN_E1);
			write_eeprom(2,0);
			Flag_Triac = 0;
		break;	
	}
	if((OkOn || OkOff || OkOnAll || OkOffAll) == 4)
	{
		
		disable_interrupts(INT_EXT2);
		NbrImpEXT2 = 0;
		enable_interrupts(INT_TIMER1);
		enable_interrupts(INT_COMP);
		enable_interrupts(INT_CCP1);
		enable_interrupts(INT_RDA);		
	} 
	else enable_interrupts(INT_EXT2);		
	
}
*/
void SetupEnvoiWireless(int8 Etat[],int8 TailleEtat, int8 Adresse[], int8 TailleAdresse)
{
	int8 u=0;
//	disable_interrupts(INT_EXT2);
	disable_interrupts(INT_TIMER1);
	disable_interrupts(INT_COMP);
	disable_interrupts(INT_CCP1);
//	disable_interrupts(INT_RDA);

	for(u = 0; u < 10; u++)	// Send trame
	{
//		output_high(PIN_D1);	
		output_high(PIN_D7);		//synchro
		delay_us(250);
		output_low(PIN_D7);
		delay_us(2600);
		
	  	EnvoiWireless(FlamingoTrame, 25);
		EnvoiWireless(Etat, 4);
		EnvoiWireless(Adresse, 4);
		//output_low(PIN_D1);
		delay_ms(8);
	}
//	enable_interrupts(INT_EXT2);
	enable_interrupts(INT_TIMER1);
	enable_interrupts(INT_COMP);
	enable_interrupts(INT_CCP1);
//	enable_interrupts(INT_RDA);

}
void EnvoiWireless(int8 Trame[], int8 data)
{
	int8 z;
	int8 j;
	for(z = 0; z < data; z++)	// Send trame
	{	
		switch(Trame[z])
		{
			case 1 :
				output_high(PIN_D7);
				delay_us(250);
			break;
			
			case 2 :							
				output_high(PIN_D7);
				delay_us(250);
				output_low(PIN_D7);
				delay_us(300);
				output_high(PIN_D7);
				delay_us(250);
			break;
			
			case 3 :						
				for(j=0; j <2 ; j++)
				{
					output_high(PIN_D7);
					delay_us(250);
					output_low(PIN_D7);
					delay_us(300);
				}
				output_high(PIN_D7);
				delay_us(250);
			break;
		}
		output_low(PIN_D7);
		delay_us(1300);
	}	
}
/*
void AdjustComparator()
{	
	disable_interrupts(INT_COMP);
	int16 valeur_timer;
	valeur_timer = get_timer0();
	while(valeur_timer > 20000 && valeur_timer < 40000)
	{
		if(C1OUT)
		{	
			PWM_Duty -= 10;
			pwm_set_duty_percent(pwm1, PWM_Duty);
		}
		else
		{
			PWM_Duty += 10;
			pwm_set_duty_percent(pwm1, PWM_Duty);
		}
		valeur_timer = get_timer0();			
	}
	enable_interrupts(INT_COMP);	
}*/
void main()  ////////////////////////////////////////////////////////////////////////////////////// MAIN ///////////////////////////////////////////////////////////////////////////////////////////////////////
{	
	int8 a = 0;
    initialisation();
    delay_ms(500);   

    while(TRUE)
    {
		//SetupEnvoiWireless(On, 4, AdresseA,4);
        Reception();

        WhatToDoPowerline();

        WhatToDoPSerial();
	  
		if(Flag_Triac && Flag_TriacVar && Autorisation_Gachette)
	    {
	        if((get_timer0() > Puissance_Triac[Charge_Valeur]) ) 
			{
				output_high(PIN_E1);
				delay_us(10);
				output_low(PIN_E1);
				Autorisation_Gachette = 0;
			}
	    }
	//	AdjustComparator();
		
			 
		
        /*if((N_Alternance == 10) && FlagBouton)
        {
            SetupTransmission(27,228); // 228 = 11 10 01 00  et 27 = 00 01 10 11
        }

        if(input_state(PIN_B6) == 0 || input_state(PIN_B4) == 0)
        {
            FlagBouton = !FlagBouton;
            delay_ms(250);
        }*/        

    }
}   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#int_CCP1 
void CCP1_isr() // Interruption sur passage par zéro
{	
//output_high(PIN_D1);
    if(get_timer0() > 59000)
    {
        set_timer0(0); // Remise à 0 du timer0
		Autorisation_Gachette = 1;     
		Flag_Positif = !Flag_Positif;
        if(Flag_Positif)
        {
            output_high(PIN_D0);
            setup_ccp1(CCP_CAPTURE_FE); // On passe l'interruption sur flanc descendant
        }
        else
        {
            output_low(PIN_D0);
            setup_ccp1(CCP_CAPTURE_RE); // On passe l'interruption sur flanc montant
        }
        N_Alternance++;
        N_AlternanceRx++;
        N_AlternanceTx++;
		Compteur_ActivationRF++;
        if(N_Alternance == 51)N_Alternance = 1;	 
   
		Transmission(); // Appel de la fonction de transmission
        FlagRxAlternance = 0; // Comme on démarre une nouvelle alternance on peut recommencer à regarder pour une réception

		//disable_interrupts(INT_COMP); 
		clear_interrupt(INT_CCP1); // Clear du flag d'interruption
    }
//output_low(PIN_D1);
}

#int_COMP
void COMP_isr() // Interruption sur comparateur pour la réception
{
output_high(PIN_D1);
    if(!FlagRx && !C1OUT)
    {
    	Timer0_Rx = get_timer0();
    	FlagRx = 1;
    }
    clear_interrupt(INT_COMP);
output_low(PIN_D1);
}

#int_TIMER1 // Interruption sur timer pour l'envoi des impulsions
void TIMER1_isr()
{
    if(TxOn || StartTx)			
    {
        output_high(PIN_A5);
        delay_us(100);
        output_low(PIN_A5);
        if(StartTx)				//si impulsion de charge "StartTx" faite on lance la transmission "TxOn"
        {
            StartTx = 0;
            TxOn = 1;
        }
    }
/*	if(Tx_Ack && !StartTx)
	{
		Tx_Ack = 0;
	}*/
    clear_interrupt(INT_TIMER1);
}

#int_RDA // Interruption sur port série
void serial_isr()
{
    int t;
    buffer[next_in] = getc();				// remplissage du buffer
    t = next_in;
    next_in = (next_in + 1) % BUFFER_SIZE;
    if(next_in == next_out)
    next_in = t;
}
/*
#INT_EXT2 //interruption réception RF
void  EXT2_isr(void) 
{	//1500 pour 250µs		1800 pour 300µs		15000 pour 2,6 ms et 7500 pour 1,3 ms
	int16 Valeur_TMR = 0;
	NbrImpEXT2++;

//	output_high(PIN_D1);

	Valeur_Timer0 = get_timer0();
//		output_low(PIN_D1);
	if(Valeur_Timer0 > 25000)		//vérifie tjs synchro
	{
	
		x = 0;
		synchro = 0;
		impulsion = 0;	
		impulsionTOT=0;	
	}
	else if(Valeur_Timer0 > 15000)			//SYNCHRO
	{
		synchro = 1;
		impulsion++;
		impulsionTOT++;
		set_timer0(0);	
		Valeur_Timer0 = 0;
	//	output_high(PIN_D1);
	}
	
	if(Valeur_Timer0 > 7500 && synchro == 1)	//comptage ETAT
	{	
		tableau[x] = impulsion;
		impulsion = 0;
		x++;		
	}
	if(Valeur_Timer0 > 1800 && synchro == 1)	//comptage IMPULSION
	{
		if(impulsionTOT < 64)
		{
			impulsion++;
			impulsionTOT++;
		}
	}	
	if(Valeur_Timer0 > 4000 && x==33)		//trame FINISH
	{
		output_high(PIN_D1);
		impulsion = 0;
		impulsionTOT = 0;
		x=0;
		synchro = 0;
		synchro_timer = 0;
		ReceptionRF();
		//output_low(PIN_D1);
//EnvoiWireless(tableau, 33);			
	}	
	set_timer0(0);	
	while(input(PIN_B2));
	if(get_timer0() > 1600)	
	{			
		impulsion = 0;
		impulsionTOT = 0;
		x = 0;
		synchro = 0;		
		//return;
	}
		
	if(NbrImpEXT2 > 200);
	{
//		output_low(PIN_D1);
		NbrImpEXT2 = 0;
		disable_interrupts(INT_EXT2);
		enable_interrupts(INT_TIMER1);
		enable_interrupts(INT_COMP);
		enable_interrupts(INT_CCP1);
		enable_interrupts(INT_RDA);
	}		
	set_timer0(0);
	clear_interrupt(INT_EXT2);	
}
*/
