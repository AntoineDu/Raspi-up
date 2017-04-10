//************************************************************************************
//**
//**
//**
//**
//************************************************************************************

#include <18F4550.h>

#fuses HS,NOWDT,NOPROTECT,NOLVP,NODEBUG,USBDIV,VREGEN

#use delay(clock=24M)
#use rs232(baud=19200, xmit=PIN_C6, rcv=PIN_C7, stream=RS232)
#byte INTCON2 = 0xFF1
#bit  RBPU = INTCON2.7
#include "..\include\LCD420_S3.c"

// Variables :

const int16 PosCharge = 4000, PosZero = 48000, PosUn = 49000, PosDeux = 50000, PosTrois = 51000, EcartRxInf = 100, EcartRxSup = 1100;

const int8 BUFFER_SIZE = 16;

int16 Timer0_Rx = 0, TimerTx[20];

int8 DeuxBitsTx = 0, N_AlternanceTx = 0, N_AlternanceTxMax = 16, N_AlternanceRx = 0, N_AlternanceRxPrec = 0, N_AlternanceRxMax = 16, VarBoucleTx = 0;

int8 buffer[BUFFER_SIZE], next_in = 0, next_out = 0, SerialChar[5], N_Alternance = 1, MessageTx[5], N_MessageTx = 1, DeuxBitsRx = 0, N_DeuxBitsRx = 0, MessageRx[5], N_MessageRx = 1, AdresseModule = 228;

int1 FlagBouton = 0, FlagRx = 0, FlagRxAlternance = 0, StartTx = 0, TxOn = 0, RxOn = 0, Flag_Positif = 0, Flag_WtD_P = 0, Flag_Triac = 0;

#define bkbhit (next_in != next_out)

void initialisation()  // Routine d'initialisation
{
    set_tris_a(0b00000001);
    output_a(0);
    set_tris_b(255);
    RBPU = 0;   // Active le pull up sur les IO du port B mis en entrée
    set_tris_d(0b00000000);
    output_d(0);    // Tous les pins à 0
    setup_ccp1(CCP_CAPTURE_RE);     // Configure CCP1 sur flanc montant
    setup_comparator(A0_VR_A1_VR);      // Active les comparateurs avec référence interne
    setup_vref(VREF_LOW | 6); // Règle la référence à 5V / 4
    setup_timer_0(RTCC_DIV_1|RTCC_INTERNAL);    // Timer 0 clock interne et prédiviseur de 1
    setup_timer_1(T1_DIV_BY_1|T1_INTERNAL);     // Timer 1 clock interne et prédiviseur de 1
    enable_interrupts(INT_CCP1);    // Active l'interruption sur CCP1
    enable_interrupts(INT_COMP);    // Active l'interruption sur le comparateur
    clear_interrupt(INT_RDA);
    enable_interrupts(INT_RDA);    // Active l'interruption sur le port série
    enable_interrupts(GLOBAL);    // Active l'interruption globale
    AdresseModule = read_eeprom(1);
    Flag_Triac = read_eeprom(2);
    if(Flag_Triac) output_high(PIN_E1);
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
    if(!TxOn && !StartTx)
    {
        N_MessageTx = 1;
        MessageTx[1] = 150;
        MessageTx[2] = AdresseRecepteur;
        MessageTx[3] = AdresseModule;
        MessageTx[4] = Commande;

        for(VarBoucleTx = 1 ; VarBoucleTx <= N_AlternanceTxMax ; VarBoucleTx++)
        {
            if(VarBoucleTx == 1 || VarBoucleTx == 5 || VarBoucleTx == 9 || VarBoucleTx == 13) DeuxBitsTx = MessageTx[N_MessageTx] & 3;

            if(VarBoucleTx == 2 || VarBoucleTx == 6 || VarBoucleTx == 10 || VarBoucleTx == 14) DeuxBitsTx = MessageTx[N_MessageTx] >> 2 & 3;

            if(VarBoucleTx == 3 || VarBoucleTx == 7 || VarBoucleTx == 11 || VarBoucleTx == 15) DeuxBitsTx = MessageTx[N_MessageTx] >> 4 & 3;

            if(VarBoucleTx == 4 || VarBoucleTx == 8 || VarBoucleTx == 12 || VarBoucleTx == 16)
            {
                DeuxBitsTx = MessageTx[N_MessageTx] >> 6 & 3;
                N_MessageTx = N_MessageTx + 1;
            }

            switch(DeuxBitsTx)
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
            }
            if(PosUn + EcartRxInf < Timer0_Rx && Timer0_Rx < PosUn + EcartRxSup) // Position 1 = 01
            {
                DeuxBitsRx = 1;
                FlagRxAlternance = 1;
            }
            if(PosDeux + EcartRxInf < Timer0_Rx && Timer0_Rx < PosDeux + EcartRxSup) // Position 2 = 10
            {
                DeuxBitsRx = 2;
                FlagRxAlternance = 1;
            }
            if(PosTrois + EcartRxInf < Timer0_Rx && Timer0_Rx < PosTrois + EcartRxSup) // Position 3 = 11
            {
                DeuxBitsRx = 3;
                FlagRxAlternance = 1;
            }

            // Si on a pas reçu de position valide on quitte :
            if(!FlagRxAlternance)
            {
                FlagRx = 0;
                output_low(PIN_D1);
                return;
            }

            if(!RxOn)
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
            }

            // On place les deux bits obtenus au bon endroit :
            if((N_AlternanceRxPrec + 1) == N_AlternanceRx) // Si on a pas raté d'impulsion
            {
                switch(N_DeuxBitsRx)
                {
                    case 1 : // Bits 0 et 1 :
                        MessageRx[N_MessageRx] = 0;
                        MessageRx[N_MessageRx] = MessageRx[N_MessageRx] + DeuxBitsRx;
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
                    Flag_WtD_P = 1;
                    RxOn = 0;
                }
                N_AlternanceRxPrec++;
            }
            else RxOn = 0; // On a raté une impulsion on redémarre la réception
        }
        FlagRx = 0;
        output_low(PIN_D1);
    }
}

void WhatToDoPowerline()    // Fonction qui décide de quoi faire en fonction de la commande reçue par courannt porteur
{
    if(Flag_WtD_P)
    {
        int8 AdresseSource = 0;
        if(MessageRx[2] == AdresseModule)
        {
            AdresseSource = MessageRx[3];

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
                    write_eeprom(2,1);
                break;

                case 21 : // Eteint le triac de puissance
                    output_low(PIN_E1);
                    Flag_Triac = 0;
                    write_eeprom(2,0);
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
            }
        }
        else
        {
            while(bkbhit)
            {
            bgetc();    // vidage buffer si non OK
            }
        }
    }    
}

void main()  ////////////////////////////////////////////////////////////////////////////////////// MAIN ///////////////////////////////////////////////////////////////////////////////////////////////////////
{
    initialisation();
    delay_ms(500);    
    while(TRUE)
    {
        Reception();

        WhatToDoPowerline();

        WhatToDoPSerial();

        if((N_Alternance == 10) && FlagBouton)
        {
            SetupTransmission(27,228); // 228 = 11 10 01 00  et 27 = 00 01 10 11
        }

        if(input_state(PIN_B6) == 0 || input_state(PIN_B4) == 0)
        {
            FlagBouton = !FlagBouton;
            delay_ms(250);
        }

        

    }
}   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#int_CCP1
void CCP1_isr() // Interruption sur passage par zéro
{
    if(get_timer0() > 59000)
    {
        set_timer0(0); // Remise à 0 du timer0
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
        if(N_Alternance == 51) N_Alternance = 1;
        Transmission(); // Appel de la fonction de transmission
        FlagRxAlternance = 0; // Comme on démarre une nouvelle alternance on peut recommencer à regarder pour une réception
        clear_interrupt(INT_CCP1); // Clear du flag d'interruption
    }
}

#int_COMP
void COMP_isr() // Interruption sur comparateur pour la réception
{
    if(!FlagRx && !C1OUT)
    {
    Timer0_Rx = get_timer0();
    FlagRx = 1;
    }
    clear_interrupt(INT_COMP);
}

#int_TIMER1 // Interruption sur timer pour l'envoi des impulsions
void TIMER1_isr()
{
    if(TxOn || StartTx)
    {
        output_high(PIN_A5);
        delay_us(100);
        output_low(PIN_A5);
        if(StartTx)
        {
            StartTx = 0;
            TxOn = 1;
        }
    }
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