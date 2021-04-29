/*
 * 18.09.2020
 * �������� ��������� �� ���������� ����
 * ���������� ������ ���������
 * ��������� ������ (������ ���)
 * ������ BT ����� Tuya
 * �������� ������ ������������
 */
#include <pic.h>
#include <proc\pic16f1829.h>
#include "fdataSB.h"

#define SMART

#define setbit(var, bit) ((var) |= (1 << (bit)))
#define clrbit(var, bit) ((var) &= ~(1 << (bit)))

//#pragma config WDTE = OFF, PWRTE = OFF, BOREN = OFF, MCLRE = ON, FCMEN = OFF, LVP = OFF, FOSC = INTOSC, STVREN = OFF
#pragma config WDTE = SWDTEN, PWRTE = OFF, BOREN = OFF, MCLRE = ON, FCMEN = OFF, LVP = OFF, FOSC = INTOSC, STVREN = OFF
//��������� �������
void Initial(void);
byte checkSum(byte *p, byte size);
byte moveMotor();

//------------------------------------------------------------------
//������ ����������� ����� 

byte checkSum(byte *p, byte size) {
    byte sum = 0;
    for (byte i = 0; i < size; i++) {
        sum += *p++;
    }
    return sum;
}
//------------------------------------------------------------------

byte moveMotor() {
    byte Error = 0;
    //������ ���������
    nHalfTurn = 0;
    
    //����������� ����� ���
    CHS0 = 1; CHS1 = 1; CHS2 = 0;
    ADON = 1;
    
    LED_CLOSE = 0;            
    LED_OPEN = 0;      
    
    byte bTemp = flag.b.direct;
    bTemp ^= flag.b.inverMov;
    bTemp &= 0b1;
    if(bTemp){
        setbit(PORTC, bIN1);
        clrbit(PORTC, bIN2);
    }else{
        setbit(PORTC, bIN2);
        clrbit(PORTC, bIN1);
    }
    swMove = ON; //�������� ������� � ��������� ������
    
    flag.b.motorMove = 1; //������ ����� �������
    stat = stDevTurn; //������ ������ ���
    numHighCurrent = 0;
    nWait = 18;//15;
    
    TMR2IE = 1;
    
    IOCAF5 = 0;
    IOCIF = 0;
    IOCIE = 1;

    while (1) {
        if (!flag.b.motorMove) {

            if (stat == stRevers) {//��������� ��� �������� ����
                if (flag.b.currBig) {//� ����������� ����
                    Error = errRevers; //������� ��� ��������
                } else {
                    if(bTemp){
                        setbit(PORTC, bIN1);
                        clrbit(PORTC, bIN2);
                    }else{
                        setbit(PORTC, bIN2);
                        clrbit(PORTC, bIN1);
                    }//�������� ������ �������� ������������                                       
                }
                //MPauseStop;
                break;
            }

            if (stat == stDevTurn) {
                //��������� ��� ���������� �������, ����� ���������� ����� ������������� �����
                if (flag.b.currBig) {//� ����������� ����
                    if (flag.b.reperPos) {//������� �� ������
                        if (nHalfTurn < Turn) {//������������� ���������� ������������
                            Error = errHalf1; //������� �� ������, �� �������� ������ ���������� ������������    

                        } else {//����������� ���������� ������������
                            Error = errHalf2; //������� �� ������, ��������� ������ ���������� ������������                              
                        }
                        break;
                    } else {//������� �� �� ������                               
                        if (nHalfTurn < Turn) {//������������� ���������� ������������
                            Error = errHalf3; //������� �� �� ������, �� ��������� ������ ���������� ������������                                      
                            
                            //break; //������� �� �����
                        }
                        swMove = OFF;
                        //������ ����������� �������� ������;                        
                        if(bTemp){//������
                            setbit(PORTC, bIN2);
                            clrbit(PORTC, bIN1);
                        }else{
                            setbit(PORTC, bIN1);
                            clrbit(PORTC, bIN2);
                        }
                        
                        flag.b.motorMove = 1; //��������� �����; 
                        nHalfTurn = 0;
                        nWait = 5;
                        stat = stRevers;
                        swMove = ON;
                    }
                    flag.b.currBig = 0;
                } else {//���������� �� ���� ���, �������� ���������� ������
                    //��������
                    if(bTemp){//������
                        setbit(PORTC, bIN2);
                        clrbit(PORTC, bIN1);
                    }else{
                        setbit(PORTC, bIN1);
                        clrbit(PORTC, bIN2);
                    }                   
                    break;
                }
            }
        }
        //���-�� �����������, ���������� �����������
        //swMove = ON; //�������� ������� � ��������� ������
    }
    
    TMR2IE = 0;
    TMR2 = 0;
    nWait = 2;
    TMR2IF = 0; 
    TMR2IE = 1;
    while (nWait);
    IN1 = 0;IN2 = 0;
    LED_OPEN = 0;
    LED_CLOSE = 0;
//sleep    IOCIE = 0;
    swMove = OFF; //��������� ������� � ��������� ������
    TMR2IE = 0;
    ADON = 0;
    return Error;
}
//------------------------------------------------------------------
//������� �������������

void Initial(void) {
    SCS1 = 1;
    SPLLEN = 1;

    IRCF3 = 1;
    IRCF2 = 1;
    IRCF1 = 0;
    IRCF0 = 1; //4 MHz    
/*
    byte bTemp;    
    byte numByte;
   numByte = 7;
        do{
            //����� ��� ������ ����            
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            //NOP();NOP();
            RA1 ^= 1;
            bTemp >>= 1;
            if(RB5)
                bTemp |= 0x80;
            else
                bTemp |= 0x00;
            
            //RA1 = 0;   
            NOP();                                            
        }while(numByte--);
        */
        
    LED_OPEN_DIR = OUTPUT;
    LED_CLOSE_DIR = OUTPUT;
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
//    TRISA0 = OUTPUT;
    TRISA1 = OUTPUT;
    TRISA0 = INPUT;
//    RA0 = 0;
    RA1 = 0;RA1 = 1;RA1 = 0;RA1 = 1;RA1 = 0;RA1 = 1;RA1 = 0;RA1 = 1;RA1 = 0;
    
    LATCbits.LATC5 = 0;
    IN1_DIR = OUTPUT;

    LATCbits.LATC4 = 0;
    IN2_DIR = OUTPUT;

    LATCbits.LATC7 = 0;
    swMove_DIR = OUTPUT;
    SSP1CON1bits.SSPEN = 0;
    ANSELCbits.ANSC7 = 0;
    sound_DIR = OUTPUT;
    sound = OFF;
    onBAT_DIR = OUTPUT;
    onBAT = OFF;

    sensOpto_DIR = INPUT;
    IOCAN5 = 1; //��������� ��������� �� ������ RA5, ������� �� 1 � 0    
    INLVLA5 = 0; //TTL   ������� ������
    
    IOCBN5 = 1; //��������� ��������� �� ������ RB5, ������� �� 1 � 0
    INLVLB5 = 0;
    
    IOCIE = 1;

    swConf_DIR = INPUT;
    OPTION_REGbits.nWPUEN = 0; //��������� ������ ������������� ����������       

    
    ANSELAbits.ANSA2 = 0;
    ANSELCbits.ANSC2 = 0;
    LATAbits.LATA2 = 0;
    LATCbits.LATC2 = 0;
    
    //CPSRNG0 = 1;CPSRNG1 = 1;//������������ ��� 18 uA    
    CPSRNG0 = 0; CPSRNG1 = 1; //������������ ��� 1.2 uA    
    //CPSRNG0 = 1; CPSRNG1 = 0;
    WPUC0 = 0; //
    WPUC1 = 0;
    
    numCh = 0;
    CPSCON1 = setCh[numCh];

    TMR0CS = 0; //������� Fosc/4
    PSA = 0;
    PS0 = 1; PS1 = 1; PS2 = 0;
    //PS0 = 0; PS1 = 0; PS2 = 1;
    TMR1CS0 = 1;
    TMR1CS1 = 1; //11 =Timer1 clock source is Capacitive Sensing Oscillator (CAPOSC)
    TMR1ON = 1;

    T1GPOL = 1;
    T1GSS1 = 0;
    T1GSS0 = 0;
    //sleep T1GSS1 = 0;
    //sleep T1GSS0 = 1;   
    nT1SYNC = 1;
    TMR1GE = 0;
    //sleep T1GSPM = 1;
    //sleep T1GTM = 1;
    // sleep TMR1GE = 1;
    // sleep TMR1GIE = 1;
    WDTCON &= 0b11000001;
    //WDTCON |= 0b00001010;//32ms
    WDTCON |= 0b00001000;//00100 = 1:512 (Interval 16 ms nominal)
    //WDTCON |= 0b00001110;//00111 = 1:4096 (Interval 128 ms nominal)

    T2CON = 0x07; //��� �� 64//0x7F;//�������� ������ � �������� �� 64 � �� 16
    //sleep TMR2IE = 1;

    //UART    
    TRISB5 = INPUT;
    TRISB7 = OUTPUT;
    ANSELBbits.ANSB5 = 0;
    LATBbits.LATB5 = 0;
    APFCON0bits.RXDTSEL = 0; //RB5 - RX
    APFCON0bits.TXCKSEL = 0; //RB7 - TX
    SYNC = 0;
    
    SPEN = 1;

    BRGH = 1;
    BRG16 = 0;    
    TXEN = 1;
    //SPBRGH = 0;
    //SPBRGL = 12; //19200 bit/s
    SPBRGH = 0;
    SPBRGL = 25; //9600 bit/s

    //RCIF = 0;
    //***RCIE = 1;

    //������������ ���
    ADCON0 = 0;
    ADON = 1;
    ADFM = 1; //�������� ������   
    ADCON1bits.ADCS = 0b100; //Fosc/4
    //RA4 pin3 AN3 ����� ������� ���� ��� ������
    FVREN = 1;//��� ������� ����������
    ADFVR0 = 0;ADFVR1 = 1;//2.048 V
    ADPREF0 = 1;ADPREF0 = 1;//���������� ������� ����������
    ADIE = 1;
    
    WPUA4 = 0; 
    //R�3 pin7 AN7 ����� ��������� ������ ������������
    WPUC3 = 0;
    INLVLC3 = 1;

    //T0IE = 1;     
    SWDTEN = 0; ;
    
    PEIE = 1;
    GIE = 1;
}
//------------------------------------------------------------------
//������� ��������� ���������� 

__interrupt(high_priority) void Inter(void) {
    unsigned int wTemp;    
    byte *p;       
    byte numByte;
    byte bTemp;
   
    SWDTEN = 0; 
    //��� ���� ������
 /*   if (TMR1GIF && TMR1GIE) {           
        TMR1GIF = 0;
        T1GCONbits.T1GGO = 1;
        
        wTemp = (unsigned int)(TMR1H << 8) + TMR1L;
        dl[numCh] = wTemp;
        fDl[numCh] = 1;

        CPSCON1 = setCh[numCh]; //������������� �����
               
        if (numCh) {
            numCh = 0;
        } else {
            numCh = 1;
        }
        TMR1L = 0;
        TMR1H = 0;       
    }
*/
    if (ADIE && ADIF) {//����� ���
        p = (byte *)&wTemp;
        //wValADC.b[1] = ADRESH;
        //wValADC.b[0] = ADRESL;
        *p++ = ADRESL;
        *p = ADRESH;
        if(detect.b.checkBattery){
            valuePowerADC = wTemp;//wValADC.num;
        }else{
            //if (wValADC.num > CONSTPOROG) {//������ � ������� ��������� � ������� ��������� ���������            
            if (wTemp > CONSTPOROG) {//������ � ������� ��������� � ������� ��������� ���������            
                numHighCurrent++;            
            } else {
                if (numHighCurrent)
                    numHighCurrent--;
            }
            if (numHighCurrent == CONSTBIG) {//�������� ���������� - ������������� �����
             //   IN1 = 0;IN2 = 0;
                flag.b.motorMove = 0;
                flag.b.currBig = 1;
                numHighCurrent = 0;
            }
        }
        ADIF = 0;        
    }

    if (TMR2IE && TMR2IF) {
        
        if (nWait) {
            nWait--;            
        }else{//�� ���������� ��������� ����� ��������� ����� ��� 
            
            if(detect.b.checkBattery)// �������� ����� �������
                if(!ADCON0bits.GO){
                    ADCON0bits.GO = 1;                     
                }
            if(flag.b.motorMove){//��������� �������� 
                
                if(!ADCON0bits.GO)
                    ADCON0bits.GO = 1;
                //������ �������
                nWaitS++;            
                if(nWaitS & 0b10000){
                    if(flag.b.direct){                                        
                        setbit(PORTC, bLED_OPEN);
                    }else{                    
                        setbit(PORTA, bLED_CLOSE);
                    }
                }else{
                    clrbit(PORTC, bLED_OPEN);
                    clrbit(PORTA, bLED_CLOSE);                
                }                
            }                
        }           
     
        if (timeTactRead) {
            timeTactRead--;
        } else//����� ����� �� ����� ������
            if (!detect.b.readOk) {//����� ��� �� ������
                numByteRX = 0; //����� ��������� ����� �����
                detect.b.recData = 0;//����� ������ ��������
                IOCBN5 = 1;//��������� ���������� UART �� sleep
            }
     
        //if((!timePower++) && (!flag.b.swOn))
        //    detect.b.checkBattery = 1;//��������� �������� ������ ������������
   //     RA1 ^= 1;
        TMR2IF = 0;
    }
    if(IOCIE){
    //���������� �� ����������� ������
    if(IOCAF5) {
        if (!nWait) {
            nHalfTurn++;
            
            if (  ((nHalfTurn == 1) && (stat == stRevers)) || //������������� ����� ��� ������� �� ������
                    //         ((nHalfTurn >  Turn) && flag.b.direct  ) ||//����������� �������
                    //          ((nHalfTurn == Turn) && (!flag.b.direct)) //����������� �������
                    (nHalfTurn > Turn)
                    ) {
                //IN1 = 0;IN2 = 0;
                flag.b.motorMove = 0;
            }
            TMR2 = 0;
            nWait = 5;
            TMR2IE = 1;
        }
        IOCAF5 = 0;
        IOCIF = 0;
    }
    if(IOCBF5){
    //   RA1 = 1;
        TMR2IE = 1;                        
  //      RA1 = 0;
        numByte = 7;
        do{
            //����� ��� ������ ����            
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            NOP();NOP();
            //RA1 ^= 1;
            bTemp >>= 1;
            if(RB5)
                bTemp |= 0x80;
            else
                bTemp |= 0x00;            
            //RA1 = 0;   
            NOP();                                            
        }while(numByte--);
        IOCBF5 = 0; 
 /*   }
    }

    if (RCIE && RCIF) {
        if (RCSTAbits.OERR) {// || detect.b.readOk) {//������ ������������ ��� �������� ����� ��� �� �������
            CREN = 0;
            NOP();
            CREN = 1;
        } else {//������ ����
  * */
     /*  
        byte mask = 0x80;
        do{                        
            if(bTemp & mask){
                RA1 = 1;
            }else{
                RA1 = 0;
            }
            mask >>= 1;
        }while(mask);
        RA1 = 0;*/
        
   //***         arrToRX[numByteRX] = RCREG;   
            arrToRX[numByteRX++] = bTemp;
            //numByteRX++;
            switch (numByteRX) {
                case 1:
                {                    
                    if (arrToRX[0] != 0x55) {//��� �� ������ ����
                        numByteRX = 0;
                    } else{
                        timeTactRead = 10; //������������ ����� �� ����� ����� ������
                        detect.b.recData = 1;//���� ����� ������
                    }
                    break;
                }
                case 2:
                {
                    if (arrToRX[1] != 0xAA)//��� �� ������ ����
                        numByteRX = 0;
                    break;
                }
                    /*case 3://������� ������
                    case 4://�������
                    case 5:{//������� ���� � ���������� ����                                        
                        break;                    
                    }*/
                case 6:
                {//6 + 1                    
                    allByteRX = 7 + arrToRX[5]; //����� ���������� ���� � ������
                    timeTactRead += arrToRX[5] + 5;
                    break;
                }
                default:
                {
                    if (numByteRX == allByteRX) {//����� ������ ���������
                        detect.b.readOk = 1;//������ ���� �������������
                        detect.b.recData = 0;//������ �2 ����� ���������                        
                        numByteRX = 0;
                    }
                }
            }
 //**       }   
      //  if(detect.b.recData)//���� ����� ������
        //    TMR2IE = 1;
    }
    }

    if (TXIE && TXIF) {
        TXREG = *(arrToTX + numByteTX++);
        if (numByteTX == allByteTX)//�������� ��� �����
            TXIE = 0;
    }

    return;
}
//------------------------------------------------------------------

void main(void) {
    unsigned char i;

    //�������� ������� �����
    union uLock{
        struct sLock{
            byte levelPower;//������� ������ ������������
            byte directMotor;//����������� �������� ��������� ��� ��������
        }prop;
        byte all[2];
    } myLock;
    
//������� ������    
#define lHeight 0x00
#define lMedium 0x01
#define lLow    0x02
#define lZero   0x03
    
    for(i = 0; i < 2; i++)
        myLock.all[i] = 0;
    
    byte bTemp;

#define CONSTPOR   63// 3    
#define ALLNUMPARAM 7//���������� ���������� + 1 ������� ���������� �� ��������
#define CONSTSIGNALON 80//40
    
    struct sSensorSW{
        unsigned int sampl;//������� ����������� ��������
        unsigned int level;//���������� �������        
    } sensSW[2];
#define CONSTAKK    5     
    unsigned int filterS[CONSTAKK][2];//��� ������� ��
    unsigned int filterL[CONSTAKK * 2][2];
    
    unsigned char iSampl, idUser, iLevel;   
    
    unsigned int wTemp;    
    union{
        unsigned int w;    
        byte b[2];
    }wADC; 
    
    //�������� �������
    byte commandForMotor;
    #define cNoComm        0x00    
    #define cBLEOpen       0x01
    #define cSensSWOpen    0x02
    #define cBLEClose       0x03
    #define cSensSWClose    0x04    

    Initial();

    nWait = 0;    
    arrToTX[0] = 0x55;
    arrToTX[1] = 0xAA;
    arrToTX[2] = 0x00; //������
    arrToTX[4] = 0x00; //������� ���� ��������� "���������� ����"

    //swMove = ON;
    
    Turn = 7; //���� �� �������
    CPSON = 1;
    LED_OPEN = 1;
    LED_CLOSE = 0;
    
    
    cicleGo = 0;
    flag.all = 0;
    numByteRX = 0;
    detect.all = 0;
    cicleGo = 10;
    do{
        wTemp = 40000;
        do{NOP();}while(wTemp--);
        LED_OPEN ^= 1;
        LED_CLOSE ^= 1;        
    }while(cicleGo--);
    
    LED_OPEN = 0;
    LED_CLOSE = 0;
    
    detect.b.firstOn = 1;
    detect.b.sensSWzero = 1;
    TMR2IE = 0;
    while (1) {

        if (detect.b.readOk) {//���� ���� �������� �����, �� ����������� ���
            //�� ����� �� ��������� ����������� ����� � ��������� ������

            while (TXIE); //���� ����� ���������� ���������� ��������

            switch (arrToRX[3]) {//���� � 3 - �������  
                case 0x00:
                {//�����
                    arrToTX[3] = 0x00;
                    arrToTX[5] = 0x01;
                    if (detect.b.firstOn) {//������ ���������
                        arrToTX[6] = 0x00;
                        arrToTX[7] = 0x00;
                        detect.b.firstOn = 0;
                    } else {
                        arrToTX[6] = 0x01;
                        arrToTX[7] = 0x01;
                    }
                    allByteTX = 8;
                    break;
                }
                case 0xE8:
                {//������ ������ �����������
                    arrToTX[3] = 0xE8;
                    arrToTX[5] = 0x06;
                    //������ 100100
                    arrToTX[6] = 0x01;
                    arrToTX[7] = 0x00;
                    arrToTX[8] = 0x00;
                    arrToTX[9] = 0x01;
                    arrToTX[10] = 0x00;
                    arrToTX[11] = 0x00;
                    arrToTX[12] = checkSum(arrToTX, 12);
                    allByteTX = 13;
                    break;
                }
                case 0x01:
                {//������ ���� ���������
                    arrToTX[3] = 0x01;
                    arrToTX[5] = 0x08;
                    //� �������� ASCII (WINDOWS-1251) isnwhrlh = 69 73 6E 77 68 72 6C 68
                    arrToTX[6] = 0x69;
                    arrToTX[7] = 0x73;
                    arrToTX[8] = 0x6E;
                    arrToTX[9] = 0x77;
                    arrToTX[10] = 0x68;
                    arrToTX[11] = 0x72;
                    arrToTX[12] = 0x6C;
                    arrToTX[13] = 0x68;
                    arrToTX[14] = checkSum(arrToTX, 14);
                    allByteTX = 15;
                    break;
                }
                case 0x02:
                {//������ �� ����������� ��������� ������ � ��������� ������ � ������
                    arrToTX[3] = 0x02;
                    arrToTX[5] = 0x00;
                    arrToTX[6] = 0x01;
                    allByteTX = 7;
                    break;
                }
                case 0x03:
                {//�������� ������� ������ ������
                    //arrToRX[6]//������� ������� (0�00 - �� ��������; 0�01 - �������� �� ���������; 0�02 - �������� ���������)
                    arrToTX[3] = 0x03;
                    arrToTX[5] = 0x00;
                    arrToTX[6] = 0x02;
                    allByteTX = 7;
                    break;
                }
                case 0x08:
                {//������ �������-�������� ����������� ��� �������� �� �� ��������
                    //�������� ��������� �������� � ������ ���������, ��������� ����� ����������
                    //������ ����� ������������� �������� ������ ����������� ������
                    arrToTX[3] = 0x07; //����� �� ������ ������� - �������� 
                                arrToTX[5] = 0x08;
                                arrToTX[6] = 0x08; //id ����� ������������ � ���� ��������
                                arrToTX[7] = 0x02; //��� ������ ��������
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x04; //���������� - 4 ����
                                arrToTX[10] = 0; 
                                arrToTX[11] = 0; 
                                arrToTX[12] = 0; 
                                arrToTX[13] = 17; 
                                arrToTX[14] = checkSum(arrToTX, 14);
                                allByteTX = 15;
                    numParam = ALLNUMPARAM;
                    break;
                }
                case 0x07:
                {//������������� ������ ������-������
                    if (numParam)//���� ���� �� ���������� ���������
                    {
                        switch (numParam) {
                            case 1:
                            {//�������� �������� �� ������������� ����� ����������
                                arrToTX[3] = 0x07; //����� �� ������ ������� - �������� 
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x1C; //id ���� ����� 28
                                arrToTX[7] = 0x04; //������������� ��� ������
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //���������� - ���� ����
                                arrToTX[10] = 0x07; //������� ����                                
                                arrToTX[11] = checkSum(arrToTX, 11);
                                allByteTX = 12;
                                break;
                            }
                            case 2:
                            {//��������� �����
                                arrToTX[3] = 0x07;
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x2F; //47 dp id ������ �����
                                arrToTX[7] = 0x01; //��� ������ boolean
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //���� ����� ������
                                arrToTX[10] = flag.b.direct; //��������� ����� ����� ������������ ������������                                                      
                                arrToTX[11] = checkSum(arrToTX, 11);
                                allByteTX = 12;
                                break;
                            }
                            case 3:
                            {//��������� �����
                                arrToTX[3] = 0x07;
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x30; //48 dp ����������� �������� ����������
                                arrToTX[7] = 0x04; //��� ������ 
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //���� ����� ������
                                arrToTX[10] = flag.b.inverMov; //��������� ����� ����� ������������ ������������                                                      
                                arrToTX[11] = checkSum(arrToTX, 11);
                                allByteTX = 12;
                                break;
                            }
                            case 4:
                            {
                                arrToTX[3] = 0x07; //����� �� ������ ������� - �������� 
                    arrToTX[5] = 0x05;
                    arrToTX[6] = 0x09; //id ����� ������������ � ���� ��������
                    arrToTX[7] = 0x04; //������������� ��� ������
                    arrToTX[8] = 0x00;
                    arrToTX[9] = 0x01; //���������� - ���� ����
                    arrToTX[10] = 2;//myLock.prop.levelPower; //�������� 0x00: ������� ������� ������ �������, 
                    //0x01: ������� �������, 0x02: ������ ������� ������ �������, 0x03: ������� ���������
                    arrToTX[11] = checkSum(arrToTX, 11);
                    allByteTX = 12;
                                
                                break;
                            }
                            default:
                            {
                            }
                        }
                        numParam--;
                    } else
                        allByteTX = 0;
                    break;
                }
                case 0x06:
                {//������� ��� �����
                    arrToTX[3] = 0x07; //������� ������������� ������                    
                    switch (arrToRX[6]) {
                        case 0x06:
                        {//������� ������� ����� �� Bluetooth, �� ��� �������� ����������� ������-�� ������ pdid46

                            arrToTX[5] = 0x06;
                            arrToTX[6] = 0x06;
                            arrToTX[7] = 0x00; //��� ������ raw
                            arrToTX[8] = 0x00;
                            arrToTX[9] = 0x02; //���������� - ��� �����
                            commandForMotor = cBLEOpen;
                            
                            flag.b.swOn = 1;
                            flag.b.direct = 1; //arrToRX[10];//0�00 - ������� �����, 0�01 - ������� ����� 
                            detect.b.checkBattery = 0;
                            
                            arrToTX[10] = flag.b.direct;
                            idUser = arrToRX[11]; //������������� ������������
                            arrToTX[11] = idUser;
                            arrToTX[12] = checkSum(arrToTX, 12);
                            allByteTX = 13;
                            break;
                        }
                        case 0x2E:
                        {//������� ������� ����� � ������ �� Bluetooth pdid46                            
                            arrToTX[5] = 0x05;
                            arrToTX[6] = 0x2E;
                            arrToTX[7] = 0x01; //��� ������ boolean
                            arrToTX[8] = 0x00;
                            arrToTX[9] = 0x01; //���������� - ���� ����                            
                            arrToTX[10] = 0x01; //������� ������� �������
                            commandForMotor = cBLEClose;
                            flag.b.swOn = 1;
                            flag.b.direct = 0; //0 - ������� �����
                            detect.b.checkBattery = 0;
                            
                            arrToTX[11] = checkSum(arrToTX, 11);
                            allByteTX = 12;
                            break;
                        }
                        case 0x30:
                        {//������� ����������� �������� ����������                            
                            arrToTX[5] = 0x05;
                            arrToTX[6] = 0x30;
                            arrToTX[7] = 0x04; //��� ������ raw
                            arrToTX[8] = 0x00;
                            arrToTX[9] = 0x01; //���������� - ���� ����                                                        
                            flag.b.inverMov = arrToRX[10];
                            arrToTX[10] = flag.b.inverMov;
                            arrToTX[11] = checkSum(arrToTX, 11);
                            allByteTX = 12;
                            break;
                        }
                        case 0x1C:
                        {//���������� ����
                            break;
                        }
                        default:
                        {
                        }
                    }
                    break;
                }
            }
            if (allByteTX) {//���� ����� ��������� ��������
                numByteTX = 0;
                TXIE = 1; //��������� ��������
            }                       
            detect.b.readOk = 0;
        }

        if (flag.b.swOn) {//������ ������� ������� ���������            
            CPSON = 0; //��������� ��������� ������
            //TMR1ON = 0;           
            //����� ������� ��� �����������
            //        if(moveMotor()){//�������� ������
            //            flag.b.direct ^= 1;//��� � ��������
            //RCIE = 0;RCIF = 0;
            
            moveMotor();
            
//***            RCIE = 1;
            //        }else{
//sleep TMR1ON = 1;
            //���������� ������� ��������� ������� �����
           while (TXIE)
               NOP();
            
            /*arrToTX[3] = 0x07;
            arrToTX[5] = 0x05;
            arrToTX[6] = 0x2F; //47 dp id ������ �����
            arrToTX[7] = 0x01; //��� ������ boolean
            arrToTX[8] = 0x00;
            arrToTX[9] = 0x01; //���� ����� ������
            arrToTX[10] = flag.b.direct; //��������� ����� ����� ������������ ������������                                                      
            arrToTX[11] = checkSum(arrToTX, 11);
            allByteTX = 12;*/
            arrToTX[3] = 0xE0; //������� ������ �� ����
            arrToTX[5] = 0x06; //���������� ���� ������
            arrToTX[6] = 0x01; //����� ������������ ������            
            arrToTX[7] = 0x2F; //47 dp id ������ �����
            arrToTX[8] = 0x01; //��� ������ boolean
            arrToTX[9] = 0x00;
            arrToTX[10] = 0x01; //���� ����� ������
            arrToTX[11] = flag.b.direct; //��������� ����� ����� ������������ ������������                                                      
            arrToTX[12] = checkSum(arrToTX, 12);
            allByteTX = 13;
            numByteTX = 0;
            TXIE = 1; //��������� ��������

            switch (commandForMotor){
                case cBLEOpen :{//����� ������ ��  Bluetooth            
                    //���������� ������� �������� � ������ ������� "����� ������"
                    while (TXIE);
                    arrToTX[3] = 0xE0; //������� ������ �� ����
                    arrToTX[5] = 0x09; //���������� ���� ������
                    arrToTX[6] = 0x01; //����� ������������ ������
                    arrToTX[7] = 0x13; //������� Ble unlock record
                    arrToTX[8] = 0x02; //��� ������ value
                    arrToTX[9] = 0x00;
                    arrToTX[10] = 0x04; //���������� - 4 �����
                    arrToTX[11] = 0;
                    arrToTX[12] = 0;
                    arrToTX[13] = 0;
                    arrToTX[14] = idUser;
                    arrToTX[15] = checkSum(arrToTX, 15);
                    allByteTX = 16;
                    numByteTX = 0;
                    TXIE = 1; //��������� ��������
                    break;
                }
            case cSensSWOpen :{//����� ������ � ������� ��������� ���������          
                    //���������� ������� �������� � ������ �������������� "����� ������"
                    while (TXIE);
                    
                    arrToTX[3] = 0xE0; //������� ������ �� ����
                    arrToTX[5] = 0x06; //���������� ���� ������
                    arrToTX[6] = 0x01; //����� ������������ ������
                    arrToTX[7] = 0x15; //������� lock alarm
                    arrToTX[8] = 0x04; //��� ������ enum
                    arrToTX[9] = 0x00;
                    arrToTX[10] = 0x01; //���������� - 1 �����
                    arrToTX[11] = 0x09;//������� ��������� �������
                    arrToTX[12] = checkSum(arrToTX, 12);
                    allByteTX = 13;
                    numByteTX = 0;
                    TXIE = 1; //��������� ��������
                    break;
                }
            case cSensSWClose :{//����� ������� � ������� ��������� ����������             
                    //���������� ������� �������� � ������ �������������� "����� ������"
                    while (TXIE);
                    
                    arrToTX[3] = 0xE0; //������� ������ �� ����
                    arrToTX[5] = 0x06; //���������� ���� ������
                    arrToTX[6] = 0x01; //����� ������������ ������
                    arrToTX[7] = 0x15; //������� lock alarm
                    arrToTX[8] = 0x04; //��� ������ enum
                    arrToTX[9] = 0x00;
                    arrToTX[10] = 0x01; //���������� - 1 �����
                    arrToTX[11] = 0x0A;//������� ��������� �������
                    arrToTX[12] = checkSum(arrToTX, 12);
                    allByteTX = 13;
                    numByteTX = 0;
                    TXIE = 1; //��������� ��������
                    break;
                }
            } 
            while (TXIE);
            arrToTX[3] = 0x07;
            arrToTX[5] = 0x05;
            arrToTX[6] = 0x2F; //47 dp id ������ �����
            arrToTX[7] = 0x01; //��� ������ boolean
            arrToTX[8] = 0x00;
            arrToTX[9] = 0x01; //���� ����� ������
            arrToTX[10] = flag.b.direct; //��������� ����� ����� ������������ ������������                                                      
            arrToTX[11] = checkSum(arrToTX, 11);
            allByteTX = 12;            
            numByteTX = 0;
            TXIE = 1; //��������� ��������
            
            flag.b.swOn = 0;
            detect.b.checkBattery = 1;                        
            commandForMotor = cNoComm;
        }       
        
        //��� ����� ������������
        if(detect.b.checkBattery){
  //          CPSON = 0;//��������� ��������� ������           
            ADIF = 0;
            ADON = 1;
            //����������� ����� ��� AN7
            CHS0 = 1; CHS1 = 1; CHS2 = 1;            
            //���������� ��������
            onBAT = ON; 
            nWait = 4;
            valuePowerADC = 0;            
            TMR2IE = 1;
            
            while(!valuePowerADC);
            
            valuePowerADC = 0;
            while(!valuePowerADC);//�������� ��� ���� �� ������ ������
            wADC.w = valuePowerADC;
            
            if(wADC.w > CONSTHEIGHT){
                bTemp = lHeight;
            } else {
                if(wADC.w > CONSTMEDIUM){
                    bTemp = lMedium;
                } else{
                    if(wADC.w > CONSTLOW){
                        bTemp = lLow;
                    } else{
                        bTemp = lZero;
                    }
                }
            }
            ADON = 0;
           
            //if (bTemp != myLock.prop.levelPower)
            {            
                myLock.prop.levelPower = bTemp;//������ ������ �������, �������� ���������
                while (TXIE); //���� ����� ���������� ���������� ��������
                //55 AA 00 E0 00 06 01 09 04 00 01 01 F5
                arrToTX[3] = 0xE0; //����� �� ������ ������� - �������� 
                arrToTX[5] = 0x06;
                arrToTX[6] = 0x01;
                arrToTX[7] = 0x09; //id ����� ������������ � ���� ��������
                arrToTX[8] = 0x04; //������������� ��� ������
                arrToTX[9] = 0x00;
                arrToTX[10] = 0x01; //���������� - ���� ����
                arrToTX[11] = myLock.prop.levelPower; 
                arrToTX[12] = checkSum(arrToTX, 12);
                allByteTX = 13; 
                numByteTX = 0;
                TXIE = 1;                       
            } 
            //��������� ��������
            onBAT = OFF;            
            CPSON = 1;//�������� ��������� ������   
            detect.b.checkBattery = 0;         
            nWait = 10;
            detect.b.sensSWzero = 1;             
            LED_OPEN = 0;
            LED_CLOSE = 0;
            
            while(nWait);//���� ���������� �����
            TMR2IE = 0;
        }        
        
        if(detect.b.sensSWzero){
            for(i = 0; i < CONSTAKK; i++){
                filterS[i][0] = 0;
                filterS[i][1] = 0;
            }
            bTemp = 2 * CONSTAKK;
            for(i = 0; i < bTemp; i++){
                filterL[i][0] = 0;
                filterL[i][1] = 0;
            }
                
            sensSW[0].sampl = 0;
            sensSW[1].sampl = 0;            
            sensSW[0].level = 0;
            sensSW[1].level = 0;
            detect.b.sensSWzero = 0;
        }
        //������ ��������� ������
        for (i = 0; i < 2; i++) {            
            if (fDl[i] == 0) {
                continue;
            }
            fDl[i] = 0;
            wTemp = dl[i];            
                         
            //�� ����������  
            if(i == 1){
                bTemp = CONSTAKK - 1;
                if(iSampl < bTemp){
                    iSampl++;
                }else{
                    iSampl = 0;                    
                }
            }
            sensSW[i].sampl -= filterS[iSampl][i];
            filterS[iSampl][i] = wTemp;
            sensSW[i].sampl += filterS[iSampl][i];
            
            if(i == 1){
                bTemp = 2 * CONSTAKK - 1;
                if(iLevel < bTemp){
                    iLevel++;
                }else{
                    iLevel = 0;                    
                }
            }
            sensSW[i].level -= filterL[iLevel][i];
            filterL[iLevel][i] = wTemp;
            sensSW[i].level += filterL[iLevel][i];
            
            wTemp = sensSW[i].level;
            wTemp >>= 1;
                    
            if(!nWait){
            //���������� ������� � ���������� ��������            
                if(wTemp > sensSW[i].sampl){              
                    if ((wTemp - sensSW[i].sampl) > CONSTSIGNALON)
                    {
                        if(i){                        
                            //�������, � ��� �������� �������
                     //       LED_OPEN = 1 ^ flag.b.inverMov;                        
                     //       LED_CLOSE = 0 ^ flag.b.inverMov;
                            flag.b.direct = 1 ^ flag.b.inverMov;                        
                        }else{     
                            //�������, � ��� �������� �������                        
                     //       LED_OPEN = 0 ^ flag.b.inverMov;
                     //       LED_CLOSE = 1 ^ flag.b.inverMov;
                            flag.b.direct = 0 ^ flag.b.inverMov;                        
                        }                        
                        flag.b.swOn = 1; 
                        detect.b.checkBattery = 0;//������ �� �������� ������ �������
                        if(flag.b.direct)
                            commandForMotor = cSensSWOpen;
                        else
                            commandForMotor = cSensSWClose;
                    }          
                }             
            }                      
       //     if(!detect.b.readOk)//��� ������� ��� ���������                
        }    
        
        if(!detect.b.recData && !nWait)
            TMR2IE = 0;
        
        if(TMR2IE || TXIE || IOCAF5)
            continue;
        
        IOCBN5 = 1;  
        TMR1L = 0;
        TMR1H = 0;
        TMR1ON = 1;
        SWDTEN = 1;
        SLEEP();  
        SWDTEN = 0;
        if(!TMR2IE){
            TMR1ON = 0;                    
            dl[numCh] = (unsigned int)(TMR1H << 8) + TMR1L;                    
            fDl[numCh] = 1;                
            CPSCON1 = setCh[numCh]; //������������� �����               
            if (numCh) {       
                        /*wTemp = dl[numCh];
                          do{  
                           RA1 = 1;
                          wTemp -= 6;
                          RA1 = 0;
                        }while(wTemp > 7);   */      
                numCh = 0;                        
            } else {
                numCh = 1;
            }                    
        }
    }
}