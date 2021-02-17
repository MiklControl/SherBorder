/*
 * 18.09.2020
 * �������� ��������� �� ���������� ����
 * ���������� ������ ���������
 * ��������� ������ (������ ���)
 * ������ BT ����� Tuya
 * �������� ������ ������������
 */
#include <pic.h>
#include "fdataSB.h"

#define SMART

#pragma config WDTE = OFF, PWRTE = OFF, BOREN = OFF, MCLRE = ON, FCMEN = OFF, LVP = OFF, FOSC = INTOSC, STVREN = OFF

//��������� �������
void Initial(void);
byte checkSum(byte *p, byte size);
//------------------------------------------------------------------
//������ ����������� ����� 
byte checkSum(byte *p, byte size){
    byte sum = 0;
    for(byte i = 0; i < size; i++){
        sum += *p++;
    }        
    return sum;
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
          
    LED_LEFT_DIR = OUTPUT;
    LED_RIGHT_DIR = OUTPUT;
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;

    LATCbits.LATC5 = 0;
    IN1_DIR = OUTPUT;

    LATCbits.LATC4 = 0;
    IN2_DIR = OUTPUT;

    LATCbits.LATC7 = 0;
    swMove_DIR = OUTPUT;
    SSP1CON1bits.SSPEN = 0;
    ANSELCbits.ANSC7 = 0;
    sound_DIR = OUTPUT;
    sound = ON;
    onBAT_DIR = OUTPUT;
    onBAT = OFF;

    sensOpto_DIR = INPUT;
    IOCAN5 = 1; //��������� ��������� �� ������ RA5, ������� �� 1 � 0
    //IOCAP5 = 1;//��������� ��������� �� ������ RA5, ������� �� 0 � 1
    INLVLA5 = 0; //TTL   ������� ������
    IOCIE = 1;

    swConf_DIR = INPUT;
    OPTION_REGbits.nWPUEN = 0; //��������� ������ ������������� ����������       

    TRISA2 = OUTPUT;
    ANSELAbits.ANSA2 = 0;
    LATAbits.LATA2 = 0;

    CPSON = 1; //�������� ������
    //CPSRNG0 = 1;CPSRNG1 = 1;//������������ ��� 18 uA    
    CPSRNG0 = 0;
    CPSRNG1 = 1; //������������ ��� 1.2 uA    
    WPUC0 = 0;//
    WPUC1 = 0;    

    TMR0CS = 0; //������� Fosc/4
    PSA = 0;
    PS0 = 1;
    PS1 = 1;
    PS2 = 0;
    TMR1CS0 = 1;
    TMR1CS1 = 1; //11 =Timer1 clock source is Capacitive Sensing Oscillator (CAPOSC)
    TMR1ON = 1;

    T1GPOL = 1;
    T1GSS1 = 0;
    T1GSS0 = 1;
    T1GTM = 1;
    T1GSPM = 1;
    TMR1GE = 1;
    TMR1GIE = 1;

    T2CON = 0x07; //��� �� 64//0x7F;//�������� ������ � �������� �� 64 � �� 16
    TMR2IE = 1;

    //UART    
    TRISB5 = INPUT;
    TRISB7 = OUTPUT;
    ANSELBbits.ANSB5 = 0;
    LATBbits.LATB5 = 0;
    APFCON0bits.RXDTSEL = 0; //RB5 - RX
    APFCON0bits.TXCKSEL = 0; //RB7 - TX
    SYNC = 0;

    CREN = 1; //�������� ��������    
    BRGH = 1;
    BRG16 = 0;
    SPEN = 1;
    TXEN = 1;
    //SPBRGH = 0;
    //SPBRGL = 12; //19200 bit/s
    SPBRGH = 0;
    SPBRGL = 25; //9600 bit/s
    RCIF = 0;
    RCIE = 1;

    //������������ ���
    ADON = 1;
    ADFM = 1; //�������� ������   
    ADCON1bits.ADCS = 0b100; //Fosc/4
    //RA4 pin3 AN3
    WPUA4 = 0;
    CHS0 = 1;
    CHS1 = 1;
    CHS2 = 0;
    CHS3 = 0;
    ADIF = 0;
    ADIE = 1;

    p = (unsigned int *) &TMR1L;
    //T0IE = 1;

    PEIE = 1;
    GIE = 1;
}
//------------------------------------------------------------------
//������� ��������� ���������� 

__interrupt(high_priority) void Inter(void) {
    unsigned int wTemp;    
    byte *pArr;

    //������� ������ ��� ���� ������
    if (TMR1GIF && TMR1GIE) {
        TMR1GIF = 0;
        T1GCONbits.T1GGO = 1;
        wTemp = *p;
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

    if (ADIE && ADIF) {//����� ���
        wValADC.b[1] = ADRESH;
        wValADC.b[0] = ADRESL;
        if (wValADC.num > CONSTPOROG) {//������ � ������� ��������� � ������� ��������� ���������
            flag.b.currBig = 1;
            numBig++;
        }
        if (numBig == CONSTBIG) {//�������� ���������� - ������������� �����
            IN1 = 0;
            IN2 = 0;
            flag.b.motorMove = 0;
            numBig = 0;
        }
        ADIF = 0;
    }

    if (TMR2IE && TMR2IF) {
        if (nWait) {
            nWait--;
            if (!nWait) {//�� ���������� ��������� ����� ��������� ����� ���
                ADIF = 0;
                ADIE = 1;
            }
        }

        if (nWaitS) {
            nWaitS--;
        }

        if (timeTactRead) {
            timeTactRead--;
        } else//����� ����� �� ����� ������
            if (!detect.b.readOk) {//����� ��� �� ������
            numByteRX = 0; //����� ��������� ����� �����
        }

        if (!ADCON0bits.GO)
            ADCON0bits.GO = 1;

        TMR2IF = 0;
    }


    //���������� �� ����������� ������
    if (IOCIE && IOCAF5) {
        if (!nWait) {            
            nHalfTurn++;

            if (
                    ((nHalfTurn == 1) && (stat == stRevers)) || //������������� ����� ��� ������� �� ������
                    (nHalfTurn > Turn)
                    ) {
                IN1 = 0;
                IN2 = 0;
                flag.b.motorMove = 0;
            }
            TMR2 = 0;
            nWait = 5;            
        }
        IOCAF5 = 0;
        IOCIF = 0;
    }

    if (RCIE && RCIF) {
        
        if (RCSTAbits.OERR){// || detect.b.readOk) {//������ ������������ ��� �������� ����� ��� �� �������
            CREN = 0;
            NOP();
            CREN = 1;
        } else {//������ ����
            
            arrToRX[numByteRX] = RCREG;
            pArr = arrToRX + numByteRX;
            numByteRX++;
            switch (numByteRX) {
                case 1:
                {   
                    if (*pArr != 0x55) {//��� �� ������ ����
                        numByteRX = 0;
                    } else                        
                        timeTactRead = 10; //������������ ����� �� ����� ����� ������
                    break;
                }
                case 2:
                {
                    if (*pArr != 0xAA)//��� �� ������ ����
                        numByteRX = 0;
                    break;
                }
                    /*case 3://������� ������
                    case 4://�������
                    case 5:{//������� ���� � ���������� ����                                        
                        break;                    
                    }*/
                case 6:
                {
                    
                    allByteRX = 6 + 1 + *pArr; //����� ���������� ���� � ������
                    timeTactRead += *pArr * 2;
                    break;
                }
                default:
                {
                    if (numByteRX == allByteRX) {//����� ������ ���������
                        detect.b.readOk = 1;
numByteRX = 0;
                    }
                }
            }
        }

#ifdef SMART

#else
        comm = RCREG;
        switch (comm) {
            case 0x40:
            {
                cicleGo = 1;
                numRep.num = 0;
                Status |= 0x40;
                TXREG = Status;
                break;
            }//�������� ����������� �����
            case 0x41:
            {
                cicleGo = 0;
                Status &= 0xBF;
                TXREG = Status;
                break;
            }//��������� ����������� �����
            case 0x42:
            {
                flag.b.direct = 0b1;
                TXREG = Status;
                break;
            }//������� 
            case 0x43:
            {
                flag.b.direct = 0b0;
                TXREG = Status;
                break;
            }//�������
            case 0x44:
            {
                flag.b.swOn = 0b1;
                TXREG = Status;
                break;
            }//��������� ����������
            case 0x45:
            {
                TXREG = numRep.b[0];
                TXREG = numRep.b[1];
                break;
            }//������ ���������� ������
            case 0x46:
            {
                TXREG = Status;
                break;
            }//������ �������
            case 0x47:
            {
                TXREG = wValADC.b[0];
                TXREG = wValADC.b[1];
                break;
            }//������ ���
            case 0x48:
            {
                flag.b.tong = 0b1;
                TXREG = Status;
                break;
            }//������� ����� �������
            case 0x49:
            {
                flag.b.tong = 0b0;
                TXREG = Status;
                break;
            }//���������� ����� �������
            case 0x4A:
            {
                flag.b.inverMov = 0b1;
                TXREG = Status;
                break;
            }//������ �������
            case 0x4B:
            {
                flag.b.inverMov = 0b0;
                TXREG = Status;
                break;
            }//������ ��������

            case 0x51: case 0x52: case 0x53: case 0x54:
            case 0x55: case 0x56: case 0x57: case 0x58:
            case 0x59:
            {
                Turn = comm & 0x0F;
                TXREG = Turn;
                break;
            }//���������� ������������
        }
#endif        
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
    unsigned char i, Error;

    union {
        unsigned int wT;
        byte bT[2];
    } uTemp;

    int wTemp;
    byte bTemp;

#define CONSTPOR   63// 3    
#define ALLNUMPARAM 5//���������� ���������� + 1 ������� ���������� �� ��������
    
    int arrMean[3] = {0, 0, 0};

    //������������ ������
    unsigned int delta[3] = {0, 0, 0}; //���������� �������� �������� �� ������������
    unsigned char j, idUser;
    


    Initial();
    
    nWait = 0;
    RA2 = 0;
    arrToTX[0] = 0x55;
    arrToTX[1] = 0xAA;
    arrToTX[2] = 0x00; //������
    arrToTX[4] = 0x00; //������� ���� ��������� "���������� ����"

    swMove = ON;
    sound = OFF;

    Turn = 5;
 //   CPSON = 1; 
    LED_LEFT = 0;
    LED_RIGHT = 0;

    cicleGo = 0;
    flag.b.swOn = 0;
  numByteRX = 0;
    detect.b.firstOn = 1;
    
    while (1) {
        
        if (detect.b.readOk) {//���� ���� �������� �����, �� ����������� ���
            //�� ����� �� ��������� ����������� ����� � ��������� ������
            
            while (TXIE);//���� ����� ���������� ���������� ��������
                
LED_RIGHT ^= 1;
            switch (arrToRX[3]) {//���� � 3 - �������  
                case 0x00: {//�����
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
                case 0xE8 : {//������ ������ �����������
                    arrToTX[3] = 0xE8;
                    arrToTX[5] = 0x06;
                    //������ 100100
                    arrToTX[6] = 0x01;arrToTX[7] = 0x00;arrToTX[8] = 0x00;
                    arrToTX[9] = 0x01;arrToTX[10] = 0x00;arrToTX[11] = 0x00;
                    arrToTX[12] = checkSum(arrToTX, 12);
                    allByteTX = 13;
                    break;
                }                
                case 0x01 : {//������ ���� ���������
                    arrToTX[3] = 0x01;
                    arrToTX[5] = 0x08;
                    //� �������� ASCII (WINDOWS-1251) isnwhrlh = 69 73 6E 77 68 72 6C 68
                    arrToTX[6] = 0x69;arrToTX[7] = 0x73;arrToTX[8] = 0x6E; arrToTX[9] = 0x77;
                    arrToTX[10] = 0x68;arrToTX[11] = 0x72;arrToTX[12] = 0x6C; arrToTX[13] = 0x68;
                    arrToTX[14] = checkSum(arrToTX, 14);
                    allByteTX = 15;
                    break;
                }
                case 0x02 : {//������ �� ����������� ��������� ������ � ��������� ������ � ������
                    arrToTX[3] = 0x02;
                    arrToTX[5] = 0x00;
                    arrToTX[6] = 0x01;
                    allByteTX = 7;
                    break;
                }                
                case 0x03 : {//�������� ������� ������ ������
                    //arrToRX[6]//������� ������� (0�00 - �� ��������; 0�01 - �������� �� ���������; 0�02 - �������� ���������)
                    arrToTX[3] = 0x03;
                    arrToTX[5] = 0x00;
                    arrToTX[6] = 0x02;
                    allByteTX = 7;
                    break;
                }                 		
                case 0x08 : {//������ �������-�������� ����������� ��� �������� �� �� ��������
                    //�������� ��������� �������� � ������ ���������, ��������� ����� ����������
                    //������ ����� ������������� �������� ������ ����������� ������
                    arrToTX[3] = 0x07;//����� �� ������ ������� - �������� 
                    arrToTX[5] = 0x05;
                    arrToTX[6] = 0x09;//id ����� ������������ � ���� ��������
                    arrToTX[7] = 0x04;//������������� ��� ������
                    arrToTX[8] = 0x00; arrToTX[9] = 0x01;//���������� - ���� ����
                    arrToTX[10] = 0x01;//�������� 0x00: ������� ������� ������ �������, 
                    //0x01: ������� �������, 0x02: ������ ������� ������ �������, 0x03: ������� ���������
                    arrToTX[11] = checkSum(arrToTX, 11);
                    allByteTX = 12;
                    numParam = ALLNUMPARAM;
                    break;
                }
                case 0x07 : {//������������� ������ ������-������
                    if(numParam)//���� ���� �� ���������� ���������
                    {
                        switch(numParam){
                            case 1 : {//�������� �������� �� ������������� ����� ����������
                                arrToTX[3] = 0x07;//����� �� ������ ������� - �������� 
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x1C;//id ���� ����� 28
                                arrToTX[7] = 0x04;//������������� ��� ������
                                arrToTX[8] = 0x00; arrToTX[9] = 0x01;//���������� - ���� ����
                                arrToTX[10] = 0x07;//������� ����                                
                                arrToTX[11] = checkSum(arrToTX, 11);
                                allByteTX = 12;
                                break;
                            } 
                            case 2 : {//��������� �����
                                arrToTX[3] = 0x07;
                                arrToTX[5] = 0x05;            
                                arrToTX[6] = 0x2F;//47 dp id ������ �����
                                arrToTX[7] = 0x01;//��� ������ boolean
                                arrToTX[8] = 0x00; arrToTX[9] = 0x01;//���� ����� ������
                                arrToTX[10] = flag.b.direct;//��������� ����� ����� ������������ ������������                                                      
                                arrToTX[11] = checkSum(arrToTX, 11);
                                allByteTX = 12;                                 
                                break;
                            } 
                            default:{}
                        }
                        numParam--;
                    }else
                        allByteTX = 0;                    
                    break;
                }                        
                case 0x06 : {//������� ��� �����
                    arrToTX[3] = 0x07;//������� ������������� ������
                    switch(arrToRX[6]){
                        case 0x06 :{//������� ������� ����� �� Bluetooth, �� ��� �������� ����������� ������-�� ������ pdid46
                            LED_LEFT ^= 1;
                            arrToTX[5] = 0x06;arrToTX[6] = 0x06;
                            arrToTX[7] = 0x00;//��� ������ raw
                            arrToTX[8] = 0x00; arrToTX[9] = 0x02;//���������� - ��� �����
                            
                            flag.b.swOn = 1;
                            flag.b.direct = arrToRX[10];//0�00 - ������� �����, 0�01 - ������� ����� 
                            arrToTX[10] = flag.b.direct;
                            idUser = arrToRX[11];//������������� ������������
                            arrToTX[11] = idUser;
                            arrToTX[12] = checkSum(arrToTX, 12);
                            allByteTX = 13;  
                            break;
                        }
                        case 0x2E :{//������� ������� ����� � ������ �� Bluetooth pdid46
                            LED_LEFT ^= 1;
                            arrToTX[5] = 0x05;arrToTX[6] = 0x2E;
                            arrToTX[7] = 0x01;//��� ������ boolean
                            arrToTX[8] = 0x00; arrToTX[9] = 0x01;//���������� - ���� ����                            
                            arrToTX[10] = 0x01;//������� ������� �������
                            
                            flag.b.swOn = 1;
                            flag.b.direct = 0;//0 - ������� �����
                            
                            arrToTX[11] = checkSum(arrToTX, 11);
                            allByteTX = 12;  
                            break;
                        }
                        case 0x1C :{//���������� ����
                            break;
                        }                        
                        default : {}
                    }                                     
                    break;
                }
            }
            if(allByteTX){//���� ����� ��������� ��������
                numByteTX = 0;
                TXIE = 1; //��������� ��������
            }
            //LED_LEFT = 0; LED_RIGHT = 0;           
            detect.b.readOk = 0;
        }

        if (flag.b.swOn) {//������ ������� ������� ���������
            flag.b.swOn = 0;
            CPSON = 0;//��������� ��������� ������
            
            //������ ���������
            nHalfTurn = 0;
            swMove = ON; //�������� ������� � ��������� ������
            ADIE = 0; //
            IN1 = flag.b.direct ^ flag.b.inverMov;
            IN2 = !flag.b.direct ^ flag.b.inverMov;
            flag.b.motorMove = 1; //������ ����� �������
            stat = stDevTurn; //������ ������ ���
            numBig = 0;
            nWait = 5;

            while (1) {
                //}{                 

                if (!flag.b.motorMove) {
                    if (stat == stRevers) {//��������� ��� �������� ����
                        if (flag.b.currBig) {//� ����������� ����
                            Error = errRevers; //������� ��� ��������
                        } else {
                            IN1 = flag.b.direct ^ flag.b.inverMov; //�������� ������ �������� ������������
                            IN2 = !flag.b.direct ^ flag.b.inverMov;
                            //����� (���������� ��������)
                            TMR2IE = 0;
                            TMR2 = 0;
                            nWait = 2;
                            TMR2IE = 1;
                            while (nWait);
                            IN1 = 0;
                            IN2 = 0;
                        }
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
                                if (nHalfTurn < Turn)//������������� ���������� ������������
                                    Error = errHalf3; //������� �� �� ������, �� ��������� ������ ���������� ������������  
                                //������ ����������� �������� ������;
                                IN1 = !flag.b.direct ^ flag.b.inverMov;
                                IN2 = flag.b.direct ^ flag.b.inverMov;
                                flag.b.motorMove = 1; //��������� �����; 
                                nHalfTurn = 0;
                                //����� (���������� ��������)
                                stat = stRevers;
                            }
                            flag.b.currBig = 0;
                        } else {
                            //��������
                            IN1 = !flag.b.direct ^ flag.b.inverMov;
                            IN2 = flag.b.direct ^ flag.b.inverMov;
                            //����� (���������� ��������)
                            TMR2IE = 0;
                            TMR2 = 0;
                            nWait = 2;
                            TMR2IE = 1;
                            while (nWait);
                            IN1 = 0;
                            IN2 = 0;
                            break;
                        }
                    }
                }
            }
            numRep.num++;
            //CPSON = 1;//�������� ��������� ������
            //���������� ������� ��������� ������� �����
            while(TXIE);
            
            arrToTX[3] = 0x07;
            arrToTX[5] = 0x05;            
            arrToTX[6] = 0x2F;//47 dp id ������ �����
            arrToTX[7] = 0x01;//��� ������ boolean
            arrToTX[8] = 0x00; arrToTX[9] = 0x01;//���� ����� ������
            arrToTX[10] = flag.b.direct;//��������� ����� ����� ������������ ������������                                                      
            arrToTX[11] = checkSum(arrToTX, 11);
            allByteTX = 12;  
            numByteTX = 0;
            TXIE = 1; //��������� ��������
            
            if(flag.b.direct){//����� ������
                //���������� ������� �������� � ������ ������� "����� ������"
                while(TXIE);
            
                arrToTX[3] = 0xE0;//������� ������ �� ����
                arrToTX[5] = 0x09;//���������� ���� ������
                arrToTX[6] = 0x01;//����� ������������ ������
                arrToTX[7] = 0x13;//������� Ble unlock record
                arrToTX[8] = 0x02;//��� ������ value
                arrToTX[9] = 0x00;arrToTX[10] = 0x04;//���������� - 4 �����
                arrToTX[11] = 0; arrToTX[12] = 0; arrToTX[13] = 0;
                arrToTX[14] = idUser;
                arrToTX[15] = checkSum(arrToTX, 15);
                allByteTX = 16;  
                numByteTX = 0;
                TXIE = 1; //��������� ��������
            }
        }
        nWaitS = 100;
 //���� ��� �����       while (nWaitS); //����� ����� �������
        if (cicleGo) {//���� ����������� ����� ������ ��������
            flag.b.swOn = 1;
            flag.b.direct ^= 1;
            if (flag.b.direct) {
                Status |= 0x20;
            } else
                Status &= 0xDF;
        }        
        
        
        /* �������� ������� � ����� ������    
            if(TXIF && !nWaitS){//���� ����� ����� �������� �����������
                TXREG = Status;
            
                nWaitS = 5;
                //0x45 - �� �������; 0x62 - ������ �������            
            }
            if(comm){
                flag.b.direct = comm & 0b1;
                flag.b.swOn = 1;  
                comm = 0;            
            }
         */
        //}//��� ����� ������������
        //{        

        //������ ��������� ������
               
       
              for(i = 0; i < 2; i++)
              {                
                  if(flag.b.swOn)
                      continue;      
            
                  if(fDl[i] == 0){
                      continue;
                  }
                  fDl[i] = 0;
    
                 // GIE = 0;
                  wTemp = dl[i];
                //  GIE = 1;
            
                  //delta[i] = wTemp;
                  delta[i] = 50 + (arrMean[i] >> 4) - wTemp;
                  //delta[i] = 0;
                  if((arrMean[i] >> 4) > wTemp){
                      //delta[i] = arrMean[i] - wTemp;
                
                      if(delta[i] > CONSTPOR){ 
                    
                     
                  }
                  }
                              
              }

        //������ ������������ ��������
        /*akk[i] += wTemp;
        if(i == 2){
            
            if(iakk == 63){
                arrMean[0] = akk[0] >> 6;
                akk[0] = 0;
                arrMean[1] = akk[1] >> 6;
                akk[1] = 0;
                arrMean[2] = akk[2] >> 6;
                akk[2] = 0;
                iakk = 0;
            }else
                iakk++;
        }   */
        /* ���������� � �����       
               arrMean[i] -= arrAkk[j][i];
               arrAkk[j][i] = wTemp;
               arrMean[i] += arrAkk[j][i];
               if(i == 2){
                   if(j < 15){
                       j++;
                   }else{
                       j = 0;
                   }
               }
               //uTemp.wT = arrMean[i];
               uTemp.wT = delta[i];            
               if(uTemp.bT[0] & 0x80){
                   flag.b.bitData = 1;
               }else
                   flag.b.bitData = 0;
            
               uTemp.bT[0] &= 0x7F;
            
               //uTemp.bT[1] &= 0x1F;
               uTemp.bT[1] <<= 1;
               if(flag.b.bitData){
                   uTemp.bT[1] |= 1;
               }             
            
               uTemp.bT[1] &= 0x1F;
               uTemp.bT[1] |= i << 5;
               uTemp.bT[1] |= 0x80;*/
        /*���������� � �����    
             if(TXIF){//���� ����� ����� �������� �����������
                 TXREG = uTemp.bT[0];
                 while(!TXIF);//���� ����� ����� �������� �����������
                 TXREG = uTemp.bT[1];            
             }
         }*/
        //������ ������������������ ������������ ������ (���� 0, 1, 2)

    }
}