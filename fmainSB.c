/*
 * 18.09.2020
 * контроль двигателя по привышению тока
 * оптический датчик положения
 * сенсорные кнопки (только две)
 * модуль BT фирмы Tuya
 * контроль заряда аккумулятора
 */
#include <pic.h>
#include <proc\pic16f1829.h>
#include "fdataSB.h"

#define SMART
//#define test

#pragma config WDTE = OFF, PWRTE = OFF, BOREN = OFF, MCLRE = ON, FCMEN = OFF, LVP = OFF, FOSC = INTOSC, STVREN = OFF
    
//прототипы функций
void Initial(void);
byte checkSum(byte *p, byte size);
byte moveMotor();

//------------------------------------------------------------------
//расчет контрольной суммм 

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
    //крутим двигатель
    nHalfTurn = 0;
    
    //настраиваем канал АЦП
    ADON = 1;
    CHS0 = 1; CHS1 = 1; CHS2 = 0;
    ADIF = 0;
   
    LED_CLOSE = 0;            
    LED_OPEN = 0;
    flag.b.blink = 1;
    ADIE = 0; 
    
    IN1 = flag.b.direct;
    IN1 ^= flag.b.inverMov;
    IN2 = !flag.b.direct;
    IN2 ^= flag.b.inverMov;
    swMove = ON; //включаем драйвер и оптческий датчик
    
    flag.b.motorMove = 1; //статус мотор запущен
    stat = stDevTurn; //статус прямой ход
    numHighCurrent = 0;
    nWait = 18;//15;
    IOCAF5 = 0;
    IOCIF = 0;
    IOCIE = 1;

    while (1) {
        if (!flag.b.motorMove) {
RA1 = 1;
            if (stat == stRevers) {//остановка при обратном ходе
                if (flag.b.currBig) {//с превышением тока
                    Error = errRevers; //застрял при возврате
                } else {
                    IN1 = flag.b.direct;
                    IN1 ^= flag.b.inverMov;
                    IN2 = !flag.b.direct;
                    IN2 ^= flag.b.inverMov;
                    //IN1 = flag.b.direct ^ flag.b.inverMov; //тормозим реверс обратным направлением
                    //IN2 = !flag.b.direct ^ flag.b.inverMov;                    
                }
                //MPauseStop;
                break;
            }

            if (stat == stDevTurn) {
                //остановка при превышении пройдет, когда превышение будет зафиксировано долго
                if (flag.b.currBig) {//с превышением тока
                    if (flag.b.reperPos) {//позиция на репере
                        if (nHalfTurn < Turn) {//недостаточное количество полуоборотов
                            Error = errHalf1; //застрял на репере, не выполнив нужное количество полуоборотов    

                        } else {//достаточное количество полуоборотов
                            Error = errHalf2; //застрял на репере, выполнено нужное количество полуоборотов                              
                        }
                        break;
                    } else {//позиция НЕ на репере                               
                        if (nHalfTurn < Turn) {//недостаточное количество полуоборотов
                            Error = errHalf3; //застрял НЕ на репере, не выполнено нужное количество полуоборотов                                      
                            
                            //break; //выходим из цикла
                        }
                        //меняем направление вращения мотора;
                        IN1 = !flag.b.direct;
                        IN1 ^= flag.b.inverMov;
                        IN2 = flag.b.direct;
                        IN2 ^= flag.b.inverMov;
                        
                        flag.b.motorMove = 1; //запускаем мотор; 
                        nHalfTurn = 0;
                        nWait = 5;
                        stat = stRevers;
                    }
                    flag.b.currBig = 0;
                } else {//превышения по току нет, сработал оптический датчик
                    //тормозим
                    IN1 = !flag.b.direct;
                    IN1 ^= flag.b.inverMov;
                    IN2 = flag.b.direct;
                    IN2 ^= flag.b.inverMov;
                    
                    break;
                }
            }
RA1 = 0;
        }
        RA0 ^= 1;
        //где-то выключается, приходится дублировать
        //swMove = ON; //включаем драйвер и оптческий датчик
    }
    
    TMR2IE = 0;
    TMR2 = 0;
    nWait = 2;
    TMR2IF = 0; 
    TMR2IE = 1;
    while (nWait);
    IN1 = 0;IN2 = 0;
    
    flag.b.blink = 0;
    if(flag.b.direct){
        LED_OPEN = 1;
        LED_CLOSE = 0;
    }else{
        LED_OPEN = 0;
        LED_CLOSE = 1;
    }
    IOCIE = 0;
    swMove = OFF; //вЫключаем драйвер и оптческий датчик
    ADON = 0;
    return Error;
}
//------------------------------------------------------------------
//функция инициализации

void Initial(void) {
    SCS1 = 1;
    SPLLEN = 1;

    IRCF3 = 1;
    IRCF2 = 1;
    IRCF1 = 0;
    IRCF0 = 1; //4 MHz    

    LED_OPEN_DIR = OUTPUT;
    LED_CLOSE_DIR = OUTPUT;
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
    TRISA0 = OUTPUT;
    TRISA1 = OUTPUT;
    
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
    IOCAN5 = 1; //разрешаем прерыание по каналу RA5, перепад из 1 в 0
    //IOCAP5 = 1;//разрешаем прерыание по каналу RA5, перепад из 0 в 1
    INLVLA5 = 0; //TTL   триггер шмитта
    IOCIE = 1;

    swConf_DIR = INPUT;
    OPTION_REGbits.nWPUEN = 0; //разрешаем работу подтягивающих резисторов       

    
    ANSELAbits.ANSA2 = 0;
    ANSELCbits.ANSC2 = 0;
    LATAbits.LATA2 = 0;
    LATCbits.LATC2 = 0;
    
    //CPSRNG0 = 1;CPSRNG1 = 1;//максимальный ток 18 uA    
    CPSRNG0 = 0; CPSRNG1 = 1; //максимальный ток 1.2 uA    
    //CPSRNG0 = 1; CPSRNG1 = 0;
    WPUC0 = 0; //
    WPUC1 = 0;
    
    numCh = 0;
    CPSCON1 = setCh[numCh];

    TMR0CS = 0; //частота Fosc/4
    PSA = 0;
    PS0 = 1; PS1 = 1; PS2 = 0;
    //PS0 = 0; PS1 = 0; PS2 = 1;
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

    T2CON = 0x07; //вкл на 64//0x7F;//включаем таймер и делители на 64 и на 16
    TMR2IE = 1;

    //UART    
    TRISB5 = INPUT;
    TRISB7 = OUTPUT;
    ANSELBbits.ANSB5 = 0;
    LATBbits.LATB5 = 0;
    APFCON0bits.RXDTSEL = 0; //RB5 - RX
    APFCON0bits.TXCKSEL = 0; //RB7 - TX
    SYNC = 0;
    
    SPEN = 1;
#ifndef  test
    CREN = 1; //приемник включаем    
#endif
    BRGH = 1;
    BRG16 = 0;    
    TXEN = 1;
    //SPBRGH = 0;
    //SPBRGL = 12; //19200 bit/s
    SPBRGH = 0;
    SPBRGL = 25; //9600 bit/s
#ifdef test
    TXCKSEL = 1;//RC4 pin6
#endif
    RCIF = 0;
    RCIE = 1;

    //конфигурация АЦП
    ADCON0 = 0;
    ADON = 1;
    ADFM = 1; //равнение вправо   
    ADCON1bits.ADCS = 0b100; //Fosc/4
    //RA4 pin3 AN3 канал датчика тока для мотора
    FVREN = 1;//вкл опорное напряжение
    ADFVR0 = 0;ADFVR1 = 1;//2.048 V
    ADPREF0 = 1;ADPREF0 = 1;//используем опорное напряжение
    
    WPUA4 = 0; 
    //RС3 pin7 AN7 канал измерения заряда аккумулятора
    WPUC3 = 0;
    INLVLC3 = 1;

    //T0IE = 1;

    PEIE = 1;
    GIE = 1;
}
//------------------------------------------------------------------
//функция обработки прерываний 

__interrupt(high_priority) void Inter(void) {
    unsigned int wTemp;    

    //сделать только для двух кнопок
    if (TMR1GIF && TMR1GIE) {        
        TMR1GIF = 0;
        T1GCONbits.T1GGO = 1;
        
        wTemp = (unsigned int)(TMR1H << 8) + TMR1L;
        dl[numCh] = wTemp;
        fDl[numCh] = 1;

        CPSCON1 = setCh[numCh]; //устанавливаем канал
               
        if (numCh) {
            numCh = 0;
        } else {
            numCh = 1;
        }
        TMR1L = 0;
        TMR1H = 0;       
    }

    if (ADIE && ADIF) {//опрос АЦП
        wValADC.b[1] = ADRESH;
        wValADC.b[0] = ADRESL;
        if(detect.b.checkBattery){
            valuePowerADC = wValADC.num;
        }else{
            if (wValADC.num > CONSTPOROG) {//держим в крайнем положении в течении заданного интервала            
                numHighCurrent++;            
            } else {
                if (numHighCurrent)
                    numHighCurrent--;
            }
            if (numHighCurrent == CONSTBIG) {//интервал закончился - останавливаем мотор
                IN1 = 0;IN2 = 0;
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
            if (!nWait) {//по завершению временной паузы разрешаем опрос АЦП
                ADIF = 0;
                ADIE = 1;
            }
        }       

        if (timeTactRead) {
            timeTactRead--;
        } else//время вышло на прием пакета
            if (!detect.b.readOk) {//пакет еще не принят
            numByteRX = 0; //будем принимать новый пакет
        }

        if (!ADCON0bits.GO)
            ADCON0bits.GO = 1;
        
        if(flag.b.blink){
            nWaitS++;
            
            if(nWaitS & 0b10000){
                LED_CLOSE ^= 1 ^ flag.b.direct;            
                LED_OPEN ^= 0 ^ flag.b.direct;
            }else{
                LED_CLOSE = 0;
                LED_OPEN = 0;
            }
        }
        if (timeDelaySensSW) 
            timeDelaySensSW--;    
        
        //if((!timePower++) && (!flag.b.swOn))
        //    detect.b.checkBattery = 1;//разрешить проверку заряда аккумулятора
        
        TMR2IF = 0;
    }

    //прерывание по оптическому каналу
    if (IOCIE && IOCAF5) {
        if (!nWait) {
            nHalfTurn++;

            if (  ((nHalfTurn == 1) && (stat == stRevers)) || //останавливаем мотор при реверсе на репере
                    //         ((nHalfTurn >  Turn) && flag.b.direct  ) ||//направление открыть
                    //          ((nHalfTurn == Turn) && (!flag.b.direct)) //направление закрыть
                    (nHalfTurn > Turn)
                    ) {
                IN1 = 0;IN2 = 0;
                flag.b.motorMove = 0;
            }
            TMR2 = 0;
            nWait = 5;
        }
        IOCAF5 = 0;
        IOCIF = 0;
    }

    if (RCIE && RCIF) {

        if (RCSTAbits.OERR) {// || detect.b.readOk) {//ошибка переполнения или принятый пакет еще на анализе
            CREN = 0;
            NOP();
            CREN = 1;
        } else {//читаем байт

            arrToRX[numByteRX] = RCREG;            
            numByteRX++;
            switch (numByteRX) {
                case 1:
                {
                    if (arrToRX[0] != 0x55) {//это не первый байт
                        numByteRX = 0;
                    } else
                        timeTactRead = 10; //максимальное время на прием всего пакета
                    break;
                }
                case 2:
                {
                    if (arrToRX[1] != 0xAA)//это не второй байт
                        numByteRX = 0;
                    break;
                }
                    /*case 3://текущая версия
                    case 4://команда
                    case 5:{//старший байт в количестве байт                                        
                        break;                    
                    }*/
                case 6:
                {

                    allByteRX = 6 + 1 + arrToRX[5]; //общее количество байт в пакете
                    timeTactRead += arrToRX[5] * 2;
                    break;
                }
                default:
                {
                    if (numByteRX == allByteRX) {//пакет принят полностью
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
            }//включаем циклический режим
            case 0x41:
            {
                cicleGo = 0;
                Status &= 0xBF;
                TXREG = Status;
                break;
            }//вЫключаем циклический режим
            case 0x42:
            {
                flag.b.direct = 0b1;
                TXREG = Status;
                break;
            }//открыть 
            case 0x43:
            {
                flag.b.direct = 0b0;
                TXREG = Status;
                break;
            }//закрыть
            case 0x44:
            {
                flag.b.swOn = 0b1;
                TXREG = Status;
                break;
            }//запускаем однократно
            case 0x45:
            {
                TXREG = numRep.b[0];
                TXREG = numRep.b[1];
                break;
            }//чтение количества циклов
            case 0x46:
            {
                TXREG = Status;
                break;
            }//чтение статуса
            case 0x47:
            {
                TXREG = wValADC.b[0];
                TXREG = wValADC.b[1];
                break;
            }//чтение АЦП
            case 0x48:
            {
                flag.b.tong = 0b1;
                TXREG = Status;
                break;
            }//наличие языка защелки
            case 0x49:
            {
                flag.b.tong = 0b0;
                TXREG = Status;
                break;
            }//отсутствие языка защелки
            case 0x4A:
            {
                flag.b.inverMov = 0b1;
                TXREG = Status;
                break;
            }//реверс включен
            case 0x4B:
            {
                flag.b.inverMov = 0b0;
                TXREG = Status;
                break;
            }//реверс вЫключен

            case 0x51: case 0x52: case 0x53: case 0x54:
            case 0x55: case 0x56: case 0x57: case 0x58:
            case 0x59:
            {
                Turn = comm & 0x0F;
                TXREG = Turn;
                break;
            }//количество полуоборотов
        }
#endif        
    }
#ifndef test
    if (TXIE && TXIF) {
        TXREG = *(arrToTX + numByteTX++);
        if (numByteTX == allByteTX)//переданы все байты
            TXIE = 0;
    }
#endif
    return;
}
//------------------------------------------------------------------

void main(void) {
    unsigned char i;

    //прототип объекта замка
    union uLock{
        struct sLock{
            byte levelPower;//уровень заряда аккумулятора
            byte directMotor;//направление вращения двигателя при открытии
        }prop;
        byte all[2];
    } myLock;
    
//уровень заряда    
#define lHeight 0x00
#define lMedium 0x01
#define lLow    0x02
#define lZero   0x03
    
    for(i = 0; i < 2; i++)
        myLock.all[i] = 0;
    
    byte bTemp;

#define CONSTPOR   63// 3    
#define ALLNUMPARAM 7//количество параметров + 1 которое передается на смартфон
#define CONSTSIGNALON 80
    
    struct sSensorSW{
        unsigned int sampl;//текущее усредненное значение
        unsigned int level;//постоянный уровень
        unsigned int akk;//аккумулятор        
    } sensSW[2];
#define CONSTAKK    8     
    unsigned int filterS[CONSTAKK][2];//для фильтра НЧ
    unsigned int filterL[CONSTAKK * 2][2];
    
    unsigned char iSampl, idUser, iakk, iLevel;   
    
    unsigned int wTemp;    
    union{
        unsigned int w;    
        byte b[2];
    }wADC; 
    
    //источник команды
    byte commandForMotor;
    #define cNoComm        0x00    
    #define cBLEOpen       0x01
    #define cSensSWOpen    0x02
    #define cBLEClose       0x03
    #define cSensSWClose    0x04    

    Initial();

    nWait = 0;
    //RA2 = 0;
    arrToTX[0] = 0x55;
    arrToTX[1] = 0xAA;
    arrToTX[2] = 0x00; //версия
    arrToTX[4] = 0x00; //старший байт параметра "количество байт"

    //swMove = ON;
    sound = OFF;

    Turn = 7; //чтоб на верняка
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
    
    while (1) {

        if (detect.b.readOk) {//если есть принятый пакет, то анализируем его
            //не плохо бы проверить контрольную сумму у принятого пакета

            while (TXIE); //ждем когда завершится предыдущая передача

            switch (arrToRX[3]) {//байт № 3 - команда  
                case 0x00:
                {//ПУЛЬС
                    arrToTX[3] = 0x00;
                    arrToTX[5] = 0x01;
                    if (detect.b.firstOn) {//первое включение
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
                {//запрос версии контроллера
                    arrToTX[3] = 0xE8;
                    arrToTX[5] = 0x06;
                    //версия 100100
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
                {//запрос кода продукции
                    arrToTX[3] = 0x01;
                    arrToTX[5] = 0x08;
                    //в кодироки ASCII (WINDOWS-1251) isnwhrlh = 69 73 6E 77 68 72 6C 68
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
                {//запрос на определение контактов сброса и индикатор работы у модуля
                    arrToTX[3] = 0x02;
                    arrToTX[5] = 0x00;
                    arrToTX[6] = 0x01;
                    allByteTX = 7;
                    break;
                }
                case 0x03:
                {//сообщает рабочий статус модуля
                    //arrToRX[6]//значени статуса (0х00 - не привязан; 0х01 - привязан не подключен; 0х02 - привязан подключен)
                    arrToTX[3] = 0x03;
                    arrToTX[5] = 0x00;
                    arrToTX[6] = 0x02;
                    allByteTX = 7;
                    break;
                }
                case 0x08:
                {//запрос статуса-настроек контроллера для передачи их на смартфон
                    //отвечаем передачей сведений о первом параметре, остальные будем отправлять
                    //только после подтверждения удачного приема предыдущего пакета
                    arrToTX[3] = 0x07; //ответ на запрос статуса - настроек 
                                arrToTX[5] = 0x08;
                                arrToTX[6] = 0x08; //id заряд аккумулятора в виде значений
                                arrToTX[7] = 0x02; //тип данных интеджер
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x04; //количество - 4 байт
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
                {//подтверждение приема СТАТУС-ОТВЕТа
                    if (numParam)//если есть не переданные параметры
                    {
                        switch (numParam) {
                            case 1:
                            {//передача сведений об установленном языке интерфейса
                                arrToTX[3] = 0x07; //ответ на запрос статуса - настроек 
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x1C; //id язык замка 28
                                arrToTX[7] = 0x04; //перечисляемый тип данных
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //количество - один байт
                                arrToTX[10] = 0x07; //русский язык                                
                                arrToTX[11] = checkSum(arrToTX, 11);
                                allByteTX = 12;
                                break;
                            }
                            case 2:
                            {//состояние замка
                                arrToTX[3] = 0x07;
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x2F; //47 dp id статус замка
                                arrToTX[7] = 0x01; //тип данных boolean
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //один байта данных
                                arrToTX[10] = flag.b.direct; //состояния замка пусть определяется направлением                                                      
                                arrToTX[11] = checkSum(arrToTX, 11);
                                allByteTX = 12;
                                break;
                            }
                            case 3:
                            {//состояние замка
                                arrToTX[3] = 0x07;
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x30; //48 dp направление вращения двигателем
                                arrToTX[7] = 0x04; //тип данных 
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //один байта данных
                                arrToTX[10] = flag.b.inverMov; //состояния замка пусть определяется направлением                                                      
                                arrToTX[11] = checkSum(arrToTX, 11);
                                allByteTX = 12;
                                break;
                            }
                            case 4:
                            {
                                arrToTX[3] = 0x07; //ответ на запрос статуса - настроек 
                    arrToTX[5] = 0x05;
                    arrToTX[6] = 0x09; //id заряд аккумулятора в виде градаций
                    arrToTX[7] = 0x04; //перечисляемый тип данных
                    arrToTX[8] = 0x00;
                    arrToTX[9] = 0x01; //количество - один байт
                    arrToTX[10] = 2;//myLock.prop.levelPower; //значение 0x00: высокий уровень заряда батареи, 
                    //0x01: средняя батарея, 0x02: низкий уровень заряда батареи, 0x03: батарея разряжена
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
                {//команда для замка
                    arrToTX[3] = 0x07; //команда подтверждения приема                    
                    switch (arrToRX[6]) {
                        case 0x06:
                        {//команда открыть замок по Bluetooth, но для закрытия исползуется почему-то другая pdid46

                            arrToTX[5] = 0x06;
                            arrToTX[6] = 0x06;
                            arrToTX[7] = 0x00; //тип данных raw
                            arrToTX[8] = 0x00;
                            arrToTX[9] = 0x02; //количество - два байта
                            commandForMotor = cBLEOpen;
                            
                            flag.b.swOn = 1;
                            flag.b.direct = 1; //arrToRX[10];//0х00 - закрыть замок, 0х01 - открыть замок 
                            detect.b.checkBattery = 0;
                            
                            arrToTX[10] = flag.b.direct;
                            idUser = arrToRX[11]; //идентификатор пользователя
                            arrToTX[11] = idUser;
                            arrToTX[12] = checkSum(arrToTX, 12);
                            allByteTX = 13;
                            break;
                        }
                        case 0x2E:
                        {//команда закрыть замок в ручную по Bluetooth pdid46                            
                            arrToTX[5] = 0x05;
                            arrToTX[6] = 0x2E;
                            arrToTX[7] = 0x01; //тип данных boolean
                            arrToTX[8] = 0x00;
                            arrToTX[9] = 0x01; //количество - один байт                            
                            arrToTX[10] = 0x01; //успешно принята команда
                            commandForMotor = cBLEClose;
                            flag.b.swOn = 1;
                            flag.b.direct = 0; //0 - закрыть замок
                            detect.b.checkBattery = 0;
                            
                            arrToTX[11] = checkSum(arrToTX, 11);
                            allByteTX = 12;
                            break;
                        }
                        case 0x30:
                        {//команда направление вращения двигателем                            
                            arrToTX[5] = 0x05;
                            arrToTX[6] = 0x30;
                            arrToTX[7] = 0x04; //тип данных raw
                            arrToTX[8] = 0x00;
                            arrToTX[9] = 0x01; //количество - один байт                                                        
                            flag.b.inverMov = arrToRX[10];
                            arrToTX[10] = flag.b.inverMov;
                            arrToTX[11] = checkSum(arrToTX, 11);
                            allByteTX = 12;
                            break;
                        }
                        case 0x1C:
                        {//установить язык
                            break;
                        }
                        default:
                        {
                        }
                    }
                    break;
                }
            }
            if (allByteTX) {//есть байты требующие передачу
                numByteTX = 0;
                TXIE = 1; //разрешаем передачу
            }                       
            detect.b.readOk = 0;
        }

        if (flag.b.swOn) {//пришла команта крутить двигатель            
            CPSON = 0; //вЫключаем сенсорные кнопки
            TMR1ON = 0;           
            //нужен возврат при застревании
            //        if(moveMotor()){//возникла ошибка
            //            flag.b.direct ^= 1;//все в исходную
            RCIE = 0;
            RCIF = 0;
            moveMotor();
            RCIE = 1;
            //        }else{
            TMR1ON = 1;
            //отправляем команду изменения статуса замка
            while (TXIE);

            arrToTX[3] = 0x07;
            arrToTX[5] = 0x05;
            arrToTX[6] = 0x2F; //47 dp id статус замка
            arrToTX[7] = 0x01; //тип данных boolean
            arrToTX[8] = 0x00;
            arrToTX[9] = 0x01; //один байта данных
            arrToTX[10] = flag.b.direct; //состояния замка пусть определяется направлением                                                      
            arrToTX[11] = checkSum(arrToTX, 11);
            allByteTX = 12;
            numByteTX = 0;
            TXIE = 1; //разрешаем передачу

            switch (commandForMotor){
                case cBLEOpen :{//замок открыт по  Bluetooth            
                    //отправляем команду записать в журнал событие "замок открыт"
                    while (TXIE);
                    arrToTX[3] = 0xE0; //команда записи во флэш
                    arrToTX[5] = 0x09; //количество байт данных
                    arrToTX[6] = 0x01; //время контролирует модуль
                    arrToTX[7] = 0x13; //команда Ble unlock record
                    arrToTX[8] = 0x02; //тип данных value
                    arrToTX[9] = 0x00;
                    arrToTX[10] = 0x04; //количество - 4 байта
                    arrToTX[11] = 0;
                    arrToTX[12] = 0;
                    arrToTX[13] = 0;
                    arrToTX[14] = idUser;
                    arrToTX[15] = checkSum(arrToTX, 15);
                    allByteTX = 16;
                    numByteTX = 0;
                    TXIE = 1; //разрешаем передачу
                    break;
                }
            case cSensSWOpen :{//замок открыт с помощью сенсорной клавитуры          
                    //отправляем команду записать в журнал предупреждений "замок открыт"
                    while (TXIE);
                    
                    arrToTX[3] = 0xE0; //команда записи во флэш
                    arrToTX[5] = 0x06; //количество байт данных
                    arrToTX[6] = 0x01; //время контролирует модуль
                    arrToTX[7] = 0x15; //команда lock alarm
                    arrToTX[8] = 0x04; //тип данных enum
                    arrToTX[9] = 0x00;
                    arrToTX[10] = 0x01; //количество - 1 байта
                    arrToTX[11] = 0x09;//открыть сенсорной кнопкой
                    arrToTX[12] = checkSum(arrToTX, 12);
                    allByteTX = 13;
                    numByteTX = 0;
                    TXIE = 1; //разрешаем передачу
                    break;
                }
            case cSensSWClose :{//замок закрыть с помощью сенсорной клавиатуры             
                    //отправляем команду записать в журнал предупреждений "замок закрыт"
                    while (TXIE);
                    
                    arrToTX[3] = 0xE0; //команда записи во флэш
                    arrToTX[5] = 0x06; //количество байт данных
                    arrToTX[6] = 0x01; //время контролирует модуль
                    arrToTX[7] = 0x15; //команда lock alarm
                    arrToTX[8] = 0x04; //тип данных enum
                    arrToTX[9] = 0x00;
                    arrToTX[10] = 0x01; //количество - 1 байта
                    arrToTX[11] = 0x0A;//закрыть сенсорной кнопкой
                    arrToTX[12] = checkSum(arrToTX, 12);
                    allByteTX = 13;
                    numByteTX = 0;
                    TXIE = 1; //разрешаем передачу
                    break;
                }
            }            
            flag.b.swOn = 0;
            detect.b.checkBattery = 1;                        
            commandForMotor = cNoComm;
        }       
        
        //для теста аккумулятора
        if(detect.b.checkBattery){
  //          CPSON = 0;//вЫключаем сенсорные кнопки 
     /*       ADIE = 0;
            ADON = 1;
            //настраиваем канал АЦП AN7
            CHS0 = 1; CHS1 = 1; CHS2 = 1;
            
            //подключаем делитель
            onBAT = ON; 
            nWait = 4;
            valuePowerADC = 0;
            while(!valuePowerADC);
            valuePowerADC = 0;
            while(!valuePowerADC);//измеряем два раза на всякий случай
            wADC.w = valuePowerADC;
            
            if(valuePowerADC > CONSTHEIGHT){
                bTemp = lHeight;
            } else {
                if(valuePowerADC > CONSTMEDIUM){
                    bTemp = lMedium;
                } else{
                    if(valuePowerADC > CONSTLOW){
                        bTemp = lLow;
                    } else{
                        bTemp = lZero;
                    }
                }
            }
            ADON = 0;
           
            //if (bTemp != myLock.prop.levelPower)
            {            
                myLock.prop.levelPower = bTemp;//меняем статус зарядки, сообщаем смартфону
                while (TXIE); //ждем когда завершится предыдущая передача
                //55 AA 00 E0 00 06 01 09 04 00 01 01 F5
                arrToTX[3] = 0xE0; //ответ на запрос статуса - настроек 
                arrToTX[5] = 0x06;
                arrToTX[6] = 0x01;
                arrToTX[7] = 0x09; //id заряд аккумулятора в виде градаций
                arrToTX[8] = 0x04; //перечисляемый тип данных
                arrToTX[9] = 0x00;
                arrToTX[10] = 0x01; //количество - один байт
                arrToTX[11] = myLock.prop.levelPower; 
                arrToTX[12] = checkSum(arrToTX, 12);
                allByteTX = 13; 
                numByteTX = 0;
                TXIE = 1;                       
            }
            //выключае делитель
            onBAT = OFF;
      */      
            CPSON = 1;//включаем сенсорные кнопки            
            timeDelaySensSW = 10;
            detect.b.sensSWzero = 1;
            detect.b.checkBattery = 0; 
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
            sensSW[0].akk = 0;
            sensSW[1].akk = 0;
            sensSW[0].level = 0;
            sensSW[1].level = 0;
            detect.b.sensSWzero = 0;
        }
        //анализ сенсорных кнопок
        for (i = 0; i < 2; i++) {            
            if (fDl[i] == 0) {
                continue;
            }
            fDl[i] = 0;
            wTemp = dl[i];            
      
            //расчет усредненного значения
  /*          sensSW[i].akk += wTemp;
            if(i == 1){            
                if(iakk == 15){//усредняем для постоянного уровня по 16 значениям,
                //а для мгновенного отсчета по 8 значениям, чтобы не делить на 8 и на 16
                //сдвинем постоянный уровень только на 1
                    sensSW[0].level = sensSW[0].akk >> 1;
                    sensSW[0].akk = 0;
                    sensSW[1].level = sensSW[1].akk >> 1;
                    sensSW[1].akk = 0;                
                    iakk = 0;
                }else
                    iakk++;
            }  
 */       
            //НЧ фильтрация  
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
                    
            if(!timeDelaySensSW){
            //сравниваем уровень и мгновенное значение
            //if(sensSW[i].level > sensSW[i].sampl){
              if(wTemp > sensSW[i].sampl){
              //  if ((sensSW[i].level - sensSW[i].sampl) > CONSTSIGNALON)
                  if ((wTemp - sensSW[i].sampl) > CONSTSIGNALON)
                {
                    if(i){                        
                        //открыть, а при инверсии закрыть
                        LED_OPEN = 1 ^ flag.b.inverMov;                        
                        LED_CLOSE = 0 ^ flag.b.inverMov;
                        flag.b.direct = 1 ^ flag.b.inverMov;
                        
                    }else{     
                        //закрыть, а при инверсии открыть                        
                        LED_OPEN = 0 ^ flag.b.inverMov;
                        LED_CLOSE = 1 ^ flag.b.inverMov;
                        flag.b.direct = 0 ^ flag.b.inverMov;
                        
                    }                        
                    flag.b.swOn = 1; 
                    detect.b.checkBattery = 0;//запрет на проверку заряда батареи
                    if(flag.b.direct)
                        commandForMotor = cSensSWOpen;
                    else
                        commandForMotor = cSensSWClose;
                }          
            } 
            
            }
#ifdef test
            if(i)
                uTemp.wT = sensSW[0].sampl & 0x3FFF;
            else
                uTemp.wT = sensSW[0].level & 0x3FFF;            
                
            uTemp.wT <<= 1;
            bTemp = uTemp.bT[0];
            bTemp >>= 1;
            while(!TXIF);//ждем когда буфер передачи освободится
            TXREG = 0x80 + i;
            NOP();NOP();
            while(!TXIF);
            TXREG = bTemp;
            NOP();NOP();
            while(!TXIF);//ждем когда буфер передачи освободится
            bTemp = uTemp.bT[1];                       
            TXREG = bTemp;
#endif            
               //uTemp.wT = arrMean[i];
               /*uTemp.wT = delta[i];            
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
          
             
         }        
    }
}