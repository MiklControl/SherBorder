/*
 * 18.09.2020
 * контроль двигателя по привышению тока
 * оптический датчик положения
 * сенсорные кнопки (только две)
 * модуль BT фирмы Tuya
 * контроль заряда аккумулятора
 */
#include <pic.h>
#include <proc\pic16f1828.h>
#include "fdataSB.h"

#define modeWork
//#define modeRepeat
//#define modeSW

#define setbit(var, bit) ((var) |= (1 << (bit)))
#define clrbit(var, bit) ((var) &= ~(1 << (bit)))

//#pragma config WDTE = OFF, PWRTE = OFF, BOREN = OFF, MCLRE = ON, FCMEN = OFF, LVP = OFF, FOSC = INTOSC, STVREN = OFF
#pragma config WDTE = SWDTEN, PWRTE = OFF, BOREN = OFF, MCLRE = ON, FCMEN = OFF, LVP = OFF, FOSC = INTOSC, STVREN = OFF
//прототипы функций
void Initial(void);
byte checkSum(byte size);
byte moveMotor();

//------------------------------------------------------------------
//расчет контрольной суммм 
byte checkSum(byte size) {
    byte sum = 0;
    for (byte i = 0; i < size; i++) {
        sum += arrToTX[i];
    }
    return sum;
}
//------------------------------------------------------------------
//крутим двигатель
byte moveMotor() {
    byte Error = 0;   
    nHalfTurn = 0;
    
    //настраиваем канал АЦП
    ADCON0 = 0b1101;//канал AN3, АЦП включаем    
    
    //открыть, а при инверсии закрыть
    LED_OPEN = flag.b.direct;                        
    LED_CLOSE = flag.b.direct ^ 1;
                            
    //определяем направление вращения мотора
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
    stat = stDevTurn; //статус прямой ход    
    numHighCurrent = 0;
    numLowBatt = 0;
    intervalTimeADC = 0xFF;//0xD0;   
    nWait = 7;
    TMR2IE = 1;   
    IOCAF5 = 0;
    flag.b.motorMove = 1; //статус мотор запущен
    swMove = ON; //включаем драйвер и оптческий датчик
    while (1) {
        if (!flag.b.motorMove) {//анализируем когда моторо остановлен
 //вроде вообще не надо           while(numByteRX);
            while(TXIE);
            GIE = 0;
            if (stat == stRevers) {//остановка при обратном ходе
                if (flag.b.currBig) {//с превышением тока
                    Error = errRevers; //застрял при возврате
                } else {
                    if(bTemp){
                        setbit(PORTC, bIN1);
                        clrbit(PORTC, bIN2);
                    }else{
                        setbit(PORTC, bIN2);
                        clrbit(PORTC, bIN1);
                    }//тормозим реверс обратным направлением                                                       
                }                
                break;
            }

            if (stat == stDevTongue) {//держим язычок
                CCP1CON = 0x00;//выключаем ШИМ
                T4CON = 0x00;//выключаем таймер 4  
                //меняем направление вращения мотора;  
                        if(bTemp){//реверс
                            setbit(PORTC, bIN2);
                            clrbit(PORTC, bIN1);
                        }else{
                            setbit(PORTC, bIN1);
                            clrbit(PORTC, bIN2);
                        }
                        
                        flag.b.motorMove = 1; //запускаем мотор;
                        nHalfTurn = 0;
                        nWait = 5;
                        stat = stRevers;
                        intervalTimeADC = 0xFF;
            }
            
            if (stat == stDevTurn) {//прямой ход
                //остановка при превышении пройдет, когда превышение будет зафиксировано долго
                if (flag.b.currBig) {//с превышением тока
                    if (flag.b.reperPos) {//позиция на репере
                        if (nHalfTurn < Turn) {//недостаточное количество полуоборотов
                            Error = errHalf1; //застрял на репере, не выполнив нужное количество полуоборотов
                        } else {//достаточное количество полуоборотов
                            Error = errHalf2;
                            //застрял на репере, выполнено нужное количество полуоборотов
                        }
                        break;
                    } else {//позиция НЕ на репере
                        if (nHalfTurn < Turn) {//недостаточное количество полуоборотов
                            Error = errHalf3; //застрял НЕ на репере, не выполнено нужное количество полуоборотов                            
                        }

                        stat = stDevTongue;
                        PR4 = 9;
                        CCP1CON = 0x0C;
                        CCPR1L = 5;
                        C1TSEL0 = 1;//таймер № 4
                        C1TSEL1 = 0;
                        T4CON = 0x04;

                        intervalTimeADC = 0x00;
                        nWait = 100;
                        flag.b.motorMove = 1; //запускаем мотор;
                    }
                    flag.b.currBig = 0;
                } else {//превышения по току нет, сработал оптический датчик
                    //тормозим
                    //IN1 = 0;IN2 = 0;
                    if(bTemp){//реверс
                        setbit(PORTC, bIN2);
                        clrbit(PORTC, bIN1);
                    }else{
                        setbit(PORTC, bIN1);
                        clrbit(PORTC, bIN2);
                    }
                    break;
                }
            }
            GIE = 1;
            
        }
        //где-то выключается, приходится дублировать
        //swMove = ON; //включаем драйвер и оптческий датчик    
    }
    GIE = 1;

    TMR2IE = 0;
    TMR2 = 0;
    nWait = 2;
    TMR2IF = 0;
    TMR2IE = 1;
    while (nWait);
    IN1 = 0;IN2 = 0;
    LED_OPEN = 0;
    LED_CLOSE = 0;

    swMove = OFF; //вЫключаем драйвер и оптческий датчик
    TMR2IE = 0;
    ADON = 0;
    return Error;
}
//------------------------------------------------------------------
//функция инициализации
void Initial(void) {
    SCS1 = 1;
    SPLLEN = 1;

    IRCF3 = 1;IRCF2 = 1;IRCF1 = 0;IRCF0 = 1; //4 MHz    
        
    LED_OPEN_DIR = OUTPUT;
    LED_CLOSE_DIR = OUTPUT;
    
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    ANSELAbits.ANSA0 = 0;   
    ANSELAbits.ANSA1 = 0;
    
    TRISA1 = OUTPUT;
    
    #ifdef modeRepeat
    TRISA0 = INPUT;
    IOCAN0 = 1; //разрешаем прерыание по каналу RA0, перепад из 1 в 0
    #else
    TRISA0 = OUTPUT;
    IOCBN5 = 1; //разрешаем прерыание по каналу RB5, перепад из 1 в 0 
    #endif

    #ifdef modeSW
    TRISA0 = INPUT;
    TRISA1 = INPUT;
    IOCAN0 = 1; //разрешаем прерыание по каналу RA0, перепад из 1 в 0
    IOCAN1 = 1; //разрешаем прерыание по каналу RA0, перепад из 1 в 0
    #endif

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
    ANSELBbits.ANSB4 = 0;
    LATBbits.LATB4 = 0;
    onBAT_DIR = OUTPUT;
    onBAT = OFF;

    sensOpto_DIR = INPUT;
    IOCAN5 = 1; //разрешаем прерыание по каналу RA5, перепад из 1 в 0
    INLVLA5 = 0; //TTL   триггер шмитта    
    IOCIE = 1;

    //swConf_DIR = INPUT;
    swConf_DIR = OUTPUT;    
    LATBbits.LATB6 = 0;
    
    OPTION_REGbits.nWPUEN = 0; //разрешаем работу подтягивающих резисторов

    ANSELAbits.ANSA2 = 0;
    ANSELCbits.ANSC2 = 0;
    LATAbits.LATA2 = 0;
    LATCbits.LATC2 = 0;
    
    //CPSRNG0 = 1;CPSRNG1 = 1;//максимальный ток 18 uA
    CPSRNG0 = 0; CPSRNG1 = 1; //максимальный ток 1.2 uA
    //CPSRNG0 = 1; CPSRNG1 = 0;
    WPUC0 = 0;
    WPUC1 = 0;
    
    numCh = 0;
    CPSCON1 = setCh[numCh];

    TMR0CS = 0; //частота Fosc/4
    PSA = 0;
    PS0 = 1; PS1 = 1; PS2 = 0;
    
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
    WDTCON |= 0b00001000;//00100 = 1:512 (Interval 16 ms nominal)
    //WDTCON |= 0b00001110;//00111 = 1:4096 (Interval 128 ms nominal)

    T2CON = 0x07; //вкл на 64   //0x7F;//включаем таймер и делители на 64 и на 16
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
    SPBRGH = 0;
    SPBRGL = 25; //9600 bit/s
    RCIE = 0;

    //конфигурация АЦП
    ADCON0 = 0;
    ADON = 1;
    ADFM = 1; //равнение вправо
    //ADCON1bits.ADCS = 0b100; //Fosc/4
    ADCON1bits.ADCS0 = 1;ADCON1bits.ADCS1 = 0;ADCON1bits.ADCS2 = 1;//Fosc/16 4us
    //RA4 pin3 AN3 канал датчика тока для мотора
    /*FVREN = 1;//вкл опорное напряжение
    ADFVR0 = 0;ADFVR1 = 1;//2.048 V
    ADPREF0 = 1;ADPREF0 = 1;//используем опорное напряжение
     * */
    ADIE = 1;
    /*TRISA4 = INPUT;
    T1GSEL = 1;
    ANSA4 = 1;*/
    WPUA4 = 0;
    //RС3 pin7 AN7 канал измерения заряда аккумулятора
    WPUC3 = 0;
    INLVLC3 = 1;
        
    SWDTEN = 0;

    PEIE = 1;    
    GIE = 1;
}
//------------------------------------------------------------------
//функция обработки прерываний 
__interrupt(high_priority) void Inter(void) {
    unsigned int wTemp;                  
    byte bTemp;
   
    SWDTEN = 0;
   
    //для двух кнопок    
    if (ADIE && ADIF) {//опрос АЦП        
        wADC.b[0] = ADRESL;
        wADC.b[1] = ADRESH;  
        if(detect.b.checkBattery){            
            if(wADC.w == 0)
                wADC.w = 1;
        }else
        {//анализируем сигнал с датчика тока                                                           
            if (wADC.w > CONSTPOROG) {
                numHighCurrent += 4;//вариант фильтра,увеличение с коэффициентом 4, уменьшение с коэффициентом 1
                intervalTimeADC = 0xFF;                
            } else {                
                if (numHighCurrent)
                    numHighCurrent--;
            }
            
            numLowBatt++;
            if ((numHighCurrent >= CONSTBIG) || (numLowBatt == 6000)) {//количество превышений досигла максимума                
                flag.b.motorMove = 0;
                flag.b.currBig = 1;
                numHighCurrent = 0;
                numLowBatt = 0;
            }
        }        
        ADIF = 0;        
    }

    if (TMR2IE && TMR2IF) {       
        if (nWait) {
            nWait--;
        }else{//по завершению временной паузы разрешаем опрос АЦП
            if(stat == stDevTongue){
                flag.b.motorMove = 0;
            }else{
                ADIF = 0;
                if(detect.b.checkBattery)// измеряем заряд батареи
                    if(!ADCON0bits.GO){
                        ADCON0bits.GO = 1;
                    }
                if(flag.b.motorMove){//двигатель крутится
                    if(!ADCON0bits.GO)
                        ADCON0bits.GO = 1;
                    TMR2 = intervalTimeADC;//косвенным образом устанавливаем частоту опроса АЦП                                                             
                }
            }
        }
/*    if(!flag.b.motorMove){

        if (timeTactRead) {
            timeTactRead--;
        } else//время вышло на прием пакета
            if (!detect.b.readOk) {//пакет еще не принят
                numByteRX = 0; //будем принимать новый пакет                                
            }
    } */   
        //if((!timePower++) && (!flag.b.swOn))
        //    detect.b.checkBattery = 1;//разрешить проверку заряда аккумулятора 
        
        if(synNum)
            synNum--;
        TMR2IF = 0;        
    }
    if(IOCIE){                
        //прерывание по оптическому каналу
        if(IOCAF5) {
            numLowBatt = 0;
            if (!nWait) {
                nHalfTurn++;
            
                if (  ((nHalfTurn == 1) && (stat == stRevers)) ||  
                        //останавливаем мотор при реверсе на репере
                    //         ((nHalfTurn >  Turn) && flag.b.direct  ) ||//направление открыть
                    //          ((nHalfTurn == Turn) && (!flag.b.direct)) //направление закрыть
                        (nHalfTurn > Turn) 
                    ) {
                    flag.b.motorMove = 0;
                }
                TMR2 = 0;
                nWait = 5;
                TMR2IE = 1;
            }            
            IOCAF5 = 0;            
        }
        
        #ifdef modeSW
        if(IOCAF0){
           flag.b.swOn = 1;
           flag.b.direct = 0; //0 - закрыть замок
           IOCAF0 = 0;
        }        
        if(IOCAF1){
           flag.b.swOn = 1;
           flag.b.direct = 1; //1 - открыть замок
           IOCAF1 = 0;
        }        
        #endif                   
        
        #ifdef modeRepeat
        if(IOCAF0)
        #else
        if(IOCBF5)
        #endif
        {            
            TMR2IE = 1;
            byte numBit = 7;
            detect.b.UART = 1;
        NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
            do{
                //пауза для одного бита
                NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
                NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
                NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
                NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
                NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
                NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
                NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();
                NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();NOP();                             
                bTemp >>= 1;
                NOP();NOP();
                
                #ifdef modeRepeat
                if(RA0)
                #else
                if(RB5)
                #endif
                {
                    bTemp |= 0x80;                    
                }
                else {
                    bTemp |= 0x00;                    
                }
                                                       
            }while(numBit--);
            
            #ifdef modeRepeat
            IOCAF0 = 0;
            #else
            IOCBF5 = 0;
            #endif
            
            arrToRX[numByteRX++] = bTemp;
            bTemp = PORTB;
        }                
        //только для чтения IOCIF, очищается автоматом, когда другие прерывания обнулены
    }
    
    #ifdef modeSW    
    if(IOCBF5){
        if(sound)
                clrbit(PORTB, bS);
            else
                setbit(PORTB, bS);
        IOCBF5 = 0;
    }
    #endif   
    
    if (RCIE && RCIF) {        
        if (RCSTAbits.OERR) {// || detect.b.readOk) {//ошибка переполнения или принятый пакет еще на анализе
            CREN = 0;
            NOP();
            CREN = 1;
        } else {//читаем байт
            arrToRX[numByteRX++] = RCREG;
            detect.b.UART = 1;
        }
    }
   
    if(detect.b.UART){
        detect.b.UART = 0;        
        switch (numByteRX) {
            case 1: {
                if (arrToRX[0] != 0x55) {//это не первый байт
                    numByteRX = 0;
                } else{                    
                    timeTactRead = 10; //максимальное время на прием всего пакета
                    detect.b.recData = 1;//идет прием данных
                }
                break;
            }
            case 2: {                
                if (arrToRX[1] != 0xAA)//это не второй байт
                    numByteRX = 0;
                break;
            }
                    /*case 3://текущая версия
                    case 4://команда
                    case 5:{//старший байт в количестве байт                                        
                        break;                    
                    }*/
            case 6: {//6 + 1
                allByteRX = 7 + arrToRX[5]; //общее количество байт в пакете
                timeTactRead += arrToRX[5] + 5;
                break;
            }
            default: {
                if (numByteRX == allByteRX) {//пакет принят полностью                    
                    detect.b.recData = 0;//таймер Т2 можно выключить
                    numByteRX = 0;                      
//------------------               
                    switch (arrToRX[3]) {//байт № 3 - команда
                        case 0: {//ПУЛЬС
                            if(sessionNum > 1)//ответ ПУЛЬС будет всегда, кроме одного события
                                //когда sessionNum = 1
                                sessionNum--;                            
                            //количество сигналов ПУЛЬС для текущего подключения блютуз
                            arrToTX[0] = 0x55;arrToTX[1] = 0xAA;arrToTX[2] = 0x00;
                            arrToTX[3] = 0x00;arrToTX[4] = 0x00;
                            arrToTX[5] = 0x01;
                            if (detect.b.firstOn) {//первое включение
                                arrToTX[6] = 0x00;
                                arrToTX[7] = 0x00;
                                detect.b.firstOn = 0;
                            }else {
                                arrToTX[6] = 0x01;
                                arrToTX[7] = 0x01;
                            }
                            allByteTX = 8;numByteTX = 0;
                            TXIE = 1; //разрешаем передачу
                            detect.b.readOk = 0;
                            synNum = 250;                             
                            break;
                        }
                        
                        default:{
                            detect.b.readOk = 1;//теперь надо анализировать                             
                        }
                    }                        
//--------------------
                }//if
            }//default
       }//switch
      //  if(detect.b.recData)//идет прием данных
        //    TMR2IE = 1;
    }//UART

    if (TXIE && TXIF) {
        if(numByteTX != allByteTX){//переданы все байты
            TXREG = arrToTX[numByteTX];
            numByteTX++;
        }else{
            if(TRMT)//есть проблема при передачи последних байт, поэтому вырубаем TXIE
                //только когда передан последний байт
                TXIE = 0;
        }
    } 
    
    return;
}
//------------------------------------------------------------------
void main(void) {
    unsigned char i;
    byte bTemp;
    byte pauseNumSens;    
    byte *p;
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
    
    #define ALLNUMPARAM 7//количество параметров + 1 которое передается на смартфон
    #define CONSTSIGNALON 80//40//
    #define CONSTCHECKINTERVAL 30
   
    byte check_interval[2];
    byte fDl;
    unsigned int dl;
    
    struct sSensorSW{
        unsigned int sampl;//текущее усредненное значение
        unsigned int level;//постоянный уровень        
    } sensSW[2];
    
    #define CONSTAKK    5     
    unsigned int filterS[CONSTAKK][2];//для фильтра НЧ
    unsigned int filterL[CONSTAKK * 2][2];
    unsigned char future_enabled[2];
    
    unsigned char iSampl, idUser, iLevel;
    
    unsigned int wTemp;
     
    //источник команды
    byte commandForMotor;
    #define cNoComm        0x00
    #define cBLEOpen       0x01//по блютусу открыть
    #define cSensSWOpen    0x02//сенсорная клавиатура открыть
    #define cBLEClose       0x03//по блютусу закрыть
    #define cSensSWClose    0x04//сенсорная клавиатура закрыть

    Initial();

    nWait = 0;
    arrToTX[0] = 0x55;
    arrToTX[1] = 0xAA;
    arrToTX[2] = 0x00; //версия
    arrToTX[4] = 0x00; //старший байт параметра "количество байт"
    
    Turn = 7;
//пока выключим    CPSON = 1;
    LED_OPEN = 1;
    LED_CLOSE = 0;
    
    cicleGo = 0;
    flag.all = 0;
    numByteRX = 0;
    detect.all = 0;
    cicleGo = 10;//250;//
    //swMove = ON;
    //sound = ON;
    do{
        wTemp = 40000;
        do{NOP();}while(wTemp--);
        LED_OPEN ^= 1;
        LED_CLOSE ^= 1;
        
    }while(cicleGo--);
    //sound = OFF;
    //swMove = OFF;
    LED_OPEN = 0;
    LED_CLOSE = 0;
    
    detect.b.firstOn = 1;
    detect.b.sensSWzero = 1;
    TMR2IE = 0;
    detect.b.checkBattery = 1;
    
    while (1) {        
        //пока выключим CPSON = 1;//включаем сенсорные кнопки
        #ifdef modeRepeat
    
        #else            
        if(sessionNum == 1){//рвем связь
            while (TXIE);            
            arrToTX[0] = 0x55;arrToTX[1] = 0xAA;arrToTX[2] = 0x00;
            arrToTX[3] = 0xE7;
            arrToTX[4] = 0x00;
            arrToTX[5] = 0x00;
            arrToTX[6] = 0xE6;                                                                
            allByteTX = 7;numByteTX = 0;
            TXIE = 1; //разрешаем передачу
        }    
        #endif                                                                        
                                
        if (detect.b.readOk) {//если есть принятый пакет, то анализируем его              
            //не плохо бы проверить контрольную сумму у принятого пакета            
            while (TXIE); //ждем когда завершится предыдущая передача
            arrToTX[4] = 0x00;
            switch (arrToRX[3]) {//байт № 3 - команда
                case 0x01: {//запрос кода продукции
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
                    arrToTX[14] = checkSum(14);//0x77;
                    allByteTX = 15;                                                
                    break;
                }
                case 0xE8: {//запрос версии контроллера
                    arrToTX[3] = 0xE8;
                    arrToTX[5] = 0x06;
                    //версия 100100
                    arrToTX[6] = 0x01;
                    arrToTX[7] = 0x00;
                    arrToTX[8] = 0x00;
                    arrToTX[9] = 0x01;
                    arrToTX[10] = 0x00;
                    arrToTX[11] = 0x00;
                    arrToTX[12] = checkSum(12);//0xEF;
                    allByteTX = 13;                    
                    break;
                }
                case 0xE7: {//ответ на запрос отключиться от блютуза
                    if(arrToRX[6] == 0x00){                        
                        sessionNum = 0;//отключение подтверждено
                    }                     
                    break;
                }
                case 0x02: {//запрос на определение контактов сброса и индикатор работы у модуля
                    arrToTX[3] = 0x02;
                    arrToTX[5] = 0x00;
                    arrToTX[6] = 0x01;
                    allByteTX = 7;
                    break;
                }
                case 0x03: {//сообщает рабочий статус модуля
                    //arrToRX[6]//значени статуса (0х00 - не привязан; 0х01 - привязан не подключен; 0х02 - привязан подключен)
                    if(arrToRX[6] == 0x02)//устройство привязано
                        sessionNum = 4;
                    arrToTX[3] = 0x03;
                    arrToTX[5] = 0x00;
                    arrToTX[6] = 0x02;
                    allByteTX = 7;
                    
                    break;
                }
                case 0x08: {//запрос статуса-настроек контроллера для передачи их на смартфон
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
                    arrToTX[14] = checkSum(14);
                    allByteTX = 15;
                    numParam = ALLNUMPARAM;
                    break;
                }
                case 0x07: {//подтверждение приема СТАТУС-ОТВЕТа
                    if (numParam) {//если есть не переданные параметры                    
                        switch (numParam) {
                            case 1: {//передача сведений об установленном языке интерфейса
                                arrToTX[3] = 0x07; //ответ на запрос статуса - настроек 
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x1C; //id язык замка 28
                                arrToTX[7] = 0x04; //перечисляемый тип данных
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //количество - один байт
                                arrToTX[10] = 0x07; //русский язык                                
                                arrToTX[11] = checkSum(11);
                                allByteTX = 12;
                                break;
                            }
                            case 2: {//состояние замка
                                arrToTX[3] = 0x07;
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x2F; //47 dp id статус замка
                                arrToTX[7] = 0x01; //тип данных boolean
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //один байта данных
                                arrToTX[10] = flag.b.direct; //состояния замка пусть определяется направлением
                                arrToTX[11] = checkSum(11);
                                allByteTX = 12;
                                break;
                            }
                            case 3: {//состояние замка
                                arrToTX[3] = 0x07;
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x30; //48 dp направление вращения двигателем
                                arrToTX[7] = 0x04; //тип данных
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //один байта данных
                                arrToTX[10] = flag.b.inverMov; //состояния замка пусть определяется направлением
                                arrToTX[11] = checkSum(11);
                                allByteTX = 12;
                                break;
                            }
                            case 4: {
                                arrToTX[3] = 0x07; //ответ на запрос статуса - настроек
                                arrToTX[5] = 0x05;
                                arrToTX[6] = 0x09; //id заряд аккумулятора в виде градаций
                                arrToTX[7] = 0x04; //перечисляемый тип данных
                                arrToTX[8] = 0x00;
                                arrToTX[9] = 0x01; //количество - один байт
                                arrToTX[10] = myLock.prop.levelPower;
                                //значение 0x00: высокий уровень заряда батареи,
                                //0x01: средняя батарея, 0x02: низкий уровень заряда батареи,
                                //0x03: батарея разряжена
                                arrToTX[11] = checkSum(11);
                                allByteTX = 12;
                                break;
                            }
                            default: {
                            }
                        }
                        numParam--;
                    } else{
                        allByteTX = 0;                    
                    }
                    break;
                }
                case 0x06: {//команда для замка
                    arrToTX[3] = 0x07; //команда подтверждения приема
                    switch (arrToRX[6]) {
                        case 0x06: {//команда открыть замок по Bluetooth, но для закрытия исползуется почему-то другая pdid46
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
                            arrToTX[12] = checkSum(12);
                            allByteTX = 13;
                            break;
                        }
                        case 0x2E: {//команда закрыть замок в ручную по Bluetooth pdid46
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
                            
                            arrToTX[11] = checkSum(11);
                            allByteTX = 12;
                            break;
                        }
                        case 0x30: {//команда направление вращения двигателем
                            arrToTX[5] = 0x05;
                            arrToTX[6] = 0x30;
                            arrToTX[7] = 0x04; //тип данных raw
                            arrToTX[8] = 0x00;
                            arrToTX[9] = 0x01; //количество - один байт
                            flag.b.inverMov = arrToRX[10];
                            arrToTX[10] = flag.b.inverMov;
                            arrToTX[11] = checkSum(11);
                            allByteTX = 12;
                            break;
                        }
                        case 0x1C: {//установить язык
                            break;
                        }
                        default: {
                        }
                    }
                    break;
                }
                case 0xE0: {
                    allByteTX = 0;
                    break;
                }
            }
            
            if (allByteTX) {//есть байты требующие передачу
                numByteTX = 0;                
                TXIE = 1; //разрешаем передачу
            }       
            switch (arrToRX[3]){
                case 0x00:
                case 0xE7: break;//отключаем выше
                default: {
                    sessionNum = 4;                    
                }
            }
                    
            detect.b.readOk = 0;
            
        }

        if (flag.b.swOn) {//пришла команта крутить двигатель
            CPSON = 0; //вЫключаем сенсорные кнопки
                         
            #ifdef modeWork
                    IOCBN5 = 0;
                    CREN = 1;
                    RCIF = 0;
                    RCIE = 1;                
            #endif
            while (TXIE);
            moveMotor();
            
            TMR2IE = 1;
           
            while(synNum > 200);
            synNum = 200;
            while (TXIE)
               NOP();
           
            CREN = 0;
            RCIE = 0;
            
            #ifdef modeRepeat
    
            #else            
            IOCBN5 = 1;
            #endif
            
            arrToTX[0] = 0x55;
            arrToTX[1] = 0xAA;
            arrToTX[2] = 0x00; //версия
            arrToTX[3] = 0x07;
            arrToTX[4] = 0x00;
            arrToTX[5] = 0x05;
            arrToTX[6] = 0x2F; //47 dp id статус замка
            arrToTX[7] = 0x01; //тип данных boolean
            arrToTX[8] = 0x00;
            arrToTX[9] = 0x01; //один байта данных
            arrToTX[10] = flag.b.direct; //состояния замка пусть определяется направлением
            arrToTX[11] = checkSum(11);
            allByteTX = 12;
            numByteTX = 0;
            TXIE = 1; //разрешаем передачу
            
   /*         
          while (TXIE);  
          arrToTX[3] = 0xE0; //команда записи во флэш
            arrToTX[5] = 0x06; //количество байт данных
            arrToTX[6] = 0x01; //время контролирует модуль            
            arrToTX[7] = 0x2F; //47 dp id статус замка
            arrToTX[8] = 0x01; //тип данных boolean
            arrToTX[9] = 0x00;
            arrToTX[10] = 0x01; //один байта данных
            arrToTX[11] = flag.b.direct; //состояния замка пусть определяется направлением                                                      
            arrToTX[12] = checkSum(12);
            allByteTX = 13;
            numByteTX = 0;
            TXIE = 1; //разрешаем передачу
            //while (TXIE);
   */
            //TMR2IE = 1;
            //nWait = 3;                        
            //while(nWait);//ждем завершения паузы
            
            //while(synNum > 200)
                //NOP();//чтобы сразу за командой ПУЛЬС
            while (TXIE)
                NOP();
            
            switch (commandForMotor){
                case cBLEOpen :{//замок открыт по  Bluetooth
                    //отправляем команду записать в журнал событие "замок открыт"
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
                    arrToTX[15] = checkSum(15);
                    allByteTX = 16;
                    numByteTX = 0;
                    TXIE = 1; //разрешаем передачу
                    break;
                }
            case cSensSWOpen :{//замок открыт с помощью сенсорной клавитуры
                    //отправляем команду записать в журнал предупреждений "замок открыт"
                    arrToTX[3] = 0xE0; //команда записи во флэш
                    arrToTX[5] = 0x06; //количество байт данных
                    arrToTX[6] = 0x01; //время контролирует модуль
                    arrToTX[7] = 0x15; //команда lock alarm
                    arrToTX[8] = 0x04; //тип данных enum
                    arrToTX[9] = 0x00;
                    arrToTX[10] = 0x01; //количество - 1 байта
                    arrToTX[11] = 0x09;//открыть сенсорной кнопкой
                    arrToTX[12] = checkSum(12);
                    allByteTX = 13;
                    numByteTX = 0;
                    TXIE = 1; //разрешаем передачу
                    break;
                }
            case cSensSWClose :{//замок закрыть с помощью сенсорной клавиатуры
                    //отправляем команду записать в журнал предупреждений "замок закрыт"
                    arrToTX[3] = 0xE0; //команда записи во флэш
                    arrToTX[5] = 0x06; //количество байт данных
                    arrToTX[6] = 0x01; //время контролирует модуль
                    arrToTX[7] = 0x15; //команда lock alarm
                    arrToTX[8] = 0x04; //тип данных enum
                    arrToTX[9] = 0x00;
                    arrToTX[10] = 0x01; //количество - 1 байта
                    arrToTX[11] = 0x0A;//закрыть сенсорной кнопкой
                    arrToTX[12] = checkSum(12);
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
            CPSON = 0;//вЫключаем сенсорные кнопки
            ADIF = 0;
            ADON = 1;
            //настраиваем канал АЦП AN7
            CHS0 = 1; CHS1 = 1; CHS2 = 1;
            //подключаем делитель
            onBAT = ON;
            nWait = 4;
            wADC.w = 0;//valuePowerADC = 0;
            TMR2IE = 1;
            
            while(!wADC.w);//while(!valuePowerADC);
            
            wADC.w = 0;//valuePowerADC = 0;
            while(!wADC.w);//while(!valuePowerADC);//измеряем два раза на всякий случай
            //wADC.w = valuePowerADC;
            
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
           //выключаем делитель
            onBAT = OFF;
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
                arrToTX[12] = checkSum(12);
                allByteTX = 13;
                numByteTX = 0;
                TXIE = 1;
            }
            #ifdef modeWork           
                //пока выключим CPSON = 1;//включаем сенсорные кнопки
            #endif
            detect.b.checkBattery = 0;
            nWait = 10;
            detect.b.sensSWzero = 1;
            while(nWait);//ждем завершения паузы
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
            pauseNumSens = 2 * CONSTAKK;
        }
        //анализ сенсорных кнопок
        /*for (i = 0; i < 2; i++) {        
            if (fDl[i] == 0) {
                continue;
            }
            fDl[i] = 0;            
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
            filterS[iSampl][i] = dl[i];
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
            filterL[iLevel][i] = dl[i];
            sensSW[i].level += filterL[iLevel][i];
            
            wTemp = sensSW[i].level;
            wTemp >>= 1;
            
            if(!pauseNumSens){
            //сравниваем уровень и мгновенное значение
                if(wTemp > sensSW[i].sampl){
                    if ((wTemp - sensSW[i].sampl) > CONSTSIGNALON) {                                            
                        check_interval[i] = CONSTCHECKINTERVAL; 
                        if(i) RA1 = 1; else RA0 = 1;
                    }   
                }
            } else {
                pauseNumSens--;
            }
                    
            if(check_interval[i] > 0) {
                bTemp = i;
                bTemp ^= 1;
                bTemp &= 0b1;
                if(check_interval[bTemp] > 0){
                    future_enabled[i] = 0;//на текущее срабатывание кнопки запрещаем вращение
                }
            
                if(check_interval[i] == 1){                                
                    if(future_enabled[i]) {//вращение разрешено
                        if(i) {//открыть, а при инверсии закрыть
                        //    LED_OPEN = 1 ^ flag.b.inverMov;                        
                        //    LED_CLOSE = 0 ^ flag.b.inverMov;
                            flag.b.direct = 1 ^ flag.b.inverMov;
                        } else{//закрыть, а при инверсии открыть                                                            
                        //    LED_OPEN = 0 ^ flag.b.inverMov;
                        //    LED_CLOSE = 1 ^ flag.b.inverMov;
                            flag.b.direct = 0 ^ flag.b.inverMov;
                        }
                        flag.b.swOn = 1;                        
                        if(flag.b.direct)
                            commandForMotor = cSensSWOpen;
                        else
                            commandForMotor = cSensSWClose;
                    }                     
                }
                check_interval[i]--;                        
            } else {
                if(i) RA1 = 0; else RA0 = 0;
                future_enabled[i] = 1;//потенциально разрешаем срабатывание кнопки
            }
        }*/
        
        if (fDl == 1) {                
            fDl = 0;            
            //НЧ фильтрация
            if(numCh == 1){
                bTemp = CONSTAKK - 1;
                if(iSampl < bTemp){
                    iSampl++;
                }else{
                    iSampl = 0;
                }
            }
            sensSW[numCh].sampl -= filterS[iSampl][numCh];
            filterS[iSampl][numCh] = dl;
            sensSW[numCh].sampl += filterS[iSampl][numCh];
            
            if(numCh == 1){
                bTemp = 2 * CONSTAKK - 1;
                if(iLevel < bTemp){
                    iLevel++;
                }else{
                    iLevel = 0;
                }
            }
            sensSW[numCh].level -= filterL[iLevel][numCh];
            filterL[iLevel][numCh] = dl;
            sensSW[numCh].level += filterL[iLevel][numCh];
            
            wTemp = sensSW[numCh].level;
            wTemp >>= 1;
            
            if(!pauseNumSens){
            //сравниваем уровень и мгновенное значение
                if(wTemp > sensSW[numCh].sampl){
                    if ((wTemp - sensSW[numCh].sampl) > CONSTSIGNALON) {                                            
                        check_interval[numCh] = CONSTCHECKINTERVAL;  
                    }   
                }
            } else {
                pauseNumSens--;
            }
                    
            if(check_interval[numCh] > 0) {                
                if(check_interval[numChNew] > 0){
                    future_enabled[numCh] = 0;//на текущее срабатывание кнопки запрещаем вращение
                }
            
                if(check_interval[numCh] == 1){                                
                    if(future_enabled[numCh]) {//вращение разрешено
                        if(numCh) {//закрыть, а при инверсии открыть                                                            
                        //    LED_OPEN = 0 ^ flag.b.inverMov;
                        //    LED_CLOSE = 1 ^ flag.b.inverMov;
                            flag.b.direct = 0 ^ flag.b.inverMov;
                        } else{//открыть, а при инверсии закрыть
                        //    LED_OPEN = 1 ^ flag.b.inverMov;                        
                        //    LED_CLOSE = 0 ^ flag.b.inverMov;
                            flag.b.direct = 1 ^ flag.b.inverMov;                            
                        }
                        flag.b.swOn = 1;                        
                        if(flag.b.direct)
                            commandForMotor = cSensSWOpen;
                        else
                            commandForMotor = cSensSWClose;
                    }                     
                }
                check_interval[numCh]--;                        
            } else {
 //               if(numCh) RA1 = 0; else RA0 = 0;
                future_enabled[numCh] = 1;//потенциально разрешаем срабатывание кнопки
            }
        }
         
        if(!detect.b.recData && !nWait)
            TMR2IE = 0;
        
        if(TMR2IE || TXIE || IOCAF5 || flag.b.swOn)
            continue;
        
//только UART        IOCBN5 = 1;  
        TMR1L = 0;
        TMR1H = 0;
#ifdef modeWork        
        TMR1ON = 1;        
        SWDTEN = 1;
        SLEEP();
#endif
        SWDTEN = 0;
        TMR1ON = 0;
        if(!STATUSbits.nTO){
            TMR1ON = 0;
            numCh = numChNew;
            p = (unsigned char *)&dl;
            *p = TMR1L;
            p++;
            *p = TMR1H;
            //fDl[numCh] = 1;
            fDl = 1;
            //numCh ^= 1;
            //numCh &= 0b1;
            //CPSCON1 = setCh[numCh]; //устанавливаем канал 
            numChNew ^= 1;
            numChNew &= 0b1;
            CPSCON1 = setCh[numChNew]; //устанавливаем канал 
        }
    }
}