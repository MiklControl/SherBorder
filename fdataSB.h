//-------------------------------------------------
#define byte unsigned char
#define word unsigned int

#define OUTPUT    0
#define INPUT      1
#define ON          1
#define OFF      !ON

#define LED_OPEN_DIR    TRISC2
#define LED_OPEN        RC2
#define bLED_OPEN   2
#define LED_CLOSE_DIR   TRISA2
#define LED_CLOSE       RA2
#define bLED_CLOSE  2    

#define IN1_DIR   TRISC5//pin 5
#define IN1       RC5
#define IN2_DIR   TRISC4//pin 6
#define IN2       RC4
#define bIN2       4
#define bIN1       5

#define swMove_DIR  TRISC7//pin 9
#define swMove      RC7
#define swConf_DIR  TRISB6//pin 11
#define swConf      RB6
#define sensCurr_DIR  TRISA4//pin 3
#define sensCurr      RA4
#define sensOpto_DIR  TRISA5//pin 2
#define sensOpto      RA5
#define sound_DIR  TRISB4//pin 13
#define sound      RB4
#define onBAT_DIR  TRISC6//pin 8
#define onBAT      RC6

#define CONSTPOROG 47//для датчика 0,2 Ом, получаем при токе 0,5 А напряжение 0,1 В,
//тогда для АЦП 10 разрядов и опорном напряжении 2,048 В это код 0,1/2,048*1024 = 50

//CPS1(CPS4) RC0 pin 16, CPS2(CPS5) RC1 pin 15
const unsigned char setCh[2] ={4, 5};
//для КИХ фильтра, частота среза 8 Гц, частота дискретизации 210 Гц
//const 

#define stDevTurn       1
#define stRevers        2

#define errRevers       1
#define errHalf1        2
#define errHalf2        3
#define errHalf3        4


volatile byte nHalfTurn;//количество срабатываний оптического датчика = количество полуоборотов

byte numCh;
volatile unsigned int dl[3];
volatile unsigned fDl[3];

volatile unsigned int nWait, nWaitS;
volatile byte stat;

unsigned int st;


byte comm;
byte cicleGo;
byte Turn;
byte Status;
int numHighCurrent;

#define CONSTBIG 5

byte numByteRX;//номер принимаемого байта
byte allByteRX;//количество байт, которое должно быть в принятом пакете
byte arrToRX[20];
volatile byte timeTactRead;//оставшееся время на прием всего пакета

byte arrToTX[20];
volatile byte allByteTX;//количество байта, которое необходимо передать
volatile byte numByteTX;//номер передаваемого байта
byte numParam;//количество параметров, требующих отправки на смартфон

#define CONSTHEIGHT 396//3,6 В  0x0100+  35d<<2
#define CONSTMEDIUM 372//3,4 В  0x0100+  29d<<2
#define CONSTLOW    352//3,2 В  0x0100+  24d<<2 исходное, после делителя 1 В при питании 2,048 В и 10 разрядном АЦП
//3.1 21
//3.0 19
volatile unsigned int valuePowerADC;//значение кода АЦП при измерении заряда батареи
word timePower;//таймер проверки заряда аккумулятора
volatile byte LD1, LD2;

volatile union{
        unsigned int num;
        byte b[2];
} numRep;

volatile union{
    byte all;
    struct flag1
    {
        unsigned swOn: 1;//кнопка нажата
        unsigned ready : 1;//данные актуальны
        unsigned inverMov: 1;//инверсия вращения
        unsigned motorMove: 1;//состояние мотора 1 - работает, 0 - не работает
        unsigned direct: 1;//направление вращения мотора 0 - закрыть, 1 - открыть        
        unsigned currBig: 1;//значение тока превышено
        unsigned reperPos: 1;//положение на репере
        unsigned blink: 1;//разрешить мигание светодиодом
    }b;
}flag;

volatile union{
    byte all;
    struct flag2
    {
        unsigned timeOut: 1;//время истекло
        unsigned readOk: 1;//пакет принят
        unsigned firstOn : 1;//первый ответ на ПУЛЬС
        unsigned checkBattery : 1;//проверить заряд аккумулятора
        unsigned sensSWzero : 1;//обнулиь сенсорную клавиатуру
    }b;
}detect;