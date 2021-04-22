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

#define CONSTPOROG 47//��� ������� 0,2 ��, �������� ��� ���� 0,5 � ���������� 0,1 �,
//����� ��� ��� 10 �������� � ������� ���������� 2,048 � ��� ��� 0,1/2,048*1024 = 50

//CPS1(CPS4) RC0 pin 16, CPS2(CPS5) RC1 pin 15
const unsigned char setCh[2] ={4, 5};
//��� ��� �������, ������� ����� 8 ��, ������� ������������� 210 ��
//const 

#define stDevTurn       1
#define stRevers        2

#define errRevers       1
#define errHalf1        2
#define errHalf2        3
#define errHalf3        4


volatile byte nHalfTurn;//���������� ������������ ����������� ������� = ���������� ������������

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

byte numByteRX;//����� ������������ �����
byte allByteRX;//���������� ����, ������� ������ ���� � �������� ������
byte arrToRX[20];
volatile byte timeTactRead;//���������� ����� �� ����� ����� ������

byte arrToTX[20];
volatile byte allByteTX;//���������� �����, ������� ���������� ��������
volatile byte numByteTX;//����� ������������� �����
byte numParam;//���������� ����������, ��������� �������� �� ��������

#define CONSTHEIGHT 396//3,6 �  0x0100+  35d<<2
#define CONSTMEDIUM 372//3,4 �  0x0100+  29d<<2
#define CONSTLOW    352//3,2 �  0x0100+  24d<<2 ��������, ����� �������� 1 � ��� ������� 2,048 � � 10 ��������� ���
//3.1 21
//3.0 19
volatile unsigned int valuePowerADC;//�������� ���� ��� ��� ��������� ������ �������
word timePower;//������ �������� ������ ������������
volatile byte LD1, LD2;

volatile union{
        unsigned int num;
        byte b[2];
} numRep;

volatile union{
    byte all;
    struct flag1
    {
        unsigned swOn: 1;//������ ������
        unsigned ready : 1;//������ ���������
        unsigned inverMov: 1;//�������� ��������
        unsigned motorMove: 1;//��������� ������ 1 - ��������, 0 - �� ��������
        unsigned direct: 1;//����������� �������� ������ 0 - �������, 1 - �������        
        unsigned currBig: 1;//�������� ���� ���������
        unsigned reperPos: 1;//��������� �� ������
        unsigned blink: 1;//��������� ������� �����������
    }b;
}flag;

volatile union{
    byte all;
    struct flag2
    {
        unsigned timeOut: 1;//����� �������
        unsigned readOk: 1;//����� ������
        unsigned firstOn : 1;//������ ����� �� �����
        unsigned checkBattery : 1;//��������� ����� ������������
        unsigned sensSWzero : 1;//������� ��������� ����������
    }b;
}detect;