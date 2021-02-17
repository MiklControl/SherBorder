//-------------------------------------------------
#define byte unsigned char
#define word unsigned int

#define OUTPUT    0
#define INPUT      1
#define ON          1
#define OFF      !ON

#define LED_LEFT_DIR    TRISA0
#define LED_LEFT        RA0
#define LED_RIGHT_DIR   TRISA1
#define LED_RIGHT       RA1

#define IN1_DIR   TRISC5//pin 5
#define IN1       RC5
#define IN2_DIR   TRISC4//pin 6
#define IN2       RC4
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

#define CONSTPOROG 70//24 ��� ������� 0,2 ��, �������� ��� ���� 0,5 � ���������� 0,1 �,
//����� ��� ��� 10 �������� � ������� 3,3 � ��� ��� 0,1/3,3*1024=31

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


byte nHalfTurn;//���������� ������������ ����������� ������� = ���������� ������������

byte numCh;
unsigned int dl[3];
unsigned fDl[3];
unsigned int *p;
unsigned int nWait, nWaitS;
byte stat;

byte comm;
byte cicleGo;
byte Turn;
byte Status;
int numBig;

#define CONSTBIG 20

byte numByteRX;//����� ������������ �����
byte allByteRX;//���������� ����, ������� ������ ���� � �������� ������
byte arrToRX[20];
byte timeTactRead;//���������� ����� �� ����� ����� ������

byte arrToTX[20];
byte allByteTX;//���������� �����, ������� ���������� ��������
byte numByteTX;//����� ������������� �����
byte numParam;//���������� ����������, ��������� �������� �� ��������

union{
        unsigned int num;
        byte b[2];
} numRep, wValADC;
    
unsigned int arrAkk[16][3];

union{
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
        unsigned tong: 1;//������� ����� �������
    }b;
}flag;

union{
    byte all;
    struct flag2
    {
        unsigned timeOut: 1;//����� �������
        unsigned readOk: 1;//����� ������
        unsigned firstOn : 1;//������ ����� �� �����
    }b;
}detect;