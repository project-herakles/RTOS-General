typedef struct
{
  unsigned short int a   : 1;
  unsigned short int b   : 2;
  unsigned short int prev: 2;
}shoot_mode;

typedef struct
{
  unsigned short int a    :1;
  unsigned short int b1   :1;
  unsigned short int b2   :1;
  unsigned short int Shoot:1;
  unsigned short int TimeO:1;
}OneBit_t;

typedef enum
{
  Nil=0, Semi=1, Burst=2, Auto=3
}MODE;

typedef enum
{
  fire=1, wait=0
}shoot_t;

void shootTask(shoot_mode * mode, unsigned short int sig, unsigned int * ctime);
shoot_t shootCtrl(unsigned short int instru, unsigned short int * fireMode, unsigned short int * fireInst, unsigned short int * n);
int setPIDRef(unsigned short int cmd, unsigned short int prev_cmd);
