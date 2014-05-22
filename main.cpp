#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <stdlib.h>
#include <iomanip>
#include <iostream>
#include <mosquitto.h>
#include <cmath>
#include<fstream>

using namespace std;

char clientid[24]="Vali e frumos"; //modificat in functie de numele echipei
struct mosquitto *mosq;

#define mqtt_host "192.168.0.100"  //adresa ip a serverului
#define mqtt_port 1883
#define PI 3.14159265358979232846

struct RobotControl{
    int left;
    int right;
    int time;
    int left_rotation;
    int right_rotation;
};

struct RobotCoords {
    int id; // id robot - 1..10
    int x;  // 0...800  - relative la terenul de joc
    int y;  // 0...600
    int angle; // unghi fata de baza ringului
    int timestamp; // timpul cand au fost calculate - UNIXTIME - Ex: 1352460922
};


struct RobotControl control[16];
struct RobotCoords  coords[16],past[1],past2[1],past3[1];

static int run = 1;

//char robotId[] = "/r2";  //reprezinta numarul robotului
//unsigned int rId = 2;  //o sa aveti nevoie de id-ul robotului si sub forma de int
int josx=28,
    susx=1368,
    stangay=283,
    dreaptay=417;

int tinta_stangay=stangay,tinta_dreaptay=dreaptay,tinta_x=susx,
defense_dreapta=515,
defense_stanga=184,
defense_x=214,
atac_x=abs(tinta_x-defense_x),
aparare_x=abs(abs(1400-tinta_x)-defense_x),
linie_atac=(tinta_x+atac_x)/2,
linie_aparare=(abs(1400-tinta_x)+aparare_x)/2;

int danger=39,danger_universal=1359,extreme_danger=-26,extreme_danger_universal=1429;
        //A SE MODIFICA IN FUNCTIE DE PARTEA TERENULUI
int left_side=-4,right_side=705,upper_side=1341,down_side=48;
        //A SE CALIBRA DACA NU CORESPUND (se ia fix pozitia robotului)
int poarta_jos_x=49,poarta_sus_x=1322,poarta_stanga_y=275,poarta_dreapta_y=449,
poarta_jos_predictie=33,poarta_sus_predictie=1370,poarta_stanga_predictie=279,poarta_dreapta_predictie=425;

int o1=3,o2=5;
char id_o1[] = "/r3",id_o2[]="/r5";
int d1=16,d2=7;
char id_d1[]="/r16",id_d2[]="/r7";
int p=11;
char id_p[]="/r11";
int e1=3,e2=7,e3=2,e4=3,e5=7;
int latimer=48,diametrur=66;
int razab=15,diametrub=30;
int distantax=0,distantay=0;
struct timeval  tv;

int target_x=atac_x;
int target_y=(tinta_dreaptay+tinta_stangay)/2;

int timp=0,timp2=0;
int k1=0,k2=0;
int eroare=1;

int reset=0,reset_sus=1385,reset_jos=18;

int po1x=536,po1y=423,po2x=531,po2y=282,pd1x=238,pd1y=520,pd2x=238,pd2y=190,ppx=80,ppy=350;
int of1,of2,def1,def2,port;

//ofstream g("rambo.out");
//ofstream h("matrice.out");

int z[150][80],v[150][80];
int a[10000];

void memorare()
{
    of1=o1;
    of2=o2;
    def1=d1;
    def2=d2;
    p=port;
}

void init()
{
    past3[0].id = 0;
    past3[0].x  = 0;
    past3[0].y  = 0;
    past3[0].angle = 0;
    past3[0].timestamp = 0;

    past2[0].id = 0;
    past2[0].x  = 0;
    past2[0].y  = 0;
    past2[0].angle = 0;
    past2[0].timestamp = 0;

    past[0].id = 0;
    past[0].x  = 0;
    past[0].y  = 0;
    past[0].angle = 0;
    past[0].timestamp = 0;

    coords[0].id = 0;
    coords[0].x  = 0;
    coords[0].y  = 0;
    coords[0].angle = 0;
    coords[0].timestamp = 0;
}

void connect_callback(struct mosquitto *mosq, void *obj, int result)
{
}

void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
    struct RobotCoords *coordonate = (struct RobotCoords *)message->payload;

    if (coordonate->id == 0)
    {
        if (coordonate->x != 0 && coordonate->y != 0)
        {
            past3[0].id = past2[0].id;
            past3[0].x  = past2[0].x;
            past3[0].y  = past2[0].y;
            past3[0].angle = past2[0].angle;
            past3[0].timestamp = past2[0].timestamp;

            past2[0].id = past[0].id;
            past2[0].x  = past[0].x;
            past2[0].y  = past[0].y;
            past2[0].angle = past[0].angle;
            past2[0].timestamp = past[0].timestamp;

            past[0].id = coords[0].id;
            past[0].x  = coords[0].x;
            past[0].y  = coords[0].y;
            past[0].angle = coords[0].angle;
            past[0].timestamp = coords[0].timestamp;

            coords[coordonate->id].id = coordonate->id;
            coords[coordonate->id].x  = coordonate->x;
            coords[coordonate->id].y  = coordonate->y;
            coords[coordonate->id].angle = coordonate->angle;
            coords[coordonate->id].timestamp = coordonate->timestamp;
        }
    }
    else
    {
        coords[coordonate->id].id = coordonate->id;
        coords[coordonate->id].x  = coordonate->x;
        coords[coordonate->id].y  = coordonate->y;
        coords[coordonate->id].angle = coordonate->angle;
        coords[coordonate->id].timestamp = coordonate->timestamp;
    }
    //cout<<"Id: "<<coordonate->id<<"Coords("<<coordonate->x<<":"<<coordonate->y<<")"<<"Angle:"<<coordonate->angle<<endl; //in loc de cout pentru a afisa in consola ca pe eroare
}

void publish(int id);



double GetAngle(RobotCoords R, RobotCoords B)
{
    if ((B.x - R.x) != 0)
        return atan((float)(B.y - R.y)/(float)(B.x - R.x)) *180/PI;
    else
        return -1;
}

double unghi_tinta(RobotCoords R, RobotCoords B)
{
    double angle, Angle;

    angle=GetAngle(R,B);
    if(R.x<B.x)
    {
        if(R.y<B.y) Angle=270+(90-angle);
        if(R.y>B.y) Angle=abs(angle);
    }
    if(R.x>B.x)
    {
        if(R.y<B.y) Angle=180+abs(angle);
        if(R.y>B.y) Angle=90+(90-angle);
    }
    return Angle;
}

int getspeed(double x)
{
    //mers drept
    double s=abs(x);
    int speed=0;
    //printf("%f \n",s);
    if(s>170) speed=140;
    if(s>140&&s<=170) speed=80;
    if(s>110&&s<=140) speed=80;
    if(s>80&&s<=110) speed=70;
    if(s>70&&s<=80) speed=70;
    if(s>60&&s<=70) speed=57;
    if(s>50&&s<=60) speed=55;
    if(s>40&&s<=50) speed=45;
    //if(s>30&&s<=40) speed=40;

    if(s>30&&s<=40) speed=25;
    if(s>25&&s<=30) speed=25;
    if(s>20&&s<=25) speed=25;
    if(s>15&&s<=20) speed=25;
    if(s>10&&s<=15) speed=15;
    if(s>5&&s<=10) speed=10;
    if(s>3&&s<=5) speed=7;
    if(s>1&&s<=3) speed=5;
    return speed;
}

int getspeed2(double x)
{
    //intoarcere
    double s=abs(x);
    int speed=0;
    if(s>170) speed=100;
    if(s>140&&s<=170) speed=90;
    if(s>110&&s<=140) speed=80;
    if(s>80&&s<=110) speed=70;
    if(s>70&&s<=80) speed=50;
    if(s>60&&s<=70) speed=45;
    if(s>50&&s<=60) speed=40;
    if(s>40&&s<=50) speed=35;
    if(s>30&&s<=40) speed=30;

    //if(s>30&&s<=40) speed=30;
    if(s>25&&s<=30) speed=30;
    if(s>20&&s<=25) speed=30;
    if(s>15&&s<=20) speed=12;
    if(s>10&&s<=15) speed=12;
    if(s>5&&s<=10) speed=10;
    if(s>3&&s<=5) speed=7;
    if(s>1&&s<=3) speed=5;
    return speed;
}

int getspeed3(double x)
{
    //mers drept
    double s=abs(x);
    int speed=0;
    //printf("%f \n",s);
    if(s>170) speed=140;
    if(s>140&&s<=170) speed=80;
    if(s>110&&s<=140) speed=80;
    if(s>80&&s<=110) speed=70;
    if(s>70&&s<=80) speed=70;
    if(s>60&&s<=70) speed=57;
    if(s>50&&s<=60) speed=55;
    if(s>40&&s<=50) speed=45;
    //if(s>30&&s<=40) speed=40;

    if(s>30&&s<=40) speed=10;
    if(s>25&&s<=30) speed=10;
    if(s>20&&s<=25) speed=10;
    if(s>15&&s<=20) speed=10;
    if(s>10&&s<=15) speed=10;
    if(s>5&&s<=10) speed=10;
    if(s>3&&s<=5) speed=7;
    if(s>1&&s<=3) speed=5;
    return speed;
}

void intoarcere_pozitiva(double A,double a,int id)
{
    int speed=0;
    double x,y;
    x=A-a;
    y=360-A+a;
    if(x<y)
    {
        speed=getspeed(x);
        {
                    //control[id].left = 0;
                    //control[id].right = 70;
                    //control[id].time = 0;
                    //control[id].left_rotation = 0;
                    //control[id].right_rotation = 2;
                    //mosquitto_publish(mosq, &mid, r8, sizeof(struct RobotControl), &control[r8id], 0, false);
        control[id].right=control[id].right+speed;
        }
    }
    else
    {
        speed=getspeed(y);
        control[id].left=control[id].left+speed;
    }

}

void intoarcere_pozitiva2(double A,double a,int id)
{
    int speed=0;
    double x,y;
    x=A-a;
    y=360-A+a;
    if(x<y)
    {
        speed=getspeed2(x);
        control[id].right=control[id].right+speed;
        control[id].left=control[id].left-speed;
    }
    else
    {
        speed=getspeed2(y);
        control[id].left=control[id].left+speed;
        control[id].right=control[id].right-speed;
    }

}

void intoarcere_pozitiva3(double A,double a,int id)
{
    int speed=0;
    double x,y;
    x=A-a;
    y=360-A+a;
    if(x<y)
    {
        speed=getspeed(x);
        {
                    //control[id].left = 0;
                    //control[id].right = 70;
                    //control[id].time = 0;
                    //control[id].left_rotation = 0;
                    //control[id].right_rotation = 2;
                    //mosquitto_publish(mosq, &mid, r8, sizeof(struct RobotControl), &control[r8id], 0, false);
        control[id].right=control[id].right-speed;
        }
    }
    else
    {
        speed=getspeed(y);
        control[id].left=control[id].left-speed;
    }

}

void intoarcere_pozitiva4(double A,double a,int id)
{
    int speed=0;
    double x,y;
    x=A-a;
    y=360-A+a;
    if(x<y)
    {
        speed=getspeed2(x);
        control[id].right=control[id].right-speed;
        control[id].left=control[id].left+speed;
    }
    else
    {
        speed=getspeed2(y);
        control[id].left=control[id].left-speed;
        control[id].right=control[id].right+speed;
    }

}

void intoarcere_negativa(double A,double a,int id)
{
    int speed=0;
    double x,y;
    x=A-a;
    y=a-360-A;
    if(x>y)
    {
        speed=getspeed(x);
        control[id].left=control[id].left+speed;
    }
    else
    {
        speed=getspeed(y);
        control[id].right=control[id].right+speed;
    }

}


void intoarcere_negativa2(double A,double a,int id)
{
    int speed=0;
    double x,y;
    x=A-a;
    y=a-360-A;
    if(x>y)
    {
        speed=getspeed2(x);
        control[id].left=control[id].left+speed;
        control[id].right=control[id].right-speed;
    }
    else
    {
        speed=getspeed2(y);
        control[id].right=control[id].right+speed;
        control[id].left=control[id].left-speed;
    }

}

void intoarcere_negativa3(double A,double a,int id)
{
    int speed=0;
    double x,y;
    x=A-a;
    y=a-360-A;
    if(x>y)
    {
        speed=getspeed(x);
        control[id].left=control[id].left-speed;
    }
    else
    {
        speed=getspeed(y);
        control[id].right=control[id].right-speed;
    }

}

void intoarcere_negativa4(double A,double a,int id)
{
    int speed=0;
    double x,y;
    x=A-a;
    y=a-360-A;
    if(x>y)
    {
        speed=getspeed2(x);
        control[id].left=control[id].left-speed;
        control[id].right=control[id].right+speed;
    }
    else
    {
        speed=getspeed2(y);
        control[id].right=control[id].right-speed;
        control[id].left=control[id].left+speed;
    }

}

void decizie(double A,double a,int id)
{
    if(A-a>1) intoarcere_pozitiva(A,a,id);
    if(A-a<-1) intoarcere_negativa(A,a,id);
}

void decizie2(double A,double a,int id)
{
    if(A-a>1) intoarcere_pozitiva2(A,a,id);
    if(A-a<-1) intoarcere_negativa2(A,a,id);
}

void decizie3(double A,double a,int id)
{
    if(A-a>1) intoarcere_pozitiva3(A,a,id);
    if(A-a<-1) intoarcere_negativa3(A,a,id);
}

void decizie4(double A,double a,int id)
{
    if(A-a>1) intoarcere_pozitiva4(A,a,id);
    if(A-a<-1) intoarcere_negativa4(A,a,id);
}

double getunghi(double x)
{
    double s=abs(x);
    return s;
}

double calculeaza_unghi(double A,double a)
{
    double unghi=0;
    double x,y;
    if(A-a>1)
    {

        x=A-a;
        y=360-A+a;
        if(x<y)
        {
            unghi=getunghi(x);

        }
        else
        {
            unghi=getunghi(y);

        }
    }
    if(A-a<-1)
    {

        x=A-a;
        y=a-360-A;
        if(x>y)
        {
            unghi=getunghi(x);

        }
        else
        {
            unghi=getunghi(y);

        }
    }
    return unghi;
}

void mergi_drept(double A, double a,int id)
{
    k2=0;
    control[id].left = 0;
    control[id].right = 0;
    decizie(A,a,id);
    if(k1==0)
    {
        control[id].left +=30;
        control[id].right += 30;
        k1++;
    }
    if(k1==1)
    {
        control[id].left +=40;
        control[id].right += 40;
        k1++;
    }
    if(k1==2)
    {
        control[id].left +=50;
        control[id].right += 50;
        k1++;
    }
    if(k1>2&&k1<5)
    //if(k1==3)
    {
        control[id].left +=60;
        control[id].right += 60;
        k1++;
    }
    if(k1>4&&k1<7)
    //if(k1==4)
    {
        control[id].left +=70;
        control[id].right += 70;
        k1++;
    }
    if(k1>6&&k1<10)
    //if(k1==5)
    {
        control[id].left +=80;
        control[id].right += 80;
        k1++;
    }
    if(k1>9&&k1<13)
    //if(k1==6)
    {
        control[id].left +=90;
        control[id].right += 90;
        k1++;
    }
    if(k1>12&&k1<16)
    //if(k1==7)
    {
        control[id].left +=100;
        control[id].right += 100;
        k1++;
    }
    if(k1>15&&k1<19)
    //if(k1==8)
    {
        control[id].left +=100;
        control[id].right += 100;
    }
    control[id].right_rotation=16;
    control[id].left_rotation=16;

}

void mergi_cu_spatele(double A, double a,int id)
{
    k1=0;
    control[id].left = 0;
    control[id].right = 0;
    decizie3(A,a,id);
    if(k2==0)
    {
        control[id].left -=30;
        control[id].right -= 30;
        k2++;
    }
    if(k2==1)
    {
        control[id].left -=40;
        control[id].right -= 40;
        k2++;
    }
    if(k2==2)
    {
        control[id].left -=40;
        control[id].right -= 40;
        k2++;
    }
    if(k2>2&&k2<5)
    //if(k==3)
    {
        control[id].left -=50;
        control[id].right -= 50;
        k2++;
    }
    if(k2>4&&k2<7)
    //if(k==4)
    {
        control[id].left -=50;
        control[id].right -= 50;
        k2++;
    }
    if(k2>6&&k2<10)
    //if(k==5)
    {
        control[id].left -=60;
        control[id].right -= 60;
        k2++;
    }
    if(k2>9&&k2<13)
    //if(k==6)
    {
        control[id].left -=60;
        control[id].right -= 60;
        k2++;
    }
    if(k2>12&&k2<16)
    //if(k==7)
    {
        control[id].left -=70;
        control[id].right -= 70;
        k2++;
    }
    if(k2>15&&k2<19)
    //if(k==8)
    {
        control[id].left -=70;
        control[id].right -= 70;
    }
    control[id].right_rotation=16;
    control[id].left_rotation=16;

}

void intoarcere(double A,double a,int id)
{
    k1=0;
    k2=0;
    control[id].left = 0;
    control[id].right = 0;
    decizie2(A,a,id);
    control[id].right_rotation=1;
    control[id].left_rotation=1;

}

void intoarcere_cu_spatele(double A,double a,int id)
{
    k1=0;
    k2=0;
    control[id].left = 0;
    control[id].right = 0;
    decizie4(A,a,id);
    control[id].right_rotation=1;
    control[id].left_rotation=1;

}

float distance_between_points(RobotCoords a,RobotCoords b)
{
    return sqrt(((b.x-a.x)*(b.x-a.x))+((b.y-a.y)*(b.y-a.y)));
}

int test_if_within_circle(RobotCoords a,RobotCoords b,int radius)
{
    int d;
    d=distance_between_points(a,b);
    if(d<radius)
    {return 1;printf("ESTE!!");}
    return 0;
}

float distanta_minima(RobotCoords punct)
{
    float d=32000;
    if(distance_between_points(punct,coords[e1])<d) d=distance_between_points(punct,coords[e1]);
    if(distance_between_points(punct,coords[e2])<d) d=distance_between_points(punct,coords[e2]);
    if(distance_between_points(punct,coords[e3])<d) d=distance_between_points(punct,coords[e3]);
    if(distance_between_points(punct,coords[e4])<d) d=distance_between_points(punct,coords[e4]);
    if(distance_between_points(punct,coords[e5])<d) d=distance_between_points(punct,coords[e5]);
    return d;
}

int points_on_the_line(RobotCoords x,RobotCoords y)
{
    int k=0;
    RobotCoords punct;
    float d;
    int m,n;
    float ax,ay,bx,by;
    ax=x.x;
    ay=x.y;
    bx=y.x;
    by=y.y;
    d=distance_between_points(x,y);
    while(m!=y.x||n!=y.y)
    {
        ax=ax+((y.x-x.x)/d);
        m=(int)round(ax);
        //printf("x= %d ",m);
        ay=ay+((y.y-x.y)/d);
        n=(int)round(ay);
        //printf("y= %d\n",n);
        punct.x=m;
        punct.y=n;
        if(!test_if_within_circle(coords[e1],punct,latimer)
                &&!test_if_within_circle(coords[e2],punct,latimer)
                &&!test_if_within_circle(coords[e3],punct,latimer)
                &&!test_if_within_circle(coords[e4],punct,latimer)
                &&!test_if_within_circle(coords[e5],punct,latimer));
        else k=1;
    }
    return k;
}

int points_on_the_portar(RobotCoords x,RobotCoords y,int id)
{
    int k=0;
    RobotCoords punct;
    float d;
    int m,n;
    float ax,ay,bx,by;
    ax=x.x;
    ay=x.y;
    bx=y.x;
    by=y.y;
    d=distance_between_points(x,y);
    while(m!=y.x||n!=y.y)
    {
        ax=ax+((y.x-x.x)/d);
        m=(int)round(ax);
        //printf("x= %d ",m);
        ay=ay+((y.y-x.y)/d);
        n=(int)round(ay);
        //printf("y= %d\n",n);
        punct.x=m;
        punct.y=n;
        if(!test_if_within_circle(coords[id],punct,latimer));
        else k=1;
    }
    return k;
}

int points_on_the_offensive2(RobotCoords x,RobotCoords y)
{
    int k=0;
    RobotCoords punct;
    float d;
    int m,n;
    float ax,ay,bx,by;
    ax=x.x;
    ay=x.y;
    bx=y.x;
    by=y.y;
    d=distance_between_points(x,y);
    while(m!=y.x||n!=y.y)
    {
        ax=ax+((y.x-x.x)/d);
        m=(int)round(ax);
        //printf("x= %d ",m);
        ay=ay+((y.y-x.y)/d);
        n=(int)round(ay);
        //printf("y= %d\n",n);
        punct.x=m;
        punct.y=n;
        if(!test_if_within_circle(punct,coords[0],latimer));
        else k=1;
    }
    return k;
}

RobotCoords points_on_the_poarta(RobotCoords x,RobotCoords y,int id)
{
    RobotCoords punct,punct_optim;
    punct_optim.y=(tinta_dreaptay+tinta_stangay)/2;
    punct_optim.x=tinta_x;
    float d;
    float distanta=0,distantamax=0;
    int m,n;
    float ax,ay,bx,by;
    ax=x.x;
    ay=x.y;
    bx=y.x;
    by=y.y;
    d=distance_between_points(x,y);
    while(m!=y.x||n!=y.y)
    {
        ax=ax+((y.x-x.x)/d);
        m=(int)round(ax);
        //printf("x= %d ",m);
        ay=ay+((y.y-x.y)/d);
        n=(int)round(ay);
        //printf("y= %d\n",n);
        punct.x=m;
        punct.y=n;
        if(!points_on_the_line(punct,coords[id]))
        {
            distanta=distanta_minima(punct);
            if(distanta>distantamax)
            {
                distantamax=distanta;
                punct_optim=punct;
            }
        }
    }
    return punct_optim;
}

RobotCoords points_on_the_verticala(RobotCoords x,RobotCoords y,RobotCoords target,int id)
{
    RobotCoords punct,punct_optim;
    punct_optim.y=target.y;
    punct_optim.x=x.x;
    float d;
    float distanta=0,distantamin=32000;
    int m,n;
    float ax,ay,bx,by;
    ax=x.x;
    ay=x.y;
    bx=y.x;
    by=y.y;
    d=distance_between_points(x,y);
    while(m!=y.x||n!=y.y)
    {
        ax=ax+((y.x-x.x)/d);
        m=(int)round(ax);
        //printf("x= %d ",m);
        ay=ay+((y.y-x.y)/d);
        n=(int)round(ay);
        //printf("y= %d\n",n);
        punct.x=m;
        punct.y=n;
        if(!points_on_the_line(punct,target))
        {
            distanta=distance_between_points(punct,target);
            if(distanta<distantamin)
            {
                distantamin=distanta;
                punct_optim=punct;
            }
        }
    }
    return punct_optim;
}

RobotCoords point_on_the_line(RobotCoords x,RobotCoords y,int d1)
{
    RobotCoords coordonate;
    float d;
    int m,n;
    float ax,ay,bx,by;
    ax=x.x;
    ay=x.y;
    bx=y.x;
    by=y.y;
    d=distance_between_points(x,y);
    d1=d1+d;
        ax=ax+(d1*(y.x-x.x)/d);
        m=(int)round(ax);
        //printf("x= %d ",m);
        ay=ay+(d1*(y.y-x.y)/d);
        n=(int)round(ay);
        //printf("y= %d\n",n);
        coordonate.x=m;
        coordonate.y=n;
        return coordonate;
}

void stop(int id)
{
    control[id].left = 0;
    control[id].right = 0;
    control[id].right_rotation=0;
    control[id].left_rotation=0;
}

int test_if_in_left(RobotCoords a,int y,int r)
{
    if(a.y<y-r) return 1;
    return 0;
}
int test_if_in_right(RobotCoords a,int y,int r)
{
    if(a.y>y+r) return 1;
    return 0;
}
int test_if_over(RobotCoords a,int x,int r)
{
    if(a.x>x+r) return 1;
    return 0;
}
int test_if_under(RobotCoords a,int x,int r)
{
    if(a.x<x-r) return 1;
    return 0;
}
void test_if_within_walls(RobotCoords b,int id,int radius)
{
    struct RobotCoords thisR = coords[id];

    if(!test_if_in_right(thisR,left_side,radius))
    {
        if((thisR.angle<45&&thisR.angle>=0)&&(thisR.angle<=360&&thisR.angle>330))
        {
            control[id].left=control[id].left+50;
            control[id].right=control[id].right-20;
            if(thisR.angle<45&&thisR.angle>0)
                control[id].right=-40;
        }
        if(thisR.angle<210&&thisR.angle>135)
        {
            control[id].right=control[id].right+50;
            control[id].left=control[id].left-20;
            if(thisR.angle<180&&thisR.angle>135)
                control[id].left=-40;
        }
    }

    if(!test_if_in_left(thisR,right_side,radius))
    {
        if((thisR.angle<30&&thisR.angle>=0)&&(thisR.angle<=360&&thisR.angle>315))
        {
            control[id].right=control[id].right+50;
            control[id].left=control[id].left-20;
            if(thisR.angle<360&&thisR.angle>315)
                control[id].left=-40;
        }
        if(thisR.angle<225&&thisR.angle>150)
        {
            control[id].left=control[id].left+50;
            control[id].right=control[id].right-20;
            if(thisR.angle<225&&thisR.angle>180)
                control[id].right=-40;
        }
    }

    if(!test_if_over(thisR,down_side,radius))
    {
        if(thisR.angle<135&&thisR.angle>60)
        {
            control[id].left=control[id].left+50;
            control[id].right=control[id].right-20;
            if(thisR.angle<135&&thisR.angle>90)
                control[id].right=-40;
        }
        if(thisR.angle<300&&thisR.angle>225)
        {
            control[id].right=control[id].right+50;
            control[id].left=control[id].left-20;
            if(thisR.angle<270&&thisR.angle>225)
                control[id].left=-40;
        }
    }

    if(!test_if_under(thisR,upper_side,radius))
    {
        if(thisR.angle<315&&thisR.angle>240)
        {
            control[id].left=control[id].left+50;
            control[id].right=control[id].right-20;
            if(thisR.angle<315&&thisR.angle>270)
                control[id].right=-40;
        }
        if(thisR.angle<120&&thisR.angle>45)
        {
            control[id].right=control[id].right+50;
            control[id].left=control[id].left-20;
            if(thisR.angle<90&&thisR.angle>45)
                control[id].left=-40;
        }
    }
}

int within_walls(RobotCoords thisR,int radius)
{

    int este=1;
    if(!test_if_in_right(thisR,left_side,radius))
    {
        este=0;
    }

    if(!test_if_in_left(thisR,right_side,radius))
    {
        este=0;
    }

    if(!test_if_over(thisR,down_side,radius))
    {
        este=0;
    }

    if(!test_if_under(thisR,upper_side,radius))
    {
        este=0;
    }
    return este;
}

//-------------------------------------------------------------------------


int test_if_warning_bila(int id)
{
    if(tinta_x==josx)
        if(test_if_within_circle(coords[0],coords[id],razab+diametrur*3+latimer/2)&&test_if_under(coords[id],coords[0].x,0))
            return 1;
    if(tinta_x==susx)
        if(test_if_within_circle(coords[0],coords[id],razab+diametrur*3+latimer/2)&&test_if_over(coords[id],coords[0].x,0))
            return 1;
    return 0;
}

int test_if_true_collision_bila(RobotCoords punct)
{
    if(tinta_x==josx)
        if(test_if_within_circle(coords[0],punct,latimer+latimer/2)&&test_if_under(punct,coords[0].x,0))
            return 1;
    if(tinta_x==susx)
        if(test_if_within_circle(coords[0],punct,latimer+latimer/2)&&test_if_over(punct,coords[0].x,0))
            return 1;
    return 0;
}

int test_if_warning_robot(int id,int robot)
{
    if(test_if_within_circle(coords[id],coords[robot],diametrur+latimer*3))
        return 1;
    return 0;
}

int test_if_true_collision_robot(RobotCoords punct,int robot)
{
    if(test_if_within_circle(punct,coords[robot],diametrur/2+latimer/2-8))
        return 1;
    return 0;
}


int test_if_collision_warning(int id)
{
    if(id==o1)
        if((test_if_warning_bila(id)||test_if_warning_robot(id,o2)||test_if_warning_robot(id,d1)||test_if_warning_robot(id,d2)||test_if_warning_robot(id,p)||test_if_warning_robot(id,e1)||test_if_warning_robot(id,e2)||test_if_warning_robot(id,e3)||test_if_warning_robot(id,e4)||test_if_warning_robot(id,e5)))
            return 1;
    if(id==o2)
        if((test_if_warning_bila(id)||test_if_warning_robot(id,o1)||test_if_warning_robot(id,d1)||test_if_warning_robot(id,d2)||test_if_warning_robot(id,p)||test_if_warning_robot(id,e1)||test_if_warning_robot(id,e2)||test_if_warning_robot(id,e3)||test_if_warning_robot(id,e4)||test_if_warning_robot(id,e5)))
            return 1;
    if(id==d1)
        if((test_if_warning_bila(id)||test_if_warning_robot(id,o2)||test_if_warning_robot(id,o1)||test_if_warning_robot(id,d2)||test_if_warning_robot(id,p)||test_if_warning_robot(id,e1)||test_if_warning_robot(id,e2)||test_if_warning_robot(id,e3)||test_if_warning_robot(id,e4)||test_if_warning_robot(id,e5)))
            return 1;
    if(id==d2)
        if((test_if_warning_bila(id)||test_if_warning_robot(id,o2)||test_if_warning_robot(id,d1)||test_if_warning_robot(id,o1)||test_if_warning_robot(id,p)||test_if_warning_robot(id,e1)||test_if_warning_robot(id,e2)||test_if_warning_robot(id,e3)||test_if_warning_robot(id,e4)||test_if_warning_robot(id,e5)))
            return 1;
    return 0;
}

int test_if_true_collision(RobotCoords punct,int id)
{
    if(id==o1)
        if((test_if_true_collision_robot(punct,o2)||test_if_true_collision_robot(punct,d1)||test_if_true_collision_robot(punct,d2)||test_if_true_collision_robot(punct,p)||test_if_true_collision_robot(punct,e1)||test_if_true_collision_robot(punct,e2)||test_if_true_collision_robot(punct,e3)||test_if_true_collision_robot(punct,e4)||test_if_true_collision_robot(punct,e5)))
            return 1;
    if(id==o2)
        if((test_if_true_collision_robot(punct,o1)||test_if_true_collision_robot(punct,d1)||test_if_true_collision_robot(punct,d2)||test_if_true_collision_robot(punct,p)||test_if_true_collision_robot(punct,e1)||test_if_true_collision_robot(punct,e2)||test_if_true_collision_robot(punct,e3)||test_if_true_collision_robot(punct,e4)||test_if_true_collision_robot(punct,e5)))
            return 1;
    if(id==d1)
        if((test_if_true_collision_robot(punct,o2)||test_if_true_collision_robot(punct,o1)||test_if_true_collision_robot(punct,d2)||test_if_true_collision_robot(punct,p)||test_if_true_collision_robot(punct,e1)||test_if_true_collision_robot(punct,e2)||test_if_true_collision_robot(punct,e3)||test_if_true_collision_robot(punct,e4)||test_if_true_collision_robot(punct,e5)))
            return 1;
    if(id==d2)
        if((test_if_true_collision_robot(punct,o2)||test_if_true_collision_robot(punct,d1)||test_if_true_collision_robot(punct,o1)||test_if_true_collision_robot(punct,p)||test_if_true_collision_robot(punct,e1)||test_if_true_collision_robot(punct,e2)||test_if_true_collision_robot(punct,e3)||test_if_true_collision_robot(punct,e4)||test_if_true_collision_robot(punct,e5)))
            return 1;
    if(id==p)
        if((test_if_true_collision_robot(punct,o2)||test_if_true_collision_robot(punct,d1)||test_if_true_collision_robot(punct,o1)||test_if_true_collision_robot(punct,d2)||test_if_true_collision_robot(punct,e1)||test_if_true_collision_robot(punct,e2)||test_if_true_collision_robot(punct,e3)||test_if_true_collision_robot(punct,e4)||test_if_true_collision_robot(punct,e5)))
            return 1;
    return 0;
}

RobotCoords gaseste_drum_pornire(RobotCoords punct_pornire,int id)
{
    int constanta=0,l,q,gasit=0,i;
    a[0]=abs(punct_pornire.x/10); a[1]=abs(punct_pornire.y/10);  q=2;
    v[a[0]][a[1]]=1;
    while(!gasit)
    {//cout<<"a intrat\n";
        l=constanta; constanta=q;
        for(i=l;i<constanta;i=i+2)
        {
            struct RobotCoords test;
            test.x=(a[i]+1)*10; test.y=(a[i+1])*10;
            if(!test_if_true_collision(test,id))
            {
                gasit=1;
                return test;
            }
            test.x=(a[i])*10; test.y=(a[i+1]+1)*10;
            if(!test_if_true_collision(test,id))
            {
                gasit=1;
                return test;
            }
            test.x=(a[i]-1)*10; test.y=(a[i+1])*10;
            if(!test_if_true_collision(test,id))
            {
                gasit=1;
                return test;
            }
            test.x=(a[i])*10; test.y=(a[i+1]-1)*10;
            if(!test_if_true_collision(test,id))
            {
                gasit=1;
                return test;
            }

           test.x=(a[i]+1)*10; test.y=(a[i+1])*10;
           if(within_walls(test,5))
           if(v[a[i]+1][a[i+1]] ==0)
           {v[a[i]+1][a[i+1]] = v[a[i]][a[i+1]]+1; a[q] = a[i]+1; a[q+1] = a[i+1]; q=q+2;}

           test.x=(a[i])*10; test.y=(a[i+1]+1)*10;
           if(within_walls(test,5))
           if(v[a[i]][a[i+1]+1] ==0)
           {v[a[i]][a[i+1]+1] = v[a[i]][a[i+1]]+1; a[q] = a[i]; a[q+1]=a[i+1]+1; q=q+2;}

           test.x=(a[i]-1)*10; test.y=(a[i+1])*10;
           if(within_walls(test,5))
           if(v[a[i]-1][a[i+1]] ==0)
           {v[a[i]-1][a[i+1]] = v[a[i]][a[i+1]]+1; a[q] = a[i]-1; a[q+1] = a[i+1]; q=q+2;}

           test.x=(a[i])*10; test.y=(a[i+1]-1)*10;
           if(within_walls(test,5))
           if(v[a[i]][a[i+1]-1] ==0)
           {v[a[i]][a[i+1]-1] = v[a[i]][a[i+1]]+1; a[q] = a[i]; a[q+1] = a[i+1]-1; q=q+2;}
        }
    }

}

RobotCoords initializeaza_punct_pornire(RobotCoords punct_pornire,int id)
{
    struct RobotCoords punct;
    int i,j;
    for(i=0;i<150;i++)
        for(j=0;j<80;j++)
        {z[i][j]=0;
            v[i][j]=0;}

    punct=gaseste_drum_pornire(punct_pornire,id);

    /*
    for(i=0;i<150;i++)
    {for(j=0;j<80;j++)
        {
            if(v[i][j]<10&&v[i][j]>=0) h<<"  "<<v[i][j]<<" ";
            else if(v[i][j]<100&&v[i][j]>=10) h<<" "<<v[i][j]<<" ";
            else h<<v[i][j]<<" ";
        }
        h<<endl;
    }
    h<<endl;
*/

    //h.close();
    return punct;
}

int test_if_spatiu_liber(RobotCoords x,RobotCoords y,int id)
{
    int k=0;
    RobotCoords punct;
    float d;
    int m,n;
    float ax,ay,bx,by;
    ax=x.x;
    ay=x.y;
    bx=y.x;
    by=y.y;
    d=distance_between_points(x,y);
    while(m!=y.x||n!=y.y)
    {
        ax=ax+((y.x-x.x)/d);
        m=(int)round(ax);
        //printf("x= %d ",m);
        ay=ay+((y.y-x.y)/d);
        n=(int)round(ay);
        //printf("y= %d\n",n);
        punct.x=m;
        punct.y=n;
        if(test_if_true_collision(punct,id));
        else k=1;
    }
    return k;
}

RobotCoords cauta_spatiu_liber(RobotCoords x,RobotCoords y,int id)
{

    RobotCoords punct;
    float d;
    int m,n;
    float ax,ay,bx,by;
    ax=x.x;
    ay=x.y;
    bx=y.x;
    by=y.y;
    d=distance_between_points(x,y);
    while(m!=y.x||n!=y.y)
    {
        ax=ax+((y.x-x.x)/d);
        m=(int)round(ax);
        //printf("x= %d ",m);
        ay=ay+((y.y-x.y)/d);
        n=(int)round(ay);
        //printf("y= %d\n",n);
        punct.x=m;
        punct.y=n;
        if(test_if_true_collision(punct,id));
        else return punct;
    }

}

RobotCoords parcurgere_inversa(int x,int y,int cautat)
{
    struct RobotCoords punct_de_returnat,punct;

 int gasit=0;
 if(cautat>v[x][y]) {cautat=v[x][y]; cout<<"BA ANIMALULE, DISTANTA E MAI MICA DE ATAT!\n";}
 while(!gasit)
 {
     z[x][y]=v[x][y];
     if(v[x+1][y] == cautat && !gasit)
     {
         punct_de_returnat.x=(x+1)*10;punct_de_returnat.y=y*10;
         gasit=1;

     }
     if(v[x][y+1] == cautat && !gasit)
     {
         punct_de_returnat.x=x*10;punct_de_returnat.y=(y+1)*10;
         gasit=1;

     }
     if(v[x-1][y] == cautat && !gasit)
     {
         punct_de_returnat.x=(x-1)*10;punct_de_returnat.y=y*10;
         gasit=1;

     }
     if(v[x][y-1] == cautat && !gasit)
     {
         punct_de_returnat.x=x*10;punct_de_returnat.y=(y-1)*10;
         gasit=1;

     }
     //cout<<v[x][y]<<endl;
     punct.x=(x+1)*10;punct.y=y*10;
     if(within_walls(punct,5))
     if(v[x+1][y] == v[x][y]-1 && !gasit) { x=x+1; y=y; z[x][y]=v[x][y];}

     punct.x=x*10;punct.y=(y+1)*10;
     if(within_walls(punct,5))
     if(v[x][y+1] == v[x][y]-1 && !gasit) { x=x; y=y+1; z[x][y]=v[x][y];}

     punct.x=(x-1)*10;punct.y=y*10;
     if(within_walls(punct,5))
     if(v[x-1][y] == v[x][y]-1 && !gasit) { x=x-1; y=y; z[x][y]=v[x][y];}

     punct.x=x*10;punct.y=(y-1)*10;
     if(within_walls(punct,5))
     if(v[x][y-1] == v[x][y]-1 && !gasit) { x=x; y=y-1; z[x][y]=v[x][y];}

 }
 return punct_de_returnat;
}

RobotCoords gaseste_drum(RobotCoords punct_plecare,RobotCoords punct_tinta,int id)
{
    struct RobotCoords punct,test;
    int x,y,constanta=0,l,q,gasit=0,i,j;
    a[0]=punct_plecare.x/10; a[1]=punct_plecare.y/10; q=2;
    v[a[0]][a[1]]=1; z[a[0]][a[1]]=1;
    i=punct_tinta.x/10; j=punct_tinta.y/10; v[i][j]=-3; z[i][j]=-3;

    while(!gasit)
    {//cout<<"a intrat\n";
        l=constanta; constanta=q;
        for(i=l;i<constanta;i=i+2)
        {
           if(v[a[i]+1][a[i+1]] == -3 && !gasit) {gasit=1; test.x=a[i]*10; test.y=a[i+1]*10;}
           if(v[a[i]][a[i+1]+1] == -3 && !gasit) {gasit=1; test.x=a[i]*10; test.y=a[i+1]*10;}
           if(v[a[i]-1][a[i+1]] == -3 && !gasit) {gasit=1; test.x=a[i]*10; test.y=a[i+1]*10;}
           if(v[a[i]][a[i+1]-1] == -3 && !gasit) {gasit=1; test.x=a[i]*10; test.y=a[i+1]*10;}

           punct.x=(a[i]+1)*10; punct.y=(a[i+1])*10;
           if(within_walls(punct,5))
           {
               if(test_if_true_collision(punct,id))
               {
                   v[a[i]+1][a[i+1]]=-1;
                   z[a[i]+1][a[i+1]]=-1;
               }
               if(!test_if_true_collision(punct,id)&&(v[a[i]+1][a[i+1]] > v[a[i]][a[i+1]]+1 || v[a[i]+1][a[i+1]] == 0) && !gasit)
           {v[a[i]+1][a[i+1]] = v[a[i]][a[i+1]]+1; a[q] = a[i]+1; a[q+1] = a[i+1]; q=q+2;}
           }

           punct.x=(a[i])*10; punct.y=(a[i+1]+1)*10;
           if(within_walls(punct,5))
           {
               if(test_if_true_collision(punct,id))
               {
                   v[a[i]][a[i+1]+1]=-1;
                   z[a[i]+1][a[i+1]]=-1;
               }
               if(!test_if_true_collision(punct,id)&&(v[a[i]][a[i+1]+1] > v[a[i]][a[i+1]]+1 || v[a[i]][a[i+1]+1] == 0) && !gasit)
           {v[a[i]][a[i+1]+1] = v[a[i]][a[i+1]]+1; a[q] = a[i]; a[q+1]=a[i+1]+1; q=q+2;}
           }

           punct.x=(a[i]-1)*10; punct.y=(a[i+1])*10;
           if(within_walls(punct,5))
           {
               if(test_if_true_collision(punct,id))
               {
                   v[a[i]-1][a[i+1]]=-1;
                   z[a[i]+1][a[i+1]]=-1;
               }
           if(!test_if_true_collision(punct,id)&&(v[a[i]-1][a[i+1]] > v[a[i]][a[i+1]]+1 || v[a[i]-1][a[i+1]] == 0) && !gasit)
           {v[a[i]-1][a[i+1]] = v[a[i]][a[i+1]]+1; a[q] = a[i]-1; a[q+1] = a[i+1]; q=q+2;}
           }

           punct.x=(a[i])*10; punct.y=(a[i+1]-1)*10;
           if(within_walls(punct,5))
           {
               if(test_if_true_collision(punct,id))
               {
                   v[a[i]][a[i+1]-1]=-1;
                   z[a[i]+1][a[i+1]]=-1;
               }
           if(!test_if_true_collision(punct,id)&&(v[a[i]][a[i+1]-1] > v[a[i]][a[i+1]]+1 || v[a[i]][a[i+1]-1] == 0) && !gasit)
           {v[a[i]][a[i+1]-1] = v[a[i]][a[i+1]]+1; a[q] = a[i]; a[q+1] = a[i+1]-1; q=q+2;}
           }
        }
    }
    punct=parcurgere_inversa(test.x/10,test.y/10,eroare);
    return punct;
}


RobotCoords ocolire(RobotCoords punct_plecare,RobotCoords punct_tinta,int id)
{
    struct RobotCoords punct;
    int i,j;
    for(i=0;i<150;i++)
        for(j=0;j<80;j++)
        {z[i][j]=0;
            v[i][j]=0;}

    punct=gaseste_drum(punct_plecare,punct_tinta,id);
/*
    for(i=0;i<150;i++)
    {for(j=0;j<80;j++)
        {
            if(v[i][j]<10&&v[i][j]>=0) g<<" "<<v[i][j]<<" ";
            else if(v[i][j]<100&&z[i][j]>=10) g<<" "<<z[i][j]<<" ";
            else if(v[i][j]==-1) g<<"===";
            else g<<v[i][j]<<" ";
        }
        g<<endl;
    }
    g<<endl;

    for(i=0;i<150;i++)
    {for(j=0;j<80;j++)
        {
            if(z[i][j]<10&&z[i][j]>=0) g<<"  "<<z[i][j]<<" ";
            else if(z[i][j]<100&&z[i][j]>=10) g<<" "<<z[i][j]<<" ";
            else if(z[i][j]==-1) g<<"===";
            else g<<z[i][j]<<" ";
        }
        g<<endl;
    }*/
    //g.close();
    return punct;
}

void berserk(int id)
{
    control[id].left=220;
    control[id].right=-220;
    control[id].left_rotation=32;
    control[id].right_rotation=32;
}

RobotCoords recalculate_target(RobotCoords thisR,RobotCoords target,int id)
{

    if(test_if_collision_warning(id))
    {
        printf("WARNING,COLLISION!!!\n");
    struct RobotCoords punct_pornire_cautare,punct_tinta;
    if(test_if_true_collision(thisR,id))
    {
        printf("!!!!!!!!TRUE COLLISION______!!!!!!\n");
        punct_pornire_cautare= initializeaza_punct_pornire(thisR,id);
        return punct_pornire_cautare;
    }
    else
        punct_pornire_cautare=thisR;

    if(!test_if_spatiu_liber(target,punct_pornire_cautare,id))
    {
        berserk(id);
        return target;
    }
    else
    {
        punct_tinta=cauta_spatiu_liber(target,punct_pornire_cautare,id);
        target=ocolire(punct_pornire_cautare,punct_tinta,id);
    }

    }
    else return target;
}

//-------------------------------------------------------------------------

void interschimba_ofensiva()
{
    int aux;
    aux=o1;
    o1=o2;
    o2=aux;
    char sir[100];
    strcpy(sir,id_o1);
    strcpy(id_o1,id_o2);
    strcpy(id_o2,sir);
}

void interschimba_defensiva()
{
    int aux;
    aux=d1;
    d1=d2;
    d2=aux;
    char sir[100];
    strcpy(sir,id_d1);
    strcpy(id_d1,id_d2);
    strcpy(id_d2,sir);
}

void interschimba_defensiva2()
{
    int aux;
    aux=o1;
    o1=d1;
    d1=aux;
    char sir[100];
    strcpy(sir,id_o1);
    strcpy(id_o1,id_d1);
    strcpy(id_d1,sir);
}


void switch_defense()
{
    RobotCoords b=coords[0];
    if(tinta_x==josx)
        if((distance_between_points(coords[d1],b)>distance_between_points(coords[d2],b)&&!test_if_under(coords[d2],coords[0].x,0))||(test_if_under(coords[d1],coords[0].x,-latimer)&&!test_if_within_circle(coords[0],coords[d1],diametrur+diametrub)&&!test_if_under(coords[d2],coords[0].x,0)))
            interschimba_defensiva();
    if(tinta_x==susx)
        if((distance_between_points(coords[d1],b)>distance_between_points(coords[d2],b)&&test_if_over(coords[d2],coords[0].x,0))||(test_if_over(coords[d1],coords[0].x,-latimer)&&!test_if_within_circle(coords[0],coords[d1],diametrur+diametrub)&&!test_if_over(coords[d2],coords[0].x,0)))
        {interschimba_defensiva();//printf("SUNT UN BOU\n");
        }
    /*
    if(tinta_x==josx)
        if(test_if_under(coords[o1],coords[0].x,-latimer)&&test_if_under(coords[o2],coords[0].x,-latimer)&&!test_if_within_circle(coords[0],coords[o1],diametrur+diametrub)&&!test_if_within_circle(coords[0],coords[o2],diametrur+diametrub))
            interschimba_defensiva2();
    if(tinta_x==susx)
        if(test_if_over(coords[o1],coords[0].x,-latimer)&&test_if_over(coords[o2],coords[0].x,-latimer)&&!test_if_within_circle(coords[0],coords[o1],diametrur+diametrub)&&!test_if_within_circle(coords[0],coords[o2],diametrur+diametrub))
            interschimba_defensiva2();
            */
}

void switch_offensive()
{
    RobotCoords b=coords[0];
    if(tinta_x==josx)
        if(distance_between_points(coords[o1],b)>distance_between_points(coords[o2],b)||(test_if_under(coords[o1],coords[0].x,-latimer)&&!test_if_within_circle(coords[0],coords[o1],diametrur+diametrub)&&!test_if_under(coords[o2],coords[0].x,0)))
            interschimba_ofensiva();
    if(tinta_x==susx)
        if(distance_between_points(coords[o1],b)>distance_between_points(coords[o2],b)||(test_if_over(coords[o1],coords[0].x,-latimer)&&!test_if_within_circle(coords[0],coords[o1],diametrur+diametrub)&&!test_if_over(coords[o2],coords[0].x,0)))
        {interschimba_ofensiva();//printf("SUNT UN BOU\n");
        }
}

void attack(RobotCoords thisR,RobotCoords poarta,RobotCoords target,int id)
{
    double angle, unghi;
    if(test_if_within_circle(target,thisR,diametrub))
    {

        //inainte era 33
        //printf("\nLA TINTA\n");
        angle=unghi_tinta(thisR,poarta);
        unghi=calculeaza_unghi(angle,thisR.angle);
        //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);

        if(unghi>20)
            intoarcere(angle,thisR.angle,id);

        else
        {


            mergi_drept(angle,thisR.angle,id);
            if(control[id].right>60)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            if(control[id].right>80)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            if(control[id].right>100)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            //control[id].right_rotation=4;
            //control[id].left_rotation=4;
        }

    }

    else
    {
        //printf("CE DRACU FACI?!\n");
        angle=unghi_tinta(thisR,target);
        unghi=calculeaza_unghi(angle,thisR.angle);

        if(unghi>40)
        {
            intoarcere(angle,thisR.angle,id);

        }


        else
        {

                mergi_drept(angle,thisR.angle,id);
                if(distance_between_points(coords[0],thisR)<400)
                {
                    if(control[id].right>60)
                    {
                        control[id].right=control[id].right-10;
                        control[id].left=control[id].left-10;
                    }
                    if(control[id].right>80)
                    {
                        control[id].right=control[id].right-10;
                        control[id].left=control[id].left-10;
                    }
                    if(control[id].right>100)
                    {
                        control[id].right=control[id].right-10;
                        control[id].left=control[id].left-10;
                    }

                }
                if(distance_between_points(coords[0],thisR)<200)
                {
                    //control[id].right=control[id].right;
                    //control[id].left=control[id].left;
                    control[id].right_rotation=8;
                    control[id].left_rotation=8;
                }
                if(distance_between_points(target,thisR)<66+33)
                {
                    //control[id].right=control[id].right-40;
                    //control[id].left=control[id].left-40;
                    control[id].right_rotation=8;
                    control[id].left_rotation=8;
                }
                if(distance_between_points(target,thisR)<66)
                {
                    control[id].right=control[id].right;
                    control[id].left=control[id].left;
                    control[id].right_rotation=4;
                    control[id].left_rotation=4;
                }
                if(distance_between_points(target,thisR)<33)
                {
                    control[id].right=control[id].right;
                    control[id].left=control[id].left;
                    control[id].right_rotation=4;
                    control[id].left_rotation=4;
                }

            }


        }
    }

void attack_cu_spatele(RobotCoords thisR,RobotCoords poarta,RobotCoords target,int id)
{
    double angle, unghi;
    if(test_if_within_circle(target,thisR,diametrub))
    {

        //inainte era 33
        //printf("\nLA TINTA\n");
        angle=unghi_tinta(thisR,poarta);
        unghi=calculeaza_unghi(angle,thisR.angle);
        //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);

        if(unghi>20&&unghi<=90)
            intoarcere(angle,thisR.angle,id);
        else if(unghi>90&&unghi<160)
            intoarcere_cu_spatele(angle,thisR.angle,id);
        else if(unghi<=20)
        {


            mergi_drept(angle,thisR.angle,id);
            if(control[id].right>60)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            if(control[id].right>80)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            if(control[id].right>100)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            //control[id].right_rotation=4;
            //control[id].left_rotation=4;
        }
        else if(unghi>=160)
        {


            mergi_cu_spatele(angle,thisR.angle,id);
            if(control[id].right<-60)
            {
                control[id].right=control[id].right+10;
                control[id].left=control[id].left+10;
            }
            if(control[id].right<-80)
            {
                control[id].right=control[id].right+10;
                control[id].left=control[id].left+10;
            }
            if(control[id].right<-100)
            {
                control[id].right=control[id].right+10;
                control[id].left=control[id].left+10;
            }
            //control[id].right_rotation=4;
            //control[id].left_rotation=4;
        }
    }

    else
    {
        //printf("CE DRACU FACI?!\n");
        angle=unghi_tinta(thisR,target);
        unghi=calculeaza_unghi(angle,thisR.angle);

        if(unghi>40&&unghi<=90)
        {
            intoarcere(angle,thisR.angle,id);

        }

        else if(unghi>90&&unghi<140)
        {
            intoarcere_cu_spatele(angle,thisR.angle,id);
        }
        else if(unghi<=40)
        {

                mergi_drept(angle,thisR.angle,id);
                if(distance_between_points(coords[0],thisR)<500)
                {
                    if(control[id].right>60)
                    {
                        control[id].right=control[id].right-10;
                        control[id].left=control[id].left-10;
                    }
                    if(control[id].right>80)
                    {
                        control[id].right=control[id].right-10;
                        control[id].left=control[id].left-10;
                    }
                    if(control[id].right>100)
                    {
                        control[id].right=control[id].right-10;
                        control[id].left=control[id].left-10;
                    }

                }
                if(distance_between_points(coords[0],thisR)<200)
                {
                    //control[id].right=control[id].right;
                    //control[id].left=control[id].left;
                    control[id].right_rotation=8;
                    control[id].left_rotation=8;
                }
                if(distance_between_points(target,thisR)<66+33)
                {
                    //control[id].right=control[id].right-40;
                    //control[id].left=control[id].left-40;
                    control[id].right_rotation=8;
                    control[id].left_rotation=8;
                }
                if(distance_between_points(target,thisR)<66)
                {
                    control[id].right=control[id].right;
                    control[id].left=control[id].left;
                    control[id].right_rotation=4;
                    control[id].left_rotation=4;
                }
                if(distance_between_points(target,thisR)<33)
                {
                    control[id].right=control[id].right;
                    control[id].left=control[id].left;
                    control[id].right_rotation=4;
                    control[id].left_rotation=4;
                }

            }
        else if(unghi>=140)
        {

                mergi_cu_spatele(angle,thisR.angle,id);
                if(distance_between_points(coords[0],thisR)<500)
                {
                    if(control[id].right<-60)
                    {
                        control[id].right=control[id].right+10;
                        control[id].left=control[id].left+10;
                    }
                    if(control[id].right<-80)
                    {
                        control[id].right=control[id].right+10;
                        control[id].left=control[id].left+10;
                    }
                    if(control[id].right<-100)
                    {
                        control[id].right=control[id].right+10;
                        control[id].left=control[id].left+10;
                    }

                }
                if(distance_between_points(coords[0],thisR)<200)
                {
                    //control[id].right=control[id].right;
                    //control[id].left=control[id].left;
                    control[id].right_rotation=8;
                    control[id].left_rotation=8;
                }
                if(distance_between_points(target,thisR)<66+33)
                {
                    //control[id].right=control[id].right-40;
                    //control[id].left=control[id].left-40;
                    control[id].right_rotation=8;
                    control[id].left_rotation=8;
                }
                if(distance_between_points(target,thisR)<66)
                {
                    control[id].right=control[id].right;
                    control[id].left=control[id].left;
                    control[id].right_rotation=4;
                    control[id].left_rotation=4;
                }
                if(distance_between_points(target,thisR)<33)
                {
                    control[id].right=control[id].right;
                    control[id].left=control[id].left;
                    control[id].right_rotation=4;
                    control[id].left_rotation=4;
                }

            }

        }
    }

void go_to_pozitie(RobotCoords thisR,RobotCoords target,RobotCoords pozitie,int id)
{

    double angle, unghi;
    if(test_if_within_circle(pozitie,thisR,diametrur))
    {

        //inainte era 33
        //printf("\nLA TINTA\n");
        angle=unghi_tinta(thisR,target);
        unghi=calculeaza_unghi(angle,thisR.angle);
        //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);

        if(target.y=(tinta_dreaptay+tinta_stangay)/2)
        {
            if(points_on_the_offensive2(thisR,target))
            {
                control[id].left = 500;
                control[id].right = 500;
                control[id].right_rotation=32;
                control[id].left_rotation=32;
                publish(id);

            }
        }

        if(unghi>20)
            intoarcere(angle,thisR.angle,id);
        else
            stop(id);
    }

    else
    {

        //printf("CE DRACU FACI?!\n");
        angle=unghi_tinta(thisR,pozitie);
        unghi=calculeaza_unghi(angle,thisR.angle);

        if(unghi>40)
        {

            intoarcere(angle,thisR.angle,id);
            /*if(distance_between_points(target,thisR)<66+33)
            {
                control[id].right=control[id].right/1.5;
                control[id].left=control[id].left/1.5;
            }*/
        }
        else
        {
            //printf("merge la pozitie");
            mergi_drept(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
        }
    }
}

void go_to_pozitie_reset(RobotCoords thisR,RobotCoords target,RobotCoords pozitie,int id)
{

    double angle, unghi;
    if(test_if_within_circle(pozitie,thisR,latimer))
    {

        //inainte era 33
        //printf("\nLA TINTA\n");
        angle=unghi_tinta(thisR,target);
        unghi=calculeaza_unghi(angle,thisR.angle);
        //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);



        if(unghi>40)
            intoarcere(angle,thisR.angle,id);
        else
            stop(id);
    }

    else
    {

        //printf("CE DRACU FACI?!\n");
        angle=unghi_tinta(thisR,pozitie);
        unghi=calculeaza_unghi(angle,thisR.angle);

        if(unghi>40)
        {

            intoarcere(angle,thisR.angle,id);
            /*if(distance_between_points(target,thisR)<66+33)
            {
                control[id].right=control[id].right/1.5;
                control[id].left=control[id].left/1.5;
            }*/
        }
        else
        {
            //printf("merge la pozitie");
            mergi_drept(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=3;
                control[id].left_rotation=3;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=2;
                control[id].left_rotation=2;
            }
        }
    }
}

void go_to_pozitie_cu_spatele(RobotCoords thisR,RobotCoords target,RobotCoords pozitie,int id)
{

    double angle, unghi;
    if(test_if_within_circle(pozitie,thisR,diametrur))
    {

        //inainte era 33
        //printf("\nLA TINTA\n");
        angle=unghi_tinta(thisR,target);
        unghi=calculeaza_unghi(angle,thisR.angle);
        //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);


        if(unghi>20&&unghi<=90)
            intoarcere(angle,thisR.angle,id);
        else if(unghi>90&&unghi<160)
            intoarcere_cu_spatele(angle,thisR.angle,id);
        else
            stop(id);
    }

    else
    {

        //printf("CE DRACU FACI?!\n");
        angle=unghi_tinta(thisR,pozitie);
        unghi=calculeaza_unghi(angle,thisR.angle);

        if(unghi>40&&unghi<=90)
        {

            intoarcere(angle,thisR.angle,id);
            /*if(distance_between_points(target,thisR)<66+33)
            {
                control[id].right=control[id].right/1.5;
                control[id].left=control[id].left/1.5;
            }*/
        }
        else if(unghi>90&&unghi<140)
            intoarcere_cu_spatele(angle,thisR.angle,id);
        else if(unghi<=40)
        {
            //printf("merge la pozitie");
            mergi_drept(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
        }
        else if(unghi>=140)
        {
            //printf("merge la pozitie");
            mergi_cu_spatele(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right<-60)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-80)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-100)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
        }
    }
}

void go_to_pozitie_def(RobotCoords thisR,RobotCoords pozitie,int id)
{
    double angle, unghi;

    {
        //printf("CE DRACU FACI?!\n");
        angle=unghi_tinta(thisR,pozitie);
        unghi=calculeaza_unghi(angle,thisR.angle);

        if(unghi>40)
        {
            intoarcere(angle,thisR.angle,id);

        }
        else
        {
            mergi_drept(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=3;
                control[id].left_rotation=3;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=2;
                control[id].left_rotation=2;
            }
        }
    }
}

void go_to_pozitie_def_cu_spatele(RobotCoords thisR,RobotCoords pozitie,int id)
{
    double angle, unghi;

    {
        //printf("CE DRACU FACI?!\n");
        angle=unghi_tinta(thisR,pozitie);
        unghi=calculeaza_unghi(angle,thisR.angle);

        if(unghi>40&&unghi<=90)
        {
            intoarcere(angle,thisR.angle,id);

        }
        else if(unghi>90&&unghi<140)
            intoarcere_cu_spatele(angle,thisR.angle,id);
        else if(unghi<=40)
        {
            mergi_drept(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=3;
                control[id].left_rotation=3;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=2;
                control[id].left_rotation=2;
            }
        }
        else if(unghi>=140)
        {
            mergi_cu_spatele(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right<-60)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-80)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-100)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=3;
                control[id].left_rotation=3;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=2;
                control[id].left_rotation=2;
            }
        }
    }
}

void defend(RobotCoords thisR,RobotCoords poarta,RobotCoords target,int id)
{
    double angle, unghi;
    if(tinta_x==josx)
    {
        if(test_if_within_circle(target,thisR,3*diametrub)&&test_if_over(thisR,coords[0].x,razab))
    {

        //inainte era 33
        //printf("\nLA TINTA\n");
        angle=unghi_tinta(thisR,poarta);
        unghi=calculeaza_unghi(angle,thisR.angle);
        //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);

        if(unghi>40)
            intoarcere(angle,thisR.angle,id);

        else
        {


            mergi_drept(angle,thisR.angle,id);
            if(control[id].right>60)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            if(control[id].right>80)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            if(control[id].right>100)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            //control[id].right_rotation=4;
            //control[id].left_rotation=4;
        }

    }

        else
        {
            //printf("CE DRACU FACI?!\n");
            angle=unghi_tinta(thisR,target);
            unghi=calculeaza_unghi(angle,thisR.angle);

            if(unghi>40)
            {
                intoarcere(angle,thisR.angle,id);

            }


            else
            {

                    mergi_drept(angle,thisR.angle,id);
                    if(distance_between_points(coords[0],thisR)<400)
                    {
                        if(control[id].right>60)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }
                        if(control[id].right>80)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }
                        if(control[id].right>100)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }

                    }
                    if(distance_between_points(coords[0],thisR)<200)
                    {
                        //control[id].right=control[id].right;
                        //control[id].left=control[id].left;
                        control[id].right_rotation=16;
                        control[id].left_rotation=16;
                    }
                    if(distance_between_points(target,thisR)<66+33)
                    {
                        //control[id].right=control[id].right-40;
                        //control[id].left=control[id].left-40;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }
                    if(distance_between_points(target,thisR)<33)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }

                }


            }

    }
    if(tinta_x==susx)
    {
        if(test_if_within_circle(target,thisR,3*diametrub)&&test_if_under(thisR,coords[0].x,razab))
    {

        //inainte era 33
        //printf("\nLA TINTA\n");
        angle=unghi_tinta(thisR,poarta);
        unghi=calculeaza_unghi(angle,thisR.angle);
        //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);

        if(unghi>40)
            intoarcere(angle,thisR.angle,id);

        else
        {


            mergi_drept(angle,thisR.angle,id);
            if(control[id].right>60)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            if(control[id].right>80)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            if(control[id].right>100)
            {
                control[id].right=control[id].right-10;
                control[id].left=control[id].left-10;
            }
            //control[id].right_rotation=4;
            //control[id].left_rotation=4;
        }

    }

        else
        {
            //printf("CE DRACU FACI?!\n");
            angle=unghi_tinta(thisR,target);
            unghi=calculeaza_unghi(angle,thisR.angle);

            if(unghi>40)
            {
                intoarcere(angle,thisR.angle,id);

            }


            else
            {

                    mergi_drept(angle,thisR.angle,id);
                    if(distance_between_points(coords[0],thisR)<400)
                    {
                        if(control[id].right>60)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }
                        if(control[id].right>80)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }
                        if(control[id].right>100)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }

                    }
                    if(distance_between_points(coords[0],thisR)<200)
                    {
                        //control[id].right=control[id].right;
                        //control[id].left=control[id].left;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66+33)
                    {
                        //control[id].right=control[id].right-40;
                        //control[id].left=control[id].left-40;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }
                    if(distance_between_points(target,thisR)<33)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }

                }


            }

    }

}

void defend_cu_spatele(RobotCoords thisR,RobotCoords poarta,RobotCoords target,int id)
{
    double angle, unghi;

    if(tinta_x==josx)
    {
        if(test_if_within_circle(target,thisR,3*diametrub)&&test_if_over(thisR,coords[0].x,razab))
        {

            //inainte era 33
            //printf("\nLA TINTA\n");
            angle=unghi_tinta(thisR,poarta);
            unghi=calculeaza_unghi(angle,thisR.angle);
            //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);

            if(unghi>30&&unghi<=90)
                intoarcere(angle,thisR.angle,id);
            else if(unghi>90&&unghi<160)
                intoarcere_cu_spatele(angle,thisR.angle,id);
            else if(unghi<=20)
            {


                mergi_drept(angle,thisR.angle,id);
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                //control[id].right_rotation=4;
                //control[id].left_rotation=4;
            }
            else if(unghi>=160)
            {


                mergi_cu_spatele(angle,thisR.angle,id);
                if(control[id].right<-60)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-80)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-100)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                //control[id].right_rotation=4;
                //control[id].left_rotation=4;
            }
        }

        else
        {
            //printf("CE DRACU FACI?!\n");
            angle=unghi_tinta(thisR,target);
            unghi=calculeaza_unghi(angle,thisR.angle);

            if(unghi>40&&unghi<=90)
            {
                intoarcere(angle,thisR.angle,id);

            }

            else if(unghi>90&&unghi<140)
            {
                intoarcere_cu_spatele(angle,thisR.angle,id);
            }
            else if(unghi<=40)
            {

                    mergi_drept(angle,thisR.angle,id);
                    if(distance_between_points(coords[0],thisR)<500)
                    {
                        if(control[id].right>60)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }
                        if(control[id].right>80)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }
                        if(control[id].right>100)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }

                    }
                    if(distance_between_points(coords[0],thisR)<200)
                    {
                        //control[id].right=control[id].right;
                        //control[id].left=control[id].left;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66+33)
                    {
                        //control[id].right=control[id].right-40;
                        //control[id].left=control[id].left-40;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }
                    if(distance_between_points(target,thisR)<33)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }

                }
            else if(unghi>=140)
            {

                    mergi_cu_spatele(angle,thisR.angle,id);
                    if(distance_between_points(coords[0],thisR)<500)
                    {
                        if(control[id].right<-60)
                        {
                            control[id].right=control[id].right+10;
                            control[id].left=control[id].left+10;
                        }
                        if(control[id].right<-80)
                        {
                            control[id].right=control[id].right+10;
                            control[id].left=control[id].left+10;
                        }
                        if(control[id].right<-100)
                        {
                            control[id].right=control[id].right+10;
                            control[id].left=control[id].left+10;
                        }

                    }
                    if(distance_between_points(coords[0],thisR)<200)
                    {
                        //control[id].right=control[id].right;
                        //control[id].left=control[id].left;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66+33)
                    {
                        //control[id].right=control[id].right-40;
                        //control[id].left=control[id].left-40;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }
                    if(distance_between_points(target,thisR)<33)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }

                }

            }
    }

    if(tinta_x==susx)
    {
        if(test_if_within_circle(target,thisR,3*diametrub)&&test_if_over(thisR,coords[0].x,razab))
        {

            //inainte era 33
            //printf("\nLA TINTA\n");
            angle=unghi_tinta(thisR,poarta);
            unghi=calculeaza_unghi(angle,thisR.angle);
            //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);

            if(unghi>30&&unghi<=90)
                intoarcere(angle,thisR.angle,id);
            else if(unghi>90&&unghi<160)
                intoarcere_cu_spatele(angle,thisR.angle,id);
            else if(unghi<=20)
            {


                mergi_drept(angle,thisR.angle,id);
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                //control[id].right_rotation=4;
                //control[id].left_rotation=4;
            }
            else if(unghi>=160)
            {


                mergi_cu_spatele(angle,thisR.angle,id);
                if(control[id].right<-60)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-80)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-100)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                //control[id].right_rotation=4;
                //control[id].left_rotation=4;
            }
        }

        else
        {
            //printf("CE DRACU FACI?!\n");
            angle=unghi_tinta(thisR,target);
            unghi=calculeaza_unghi(angle,thisR.angle);

            if(unghi>40&&unghi<=90)
            {
                intoarcere(angle,thisR.angle,id);

            }

            else if(unghi>90&&unghi<140)
            {
                intoarcere_cu_spatele(angle,thisR.angle,id);
            }
            else if(unghi<=40)
            {

                    mergi_drept(angle,thisR.angle,id);
                    if(distance_between_points(coords[0],thisR)<500)
                    {
                        if(control[id].right>60)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }
                        if(control[id].right>80)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }
                        if(control[id].right>100)
                        {
                            control[id].right=control[id].right-10;
                            control[id].left=control[id].left-10;
                        }

                    }
                    if(distance_between_points(coords[0],thisR)<200)
                    {
                        //control[id].right=control[id].right;
                        //control[id].left=control[id].left;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66+33)
                    {
                        //control[id].right=control[id].right-40;
                        //control[id].left=control[id].left-40;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }
                    if(distance_between_points(target,thisR)<33)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }

                }
            else if(unghi>=140)
            {

                    mergi_cu_spatele(angle,thisR.angle,id);
                    if(distance_between_points(coords[0],thisR)<500)
                    {
                        if(control[id].right<-60)
                        {
                            control[id].right=control[id].right+10;
                            control[id].left=control[id].left+10;
                        }
                        if(control[id].right<-80)
                        {
                            control[id].right=control[id].right+10;
                            control[id].left=control[id].left+10;
                        }
                        if(control[id].right<-100)
                        {
                            control[id].right=control[id].right+10;
                            control[id].left=control[id].left+10;
                        }

                    }
                    if(distance_between_points(coords[0],thisR)<200)
                    {
                        //control[id].right=control[id].right;
                        //control[id].left=control[id].left;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66+33)
                    {
                        //control[id].right=control[id].right-40;
                        //control[id].left=control[id].left-40;
                        control[id].right_rotation=8;
                        control[id].left_rotation=8;
                    }
                    if(distance_between_points(target,thisR)<66)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }
                    if(distance_between_points(target,thisR)<33)
                    {
                        control[id].right=control[id].right;
                        control[id].left=control[id].left;
                        control[id].right_rotation=4;
                        control[id].left_rotation=4;
                    }

                }

            }
    }
}

void go_to_pozitie_cu_spatele_portar(RobotCoords thisR,RobotCoords target,RobotCoords pozitie,int id)
{

    RobotCoords zonast,zonadr;

    if(tinta_x==josx)
    {zonast.x=poarta_sus_x;zonadr.x=poarta_sus_x;}
    if(tinta_x=susx)
    {zonast.x=poarta_jos_x;zonadr.x=poarta_jos_x;}
    zonast.y=poarta_stanga_y;
    zonadr.y=poarta_dreapta_y;

    double angle, unghi;
    if(test_if_within_circle(pozitie,thisR,latimer))
    {

        //inainte era 33
        //printf("\nLA TINTA\n");
        angle=unghi_tinta(thisR,target);
        unghi=calculeaza_unghi(angle,thisR.angle);
        //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);


        if(unghi>30&&unghi<=90)
            intoarcere(angle,thisR.angle,id);
        else if(unghi>90&&unghi<160)
            intoarcere_cu_spatele(angle,thisR.angle,id);
        else
            stop(id);
    }

    else
    {

        //printf("CE DRACU FACI?!\n");
        angle=unghi_tinta(thisR,pozitie);
        unghi=calculeaza_unghi(angle,thisR.angle);

        if(unghi>40&&unghi<=90)
        {

            intoarcere(angle,thisR.angle,id);
            /*if(distance_between_points(target,thisR)<66+33)
            {
                control[id].right=control[id].right/1.5;
                control[id].left=control[id].left/1.5;
            }*/
        }
        else if(unghi>90&&unghi<140)
            intoarcere_cu_spatele(angle,thisR.angle,id);
        else if(unghi<=40)
        {
            //printf("merge la pozitie");
            mergi_drept(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(points_on_the_portar(zonast,zonadr,0))
            {
                control[id].left*=2;
                control[id].right*=2;
            }
        }
        else if(unghi>=140)
        {
            //printf("merge la pozitie");
            mergi_cu_spatele(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right<-60)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-80)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-100)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(points_on_the_portar(zonast,zonadr,0))
            {
                control[id].left*=2;
                control[id].right*=2;
            }
        }
    }
}

void go_to_pozitie_portar(RobotCoords thisR,RobotCoords target,RobotCoords pozitie,int id)
{

    RobotCoords zonast,zonadr;

    if(tinta_x==josx)
    {zonast.x=poarta_sus_x;zonadr.x=poarta_sus_x;}
    if(tinta_x=susx)
    {zonast.x=poarta_jos_x;zonadr.x=poarta_jos_x;}
    zonast.y=poarta_stanga_y;
    zonadr.y=poarta_dreapta_y;

    double angle, unghi;
    if(test_if_within_circle(pozitie,thisR,latimer))
    {

        //inainte era 33
        //printf("\nLA TINTA\n");
        angle=unghi_tinta(thisR,target);
        unghi=calculeaza_unghi(angle,thisR.angle);
        //printf("Coords: %d %d    Unghi A: %f    Unghi a: %d\n", coords[id].x,coords[id].y,angle,thisR.angle);


        if(unghi>30)
            intoarcere(angle,thisR.angle,id);
        else
            stop(id);
    }

    else
    {

        //printf("CE DRACU FACI?!\n");
        angle=unghi_tinta(thisR,pozitie);
        unghi=calculeaza_unghi(angle,thisR.angle);

        if(unghi>40)
            intoarcere(angle,thisR.angle,id);

        else
        {
            //printf("merge la pozitie");
            mergi_drept(angle,thisR.angle,id);
            if(distance_between_points(pozitie,thisR)<400)
            {
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }

            }
            if(distance_between_points(pozitie,thisR)<200)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=8;
                control[id].left_rotation=8;
            }
            if(distance_between_points(pozitie,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(pozitie,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(points_on_the_portar(zonast,zonadr,0))
            {
                control[id].left*=2;
                control[id].right*=2;
            }
        }

    }
}

RobotCoords calculate_target(RobotCoords target,int id,int radius)
{
    struct RobotCoords thisR = coords[id];
    struct RobotCoords false_ball;

    if(!test_if_in_right(thisR,left_side,radius))
    {
        false_ball.y=coords[0].y;
        false_ball.x=tinta_x;
        return point_on_the_line(false_ball,coords[0],2*diametrub);
    }


    if(!test_if_in_left(thisR,right_side,radius))
    {
        false_ball.y=coords[0].y;
        false_ball.x=tinta_x;
        return point_on_the_line(false_ball,coords[0],2*diametrub);
    }

    if(!test_if_over(thisR,down_side,radius)&&test_if_in_right(thisR,dreaptay,0))
    {
        if(tinta_x==josx)
        {
            false_ball.y=left_side;
            false_ball.x=coords[0].x;
            return point_on_the_line(false_ball,coords[0],2*diametrub);
        }
        if(tinta_x==susx)
        {
            false_ball.y=right_side;
            false_ball.x=coords[0].x;
            return point_on_the_line(false_ball,coords[0],2*diametrub);
        }
    }
    if(!test_if_over(thisR,down_side,radius)&&test_if_in_left(thisR,stangay,0))
    {
        if(tinta_x==josx)
        {
            false_ball.y=right_side;
            false_ball.x=coords[0].x;
            return point_on_the_line(false_ball,coords[0],2*diametrub);
        }
        if(tinta_x==susx)
        {
            false_ball.y=left_side;
            false_ball.x=coords[0].x;
            return point_on_the_line(false_ball,coords[0],2*diametrub);
        }
    }

    if(!test_if_under(thisR,upper_side,radius)&&test_if_in_right(thisR,dreaptay,0))
    {
        if(tinta_x==josx)
        {
            false_ball.y=right_side;
            false_ball.x=coords[0].x;
            return point_on_the_line(false_ball,coords[0],2*diametrub);
        }
        if(tinta_x==susx)
        {
            false_ball.y=left_side;
            false_ball.x=coords[0].x;
            return point_on_the_line(false_ball,coords[0],2*diametrub);
        }
    }
    if(!test_if_under(thisR,upper_side,radius)&&test_if_in_left(thisR,stangay,0))
    {
        if(tinta_x==josx)
        {
            false_ball.y=left_side;
            false_ball.x=coords[0].x;
            return point_on_the_line(false_ball,coords[0],2*diametrub);
        }
        if(tinta_x==susx)
        {
            false_ball.y=right_side;
            false_ball.x=coords[0].x;
            return point_on_the_line(false_ball,coords[0],2*diametrub);
        }
    }
    return target;
}

void offensive1(int id)
{
    struct RobotCoords thisR = coords[id];
    double angle;
    struct RobotCoords b,poarta,target,target_aux;
    struct RobotCoords coordonate_poarta1,coordonate_poarta2;
    double unghi;

    RobotCoords pozitie;
    pozitie.x=(susx+josx)/2;
    pozitie.y=defense_dreapta;


    coordonate_poarta1.x=tinta_x;
    coordonate_poarta2.x=tinta_x;
    coordonate_poarta1.y=tinta_dreaptay;
    coordonate_poarta2.y=tinta_stangay;
    control[id].time=0;

    poarta=points_on_the_poarta(coordonate_poarta1,coordonate_poarta2,id);
    b=coords[0];


    target=point_on_the_line(poarta,b,2*diametrub);


        if(test_if_under(coords[0],(josx+susx)/2,latimer))
        {
            go_to_pozitie(thisR,target,pozitie,id);
        }
        else
        {
            //inainte era 66
            target_aux=target;
            target=calculate_target(target,0,24);
            if(target.x!=target_aux.x||target.y!=target_aux.y) {poarta=b;}
            //printf("ROBOT:  %d %d\n",thisR.x,thisR.y);
            //printf("BILA:   %d %d\n",b.x,b.y);
            //printf("TARGET: %d %d\n",target.x,target.y);

            if(test_if_under(thisR,danger,0)||test_if_over(thisR,danger_universal,0))
            {
                target.x=(susx+josx)/2;
                target.y=(stangay+dreaptay)/2;
            }



            attack(thisR,poarta,target,id);
        }

        /*
    if(test_if_true_collision(coords[id],id))
        berserk(id);*/

    test_if_within_walls(b,id,2);

}

void offensive2(int id)
 {
    struct RobotCoords thisR = coords[id];
    double angle;
    struct RobotCoords b,poarta,target,false_target,false_poarta,poarta_mea,false_this;
    struct RobotCoords coordonate_poarta1,coordonate_poarta2;
    double unghi;
    struct RobotCoords verticala;
    struct RobotCoords marginest,marginedr;
    struct RobotCoords pozitie;

    pozitie.x=(susx+josx)/2;
    pozitie.y=defense_stanga;

    if(tinta_x==josx)
    {
        if(test_if_over(coords[0],(josx+susx)/2,latimer))
        {

            go_to_pozitie(thisR,target,pozitie,id);
        }
        else
        {
            coordonate_poarta1.x=tinta_x;
            coordonate_poarta2.x=tinta_x;
            coordonate_poarta1.y=tinta_dreaptay;
            coordonate_poarta2.y=tinta_stangay;
            control[id].time=0;

            poarta=points_on_the_poarta(coordonate_poarta1,coordonate_poarta2,id);
            b=coords[0];


            target=point_on_the_line(poarta,b,2*diametrub);
            target=calculate_target(target,0,2);


            false_target.x=target.x;
            false_target.y=0;
            false_poarta.x=poarta.x;
            false_poarta.y=0;
            poarta_mea.x=1400-tinta_x;
            poarta_mea.y=0;
            false_this.x=thisR.x;
            false_this.y=0;
            if(distance_between_points(false_target,false_poarta)<distance_between_points(false_this,poarta_mea))
                verticala=point_on_the_line(false_poarta,false_target,distance_between_points(false_poarta,false_target));
            else
            {
                verticala.y=target.y;
                verticala.x=(target.x+poarta_mea.x)/2;
            }


            marginest.y=left_side;
            marginedr.y=right_side;
            marginest.x=verticala.x;
            marginedr.x=verticala.x;
            if(tinta_x==josx)
            {
                if(test_if_under(coords[0],atac_x,0))
                {
                    pozitie.x=atac_x;
                    pozitie.y=(stangay+dreaptay)/2;
                    target=b;
                }
                if(test_if_over(coords[0],aparare_x,0))
                {
                    pozitie.x=aparare_x;
                    pozitie.y=(stangay+dreaptay)/2;
                    target=b;
                }
                if(!test_if_under(coords[0],linie_atac,0)&&!test_if_over(coords[0],linie_aparare,0))
                    pozitie=points_on_the_verticala(marginest,marginedr,target,id);
            }
            else if(tinta_x==susx)
            {
                if(test_if_over(coords[0],atac_x,0))
                {
                    pozitie.x=atac_x;
                    pozitie.y=(stangay+dreaptay)/2;
                    target=b;
                }
                if(test_if_under(coords[0],aparare_x,0))
                {
                    pozitie.x=aparare_x;
                    pozitie.y=(stangay+dreaptay)/2;
                    //printf("A INTRAT IN ATAC\n");
                    target=b;
                }
                if(!test_if_over(coords[0],linie_atac,0)&&!test_if_under(coords[0],linie_aparare,0))
                pozitie=points_on_the_verticala(marginest,marginedr,target,id);

            }


            //printf("ROBOT:  %d %d\n",thisR.x,thisR.y);
            //printf("BILA:   %d %d\n",b.x,b.y);
            //printf("MARGst: %d %d\n",marginest.x,marginest.y);
            //printf("MARGst: %d %d\n",marginedr.x,marginedr.y);
            //printf("POZITIA:%d %d\n",pozitie.x,pozitie.y);
            //printf("TARGET: %d %d\n",target.x,target.y);

            go_to_pozitie(thisR,target,pozitie,id);
        }
    }
    if(tinta_x==susx)
    {
        if(test_if_under(coords[0],(josx+susx)/2,latimer))
        {
            go_to_pozitie(thisR,target,pozitie,id);
        }
        else
        {
            coordonate_poarta1.x=tinta_x;
            coordonate_poarta2.x=tinta_x;
            coordonate_poarta1.y=tinta_dreaptay;
            coordonate_poarta2.y=tinta_stangay;
            control[id].time=0;

            poarta=points_on_the_poarta(coordonate_poarta1,coordonate_poarta2,id);
            b=coords[0];


            target=point_on_the_line(poarta,b,2*diametrub);
            target=calculate_target(target,0,2);


            false_target.x=target.x;
            false_target.y=0;
            false_poarta.x=poarta.x;
            false_poarta.y=0;
            poarta_mea.x=1400-tinta_x;
            poarta_mea.y=0;
            false_this.x=thisR.x;
            false_this.y=0;
            if(distance_between_points(false_target,false_poarta)<distance_between_points(false_this,poarta_mea))
                verticala=point_on_the_line(false_poarta,false_target,distance_between_points(false_poarta,false_target));
            else
            {
                verticala.y=target.y;
                verticala.x=(target.x+poarta_mea.x)/2;
            }


            marginest.y=left_side;
            marginedr.y=right_side;
            marginest.x=verticala.x;
            marginedr.x=verticala.x;
            if(tinta_x==josx)
            {
                if(test_if_under(coords[0],atac_x,0))
                {
                    pozitie.x=atac_x;
                    pozitie.y=(stangay+dreaptay)/2;
                    target=b;
                }
                if(test_if_over(coords[0],aparare_x,0))
                {
                    pozitie.x=aparare_x;
                    pozitie.y=(stangay+dreaptay)/2;
                    target=b;
                }
                if(!test_if_under(coords[0],linie_atac,0)&&!test_if_over(coords[0],linie_aparare,0))
                    pozitie=points_on_the_verticala(marginest,marginedr,target,id);
            }
            else if(tinta_x==susx)
            {
                if(test_if_over(coords[0],atac_x,0))
                {
                    pozitie.x=atac_x;
                    pozitie.y=(stangay+dreaptay)/2;
                    target=b;
                }
                if(test_if_under(coords[0],aparare_x,0))
                {
                    pozitie.x=aparare_x;
                    pozitie.y=(stangay+dreaptay)/2;
                    //printf("A INTRAT IN ATAC\n");
                    target=b;
                }
                if(!test_if_over(coords[0],linie_atac,0)&&!test_if_under(coords[0],linie_aparare,0))
                pozitie=points_on_the_verticala(marginest,marginedr,target,id);

            }


            //printf("ROBOT:  %d %d\n",thisR.x,thisR.y);
            //printf("BILA:   %d %d\n",b.x,b.y);
            //printf("MARGst: %d %d\n",marginest.x,marginest.y);
            //printf("MARGst: %d %d\n",marginedr.x,marginedr.y);
            //printf("POZITIA:%d %d\n",pozitie.x,pozitie.y);
            //printf("TARGET: %d %d\n",target.x,target.y);

            go_to_pozitie(thisR,target,pozitie,id);
        }
    }

    if(test_if_true_collision(coords[id],id))
        berserk(id);

    test_if_within_walls(pozitie,id,2);

}

void defensive1(int id)
{
    RobotCoords pozitie;
    pozitie.y=defense_dreapta;
    pozitie.x=aparare_x;
    if(tinta_x==susx)
    if(test_if_under(coords[0],susx/2,0))
    {
        RobotCoords poarta_mea;
        poarta_mea.x=josx;
        poarta_mea.y=(stangay+dreaptay)/2;
        pozitie=point_on_the_line(poarta_mea,coords[0],2*diametrub);
        pozitie=point_on_the_line(pozitie,coords[0],2*diametrub);

        pozitie=calculate_target(pozitie,id,24);
        defend_cu_spatele(coords[id],coords[0],pozitie,id);
    }
    else
    {
        if(test_if_over(coords[0],atac_x,0))
        {
            pozitie.y=(defense_dreapta+defense_stanga)/2;
            pozitie.x=(josx+susx)/2;
        }
        go_to_pozitie(coords[id],coords[0],pozitie,id);
    }

    if(tinta_x==josx)
    if(test_if_over(coords[0],susx/2,0))
    {
        RobotCoords poarta_mea;
        poarta_mea.x=josx;
        poarta_mea.y=(stangay+dreaptay)/2;
        pozitie=point_on_the_line(poarta_mea,coords[0],2*diametrub);
        pozitie=point_on_the_line(pozitie,coords[0],2*diametrub);

        pozitie=calculate_target(pozitie,id,24);
        defend_cu_spatele(coords[id],coords[0],pozitie,id);
    }
    else
    {
        if(test_if_under(coords[0],atac_x,0))
        {
            pozitie.y=(defense_dreapta+defense_stanga)/2;
            pozitie.x=(josx+susx)/2;
        }
        go_to_pozitie(coords[id],coords[0],pozitie,id);
    }

    //if(test_if_true_collision(coords[id],id))
    //    berserk(id);
}

void defensive2(int id)
{

    RobotCoords pozitie;
    pozitie.y=defense_stanga;
    pozitie.x=aparare_x;
    if(tinta_x==susx)
    {
        if(test_if_under(coords[0],susx/2,0))
        {
            pozitie.y=(defense_stanga+defense_dreapta)/2;
            go_to_pozitie(coords[id],coords[0],pozitie,id);
        }
        else
        {
            go_to_pozitie(coords[id],coords[0],pozitie,id);
        }
    }
    if(tinta_x==josx)
    {
        if(test_if_over(coords[0],susx/2,0))
        {
            pozitie.y=(defense_stanga+defense_dreapta)/2;
            go_to_pozitie(coords[id],coords[0],pozitie,id);
        }
        else
        {
            go_to_pozitie(coords[id],coords[0],pozitie,id);
        }
    }
    if(test_if_true_collision(coords[id],id))
        berserk(id);

}

void portar(int id)
{
    RobotCoords zonast,zonadr,pozitie,fake_target,predictie;

    if(tinta_x=susx)
    predictie.x=poarta_jos_predictie;
    if(tinta_x==josx)
    predictie.x=poarta_sus_predictie;

    if(coords[0].x==past3[0].x||coords[0].y==past3[0].y) predictie.y=0;
    else
    predictie.y=((predictie.x-coords[0].x)*(past3[0].y-coords[0].y))/(past3[0].x-coords[0].x)+coords[0].y;



    if(tinta_x==josx)
    {zonast.x=poarta_sus_x;zonadr.x=poarta_sus_x;}
    if(tinta_x=susx)
    {zonast.x=poarta_jos_x;zonadr.x=poarta_jos_x;}
    zonast.y=poarta_stanga_y;
    zonadr.y=poarta_dreapta_y;


    fake_target.x=zonast.x;
    fake_target.y=left_side;

    if(predictie.y>poarta_stanga_predictie&&predictie.y<poarta_dreapta_predictie)
    {
        pozitie.x=zonast.x;
        pozitie.y=((pozitie.x-coords[0].x)*(past3[0].y-coords[0].y))/(past3[0].x-coords[0].x)+coords[0].y;
    }
    else
    {
        pozitie.x=zonast.x;
        pozitie.y=coords[0].y;
        if(coords[0].y>poarta_dreapta_y)
            pozitie.y=poarta_dreapta_y;
        if(coords[0].y<poarta_stanga_y)
            pozitie.y=poarta_stanga_y;
    }
go_to_pozitie_cu_spatele_portar(coords[id],fake_target,pozitie,id);

    {
        if(points_on_the_portar(zonast,zonadr,0))
        {
            control[id].left*=2;
            control[id].right*=2;
        }
    }
    if(test_if_within_circle(coords[id],coords[0],razab+2*diametrur))
    {
        if(tinta_x==josx)
        {
            RobotCoords poarta_mea;
            poarta_mea.x=susx;
            poarta_mea.y=(stangay+dreaptay)/2;

            pozitie=point_on_the_line(poarta_mea,coords[0],2*diametrub);
            pozitie=point_on_the_line(pozitie,coords[0],2*diametrub);
            defend(coords[id],coords[0],pozitie,id);
        }
        if(tinta_x==susx)
        {
            RobotCoords poarta_mea;
            poarta_mea.x=josx;
            poarta_mea.y=(stangay+dreaptay)/2;

            pozitie=point_on_the_line(poarta_mea,coords[0],2*diametrub);
            pozitie=point_on_the_line(pozitie,coords[0],2*diametrub);
            defend(coords[id],coords[0],pozitie,id);
        }

    }
    if(!points_on_the_portar(zonast,zonadr,id))
    {
        if(predictie.y>poarta_stanga_predictie&&predictie.y<poarta_dreapta_predictie)
        {
            pozitie.x=zonast.x;
            pozitie.y=((pozitie.x-coords[0].x)*(past3[0].y-coords[0].y))/(past3[0].x-coords[0].x)+coords[0].y;
        }
        else
        {
            pozitie.x=zonast.x;
            pozitie.y=coords[0].y;
            if(coords[0].y>poarta_dreapta_y)
                pozitie.y=poarta_dreapta_y;
            if(coords[0].y<poarta_stanga_y)
                pozitie.y=poarta_stanga_y;
        }
    go_to_pozitie_cu_spatele_portar(coords[id],fake_target,pozitie,id);
        if(points_on_the_portar(zonast,zonadr,0))
        {
            control[id].left*=2;
            control[id].right*=2;
        }
    }

    if(tinta_x==josx)
        if(test_if_under(coords[id],extreme_danger_universal,0))

            {
                pozitie.x=(susx+josx)/2;
                pozitie.y=(stangay+dreaptay)/2;
            }
    if(tinta_x==susx)
        if(test_if_under(coords[id],extreme_danger,0))
        {
            pozitie.x=(susx+josx)/2;
            pozitie.y=(stangay+dreaptay)/2;
        }
/*
    if(test_if_true_collision(coords[id],id))
        berserk(id);*/
    //test_if_within_walls(coords[0],id,2);
}

void calculate_robot_next_movement() {
/*
    if((coords[0].x<reset_jos&&coords[0].y>poarta_stanga_predictie&&coords[0].y<poarta_dreapta_predictie)||(coords[0].x>reset_sus&&coords[0].y>poarta_stanga_predictie&&coords[0].y<poarta_dreapta_predictie))
        reset=1;

    if(!reset)
    {*/

    /*}
    else
    {
        if(o1!=of1) interschimba_ofensiva();
        if(d1!=def1) interschimba_defensiva();
        RobotCoords pozitie,target;
        target.x=(susx+josx)/2;
        target.y=(stangay+dreaptay)/2;
        pozitie.x=po1x;
        pozitie.y=po1y;
        go_to_pozitie_reset(coords[o1],target,pozitie,o1);
        pozitie.x=po2x;
        pozitie.y=po2y;
        go_to_pozitie_reset(coords[o2],target,pozitie,o2);
        pozitie.x=pd1x;
        pozitie.y=pd1y;
        go_to_pozitie_reset(coords[d1],target,pozitie,d1);
        pozitie.x=pd2x;
        pozitie.y=pd2y;
        go_to_pozitie_reset(coords[d2],target,pozitie,d2);
        pozitie.x=ppx;
        pozitie.y=ppy;
        go_to_pozitie_reset(coords[p],target,pozitie,p);
        if(test_if_true_collision(coords[o1],o1))
            berserk(o1);
        if(test_if_true_collision(coords[o2],o2))
            berserk(o2);
        if(test_if_true_collision(coords[d1],d1))
            berserk(d1);
        if(test_if_true_collision(coords[d2],d2))
            berserk(d2);
        if(test_if_true_collision(coords[p],p))
            berserk(p);
    }*/
/*
    struct RobotCoords thisR=coords[o1];
    int id=o1;
    struct RobotCoords target;
    double angle, unghi;

    target.x=target_x;
    target.y=target_y;
    if(test_if_within_circle(coords[o1],target,2*diametrur))
    {
        target_x=abs(1400-target_x);
    }

    angle=unghi_tinta(thisR,target);
    unghi=calculeaza_unghi(angle,thisR.angle);
*/

    //
    //printf("CE DRACU FACI?!\n");
    //

/*
    if(unghi>40&&unghi<=90)
    {
        intoarcere(angle,thisR.angle,id);

    }

    else if(unghi>90&&unghi<140)
    {
        intoarcere_cu_spatele(angle,thisR.angle,id);
    }
    else if(unghi<=40)
    {

            mergi_drept(angle,thisR.angle,id);
            if(distance_between_points(coords[0],thisR)<500)
            {
                if(control[id].right>60)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>80)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }
                if(control[id].right>100)
                {
                    control[id].right=control[id].right-10;
                    control[id].left=control[id].left-10;
                }

            }
            if(distance_between_points(coords[0],thisR)<200)
            {
                //control[id].right=control[id].right;
                //control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(target,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(target,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=3;
                control[id].left_rotation=3;
            }
            if(distance_between_points(target,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=2;
                control[id].left_rotation=2;
            }

        }
    else if(unghi>=140)
    {

            mergi_cu_spatele(angle,thisR.angle,id);
            if(distance_between_points(coords[0],thisR)<500)
            {
                if(control[id].right<-60)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-80)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }
                if(control[id].right<-100)
                {
                    control[id].right=control[id].right+10;
                    control[id].left=control[id].left+10;
                }

            }
            if(distance_between_points(coords[0],thisR)<200)
            {
                //control[id].right=control[id].right;
                //control[id].left=control[id].left;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(target,thisR)<66+33)
            {
                //control[id].right=control[id].right-40;
                //control[id].left=control[id].left-40;
                control[id].right_rotation=4;
                control[id].left_rotation=4;
            }
            if(distance_between_points(target,thisR)<66)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=3;
                control[id].left_rotation=3;
            }
            if(distance_between_points(target,thisR)<33)
            {
                control[id].right=control[id].right;
                control[id].left=control[id].left;
                control[id].right_rotation=2;
                control[id].left_rotation=2;
            }

        }
*/

/*
    if(unghi>40)
    {
        intoarcere(angle,thisR.angle,id);

    }
    else
    {
        mergi_drept(angle,thisR.angle,id);
        if(distance_between_points(target,thisR)<66+33)
        {
            control[id].right=control[id].right;
            control[id].left=control[id].left;
            control[id].right_rotation=4;
            control[id].left_rotation=4;
        }
    }

*/
    //printf("%d\n",coords[o1].angle);
    printf("x:%d y:%d\n",coords[o1].x,coords[o1].y);
    //printf("x:%d y:%d\n",coords[o2].x,coords[o2].y);
    //printf("x:%d y:%d\n",coords[d1].x,coords[d1].y);
    //printf("x:%d y:%d\n",coords[d2].x,coords[d2].y);
    //printf("x:%d y:%d\n",coords[p].x,coords[p].y);
    //printf("%d %d\n",coords[0].x,coords[0].y);
    //printf("%d\n",coords[o1].timestamp);
    //printf("%d %d\n",coords[o2].x,coords[o2].y);

}

void publish(int id)
{
    int mid;
    fflush(stdout);
    if(id==o1) mosquitto_publish(mosq, &mid, id_o1, sizeof(struct RobotControl), &control[o1], 0, false);
    if(id==o2) mosquitto_publish(mosq, &mid, id_o2, sizeof(struct RobotControl), &control[o2], 0, false);
    if(id==d1) mosquitto_publish(mosq, &mid, id_d1, sizeof(struct RobotControl), &control[d1], 0, false);
    if(id==d2) mosquitto_publish(mosq, &mid, id_d2, sizeof(struct RobotControl), &control[d2], 0, false);
    if(id==p) mosquitto_publish(mosq, &mid, id_p, sizeof(struct RobotControl), &control[p], 0, false);
    //gettimeofday(&tv, NULL);
}

void do_robot_control_loop() {
    int mid;
    fflush(stdout);

    calculate_robot_next_movement();

    mosquitto_publish(mosq, &mid, id_o1, sizeof(struct RobotControl), &control[o1], 0, false);
    mosquitto_publish(mosq, &mid, id_o2, sizeof(struct RobotControl), &control[o2], 0, false);
    mosquitto_publish(mosq, &mid, id_d1, sizeof(struct RobotControl), &control[d1], 0, false);
    mosquitto_publish(mosq, &mid, id_d2, sizeof(struct RobotControl), &control[d2], 0, false);
    mosquitto_publish(mosq, &mid, id_p, sizeof(struct RobotControl), &control[p], 0, false);
    gettimeofday(&tv, NULL);
}

int need_to_send() {
    struct timeval now;

    gettimeofday(&now, NULL);

    if(tv.tv_usec > 800000)
        gettimeofday(&tv, NULL);

    if(now.tv_usec >= tv.tv_usec + 100000) {
            return 1;
    }

    return 0;
}

int main()
{
    int rc = 0;
    int mid;

    init();
    //memorare();


    mosquitto_lib_init();
    mosq = mosquitto_new(clientid, true, NULL);
    if(mosq){
        mosquitto_connect_callback_set(mosq, connect_callback);
        mosquitto_message_callback_set(mosq, message_callback);

        rc = mosquitto_connect(mosq, mqtt_host, mqtt_port, 60);

        mosquitto_subscribe(mosq, NULL, "coords", 0);
        gettimeofday(&tv, NULL);

        while(run){

          //  calculate_robot_next_movement();

            if(need_to_send()) {
                do_robot_control_loop();
            }

            rc = mosquitto_loop(mosq, 1, 50);
            if(run && rc){
                sleep(1);
                mosquitto_reconnect(mosq);
            }
        }
        mosquitto_destroy(mosq);
    }

    mosquitto_lib_cleanup();

    return rc;
}

