// Left-hand traffic 4-way intersection. Needs raylib + raygui.h
// gcc traffic_sim.c -o sim -lraylib -lm -lGL -lpthread -ldl -lrt -lX11
#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#define SZ 900
#define LW 50.0f
#define SO 10.0f
#define SD 480.0f
#define YT 5.0f
#define AR 2.0f
#define ACC 140.0f
#define BRK 260.0f
#define GAP 8.0f
#define TSPD 75.0f
#define LOOK 160.0f
#define BN 16
#define NP (BN+3)
enum{LRED,LYEL,LGRN};enum{ST_G,ST_Y,ST_R};
typedef struct{Vector2 p[NP];float c[NP],tot,sIn,sOut,sStop;}Path;
typedef struct Car{int road,turn,yd;const Path*pa;float s,v,vm,len,wid;Color col;Vector2 pos,hd;struct Car*nx;}Car;
typedef struct{int waiting[4];float lastGreen[4];int cycle;}Snap;
typedef struct{int act,stage,cycle;float timer,gt[4];}Ctl;
// EXTERNAL HOOK: return green seconds for 'road' (placeholder: queue based)
float External_GetGreenTime(int road,const Snap*s){float t=5+s->waiting[road];return t<8?8:t>30?30:t;}
static const Vector2 C0={SZ/2.0f,SZ/2.0f},DIR[4]={{0,-1},{1,0},{0,1},{-1,0}};
static const char*RN[4]={"North","East","South","West"};
static const Color COL[]={{220,60,60,255},{60,120,220,255},{240,200,50,255},{80,200,120,255},{230,230,235,255},{160,90,210,255}};
static Path P[4][3];static Car*cars;static int nc;
static float RF(float a,float b){return a+(b-a)*GetRandomValue(0,10000)/10000.0f;}
static Vector2 Lf(Vector2 h){return(Vector2){h.y,-h.x};}
static Vector2 Pt(Vector2 o,Vector2 a,float da,Vector2 b,float db){return(Vector2){o.x+a.x*da+b.x*db,o.y+a.y*da+b.y*db};}
static void Build(void){for(int d=0;d<4;d++){Vector2 u=DIR[d],hi=Vector2Scale(u,-1),li=Lf(hi);for(int t=0;t<3;t++){
 Vector2 w=t==0?hi:t==1?li:Vector2Scale(li,-1);int e=0;for(int j=0;j<4;j++)if(Vector2DotProduct(DIR[j],w)>.5f)e=j;
 Vector2 ho=DIR[e],lo=Lf(ho),p0=Pt(C0,u,SD,li,LW/2),p1=Pt(C0,u,LW,li,LW/2),p2=Pt(C0,ho,LW,lo,LW/2),pe=Pt(C0,ho,SD,lo,LW/2),k;
 k=t==0?Vector2Lerp(p1,p2,.5f):Pt(p1,hi,Vector2DotProduct(Vector2Subtract(p2,p1),hi),hi,0);
 Path*a=&P[d][t];a->p[0]=p0;
 for(int i=0;i<=BN;i++){float q=(float)i/BN,m=1-q;a->p[1+i]=(Vector2){m*m*p1.x+2*m*q*k.x+q*q*p2.x,m*m*p1.y+2*m*q*k.y+q*q*p2.y};}
 a->p[NP-1]=pe;a->c[0]=0;for(int i=1;i<NP;i++)a->c[i]=a->c[i-1]+Vector2Distance(a->p[i-1],a->p[i]);
 a->tot=a->c[NP-1];a->sIn=a->c[1];a->sOut=a->c[BN+1];a->sStop=a->sIn-SO;}}}
static Vector2 PosAt(const Path*p,float s){if(s<=0)return p->p[0];if(s>=p->tot)return p->p[NP-1];int i=0;while(i<NP-2&&s>p->c[i+1])i++;
 float g=p->c[i+1]-p->c[i];return Vector2Lerp(p->p[i],p->p[i+1],g>0?(s-p->c[i])/g:0);}
static void Pose(Car*c){const Path*p=c->pa;c->pos=PosAt(p,c->s);float q=fminf(fmaxf(c->s,4),p->tot-4);
 c->hd=Vector2Normalize(Vector2Subtract(PosAt(p,q+4),PosAt(p,q-4)));}
static int Sig(const Ctl*k,int r){return r!=k->act?LRED:k->stage==ST_G?LGRN:k->stage==ST_Y?LYEL:LRED;}
static void Refresh(Ctl*k){Snap s={0};s.cycle=k->cycle;for(Car*c=cars;c;c=c->nx)if(c->s+c->len/2<c->pa->sIn)s.waiting[c->road]++;
 for(int r=0;r<4;r++)s.lastGreen[r]=k->gt[r];for(int r=0;r<4;r++){float t=External_GetGreenTime(r,&s);k->gt[r]=t>0?t:0;}}
static void UpdCtl(Ctl*k,float dt){k->timer-=dt;while(k->timer<=0){
 if(k->stage==ST_G){k->stage=ST_Y;k->timer+=YT;}else if(k->stage==ST_Y){k->stage=ST_R;k->timer+=AR;}
 else{k->act=(k->act+1)%4;if(!k->act){k->cycle++;Refresh(k);}k->stage=ST_G;k->timer+=k->gt[k->act];}}}
static void Spawn(int r){for(Car*o=cars;o;o=o->nx)if(o->road==r&&o->s<90)return;Car*c=calloc(1,sizeof(Car));if(!c)return;
 int q=GetRandomValue(0,99);c->turn=q<50?0:q<75?1:2;c->road=r;c->pa=&P[r][c->turn];c->len=RF(32,44);c->wid=20;c->vm=RF(100,150);
 c->v=c->vm/2;c->col=COL[GetRandomValue(0,5)];Pose(c);c->nx=cars;cars=c;nc++;}
static int Must(Car*c,const Ctl*k,float g){int s=Sig(k,c->road);if(s==LGRN){c->yd=0;return 0;}
 if(s==LYEL){if(!c->yd)c->yd=g+1>=c->v*c->v/(2*BRK)?1:2;return c->yd==1;}return c->yd!=2;}
static int Lead(const Car*c,float*gp,float*vl){float b=1e9f;int f=0;for(const Car*o=cars;o;o=o->nx){if(o==c)continue;float g;
 if(o->pa==c->pa||(o->road==c->road&&c->s<=c->pa->sIn)){if(o->s<=c->s)continue;g=o->s-c->s-(c->len+o->len)/2;}
 else{Vector2 d=Vector2Subtract(o->pos,c->pos);float fw=Vector2DotProduct(d,c->hd);if(fw<=0||fw>LOOK||fabsf(d.x*c->hd.y-d.y*c->hd.x)>LW*.4f)continue;g=fw-(c->len+o->len)/2;}
 if(g<b){b=g;*vl=o->v;f=1;}}*gp=b;return f;}
static void UpdCar(Car*c,float dt,const Ctl*k){const Path*p=c->pa;float vt=c->vm,g,vl=0,gs=p->sStop-c->s-c->len/2;
 if(c->turn&&c->s>p->sIn-60&&c->s<p->sOut+c->len)vt=fminf(vt,TSPD);
 if(gs>-2&&Must(c,k,gs))vt=fminf(vt,sqrtf(2*BRK*fmaxf(gs,0)));
 if(Lead(c,&g,&vl))vt=fminf(vt,sqrtf(vl*vl+2*BRK*fmaxf(g-GAP,0)));
 c->v=c->v<vt?fminf(vt,c->v+ACC*dt):fmaxf(vt,c->v-1.5f*BRK*dt);c->s+=c->v*dt;Pose(c);}
static void UpdCars(float dt,const Ctl*k){Car**pp=&cars;while(*pp){Car*c=*pp;UpdCar(c,dt,k);Vector2 q=c->pos;
 if(c->s>=c->pa->tot||(c->s>c->pa->sOut&&(q.x<-24||q.x>SZ+24||q.y<-24||q.y>SZ+24))){*pp=c->nx;free(c);nc--;}else pp=&c->nx;}}
static void DrawRoads(void){Color as={55,58,64,255};ClearBackground((Color){34,85,45,255});
 DrawRectangle(SZ/2-LW,0,2*LW,SZ,as);DrawRectangle(0,SZ/2-LW,SZ,2*LW,as);
 for(int d=0;d<4;d++){Vector2 u=DIR[d],li=Lf(Vector2Scale(u,-1)),b=Pt(C0,u,LW+SO,li,0);
  DrawLineEx(Pt(C0,u,LW,u,0),Pt(C0,u,700,u,0),2,GOLD);DrawLineEx(b,Pt(b,li,LW,li,0),4,WHITE);}}
static void DrawSig(int d,const Ctl*k){Vector2 u=DIR[d],c=Pt(C0,u,100,Lf(Vector2Scale(u,-1)),88);int s=Sig(k,d);
 DrawRectangleRounded((Rectangle){c.x-12,c.y-32,24,64},.3f,6,(Color){20,20,24,255});
 DrawCircleV((Vector2){c.x,c.y-20},8,s==LRED?RED:Fade(RED,.25f));DrawCircleV((Vector2){c.x,c.y},8,s==LYEL?YELLOW:Fade(YELLOW,.25f));
 DrawCircleV((Vector2){c.x,c.y+20},8,s==LGRN?GREEN:Fade(GREEN,.25f));
 const char*t=d==k->act&&k->stage!=ST_R?TextFormat("%s %ds",RN[d],(int)ceilf(k->timer)):RN[d];
 DrawText(t,c.x<C0.x?(int)c.x-18-MeasureText(t,18):(int)c.x+18,(int)c.y-9,18,RAYWHITE);}
static void Hud(const Ctl*k,int ps){DrawRectangle(8,8,450,88,Fade(BLACK,.6f));
 DrawText(TextFormat("Cycle %d | %s: %s %.1fs",k->cycle+1,RN[k->act],k->stage==ST_G?"GREEN":k->stage==ST_Y?"YELLOW":"ALL RED",k->timer),16,14,18,RAYWHITE);
 DrawText(TextFormat("Green: N %.0fs  E %.0fs  S %.0fs  W %.0fs",k->gt[0],k->gt[1],k->gt[2],k->gt[3]),16,38,18,LIGHTGRAY);
 DrawText(TextFormat("Cars alive: %d | [SPACE] %s",nc,ps?"resume":"pause"),16,62,16,LIGHTGRAY);}
int main(void){InitWindow(SZ,SZ,"Intersection (left-hand traffic)");SetTargetFPS(60);SetRandomSeed((unsigned)time(NULL));
 GuiSetStyle(DEFAULT,TEXT_SIZE,14);Build();Ctl k={0};Refresh(&k);k.timer=k.gt[0];
 float sp[4],dn[4]={1,1,1,1};int ps=0;for(int r=0;r<4;r++)sp[r]=RF(.2f,3);
 while(!WindowShouldClose()){float dt=fminf(GetFrameTime(),.05f);if(IsKeyPressed(KEY_SPACE))ps=!ps;
  if(!ps){UpdCtl(&k,dt);for(int r=0;r<4;r++){sp[r]-=dt*dn[r];if(sp[r]<=0){Spawn(r);sp[r]=RF(3,7);}}UpdCars(dt,&k);}
  BeginDrawing();DrawRoads();for(Car*c=cars;c;c=c->nx)DrawRectanglePro((Rectangle){c->pos.x,c->pos.y,c->len,c->wid},
   (Vector2){c->len/2,c->wid/2},atan2f(c->hd.y,c->hd.x)*RAD2DEG,c->col);
  for(int d=0;d<4;d++)DrawSig(d,&k);Hud(&k,ps);DrawRectangle(8,100,250,104,Fade(RAYWHITE,.85f));
  for(int r=0;r<4;r++)GuiSliderBar((Rectangle){70,108+24*r,120,18},RN[r],TextFormat("%.1f",dn[r]),&dn[r],0,4);EndDrawing();}
 while(cars){Car*n=cars->nx;free(cars);cars=n;}CloseWindow();return 0;}
