/* Vita League 3D - primera integracion del motor rlphys.h
 * VitaSDK + VitaGL. Coordenadas del motor: X/Y en el suelo, Z arriba.
 */
#include <vitaGL.h>
#include <psp2/ctrl.h>
#include <psp2/kernel/processmgr.h>
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "rlphys.h"

#define SCR_W 960
#define SCR_H 544
#define DT (1.0f/60.0f)
#define WIN_SCORE 5
#define PI 3.14159265f
#define SCALE 0.01f
#define HX (ARENA_HX*SCALE)
#define HY (ARENA_HY*SCALE)
#define GZ (GOAL_HW*SCALE)
#define GD (GOAL_D*SCALE)
#define CLAMP(v,a,b) ((v)<(a)?(a):((v)>(b)?(b):(v)))

enum { ST_MENU, ST_CONTROLS, ST_COUNT, ST_PLAY, ST_GOAL, ST_OVER };
static const char *DIFF_NAME[3] = {"FACIL", "NORMAL", "DIFICIL"};
static const float AI_POWER[3] = {0.72f, 0.90f, 1.05f};
static float ball_rot[9] = {1,0,0,0,1,0,0,0,1};

static float wrap_angle(float a) { while(a>PI)a-=2*PI; while(a<-PI)a+=2*PI; return a; }
static float car_yaw(const Car *c) { return atan2f(c->f.y,c->f.x); }
static void ball_roll(const Ball *b) {
    float vx=b->v.x*SCALE, vy=b->v.y*SCALE, sp=sqrtf(vx*vx+vy*vy);
    if(sp<0.001f)return;
    float radius=BALL_R*SCALE, ang=sp*DT/radius;
    float ax=vy/sp, ay=0, az=-vx/sp, co=cosf(ang), si=sinf(ang), t=1-co;
    float d[9]={co+ax*ax*t,ax*ay*t-az*si,ax*az*t+ay*si,
                ay*ax*t+az*si,co+ay*ay*t,ay*az*t-ax*si,
                az*ax*t-ay*si,az*ay*t+ax*si,co+az*az*t};
    float r[9];
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)
        r[i*3+j]=d[i*3]*ball_rot[j]+d[i*3+1]*ball_rot[3+j]+d[i*3+2]*ball_rot[6+j];
    memcpy(ball_rot,r,sizeof(r));
}

static void begin3d(void) {
    float n=1.0f, f=250.0f, t=n*tanf(32.5f*PI/180.0f), r=t*(float)SCR_W/SCR_H;
    glDisable(GL_BLEND); glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glFrustum(-r,r,-t,t,n,f);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
}
static void look_at(float ex,float ey,float ez,float cx,float cy,float cz) {
    float fx=cx-ex,fy=cy-ey,fz=cz-ez,fl=sqrtf(fx*fx+fy*fy+fz*fz);
    if(fl<0.0001f)fl=1;
    fx/=fl;fy/=fl;fz/=fl;
    float sx=-fz,sy=0,sz=fx,sl=sqrtf(sx*sx+sz*sz); if(sl<0.0001f)sl=1;
    sx/=sl;sz/=sl;
    float ux=sy*fz-sz*fy,uy=sz*fx-sx*fz,uz=sx*fy-sy*fx;
    float m[16]={sx,ux,-fx,0, sy,uy,-fy,0, sz,uz,-fz,0, 0,0,0,1};
    glMultMatrixf(m); glTranslatef(-ex,-ey,-ez);
}
static void box(float x,float y,float z,float hx,float hy,float hz,float r,float g,float b) {
    float x0=x-hx,x1=x+hx,y0=y-hy,y1=y+hy,z0=z-hz,z1=z+hz;
    glBegin(GL_QUADS); glColor3f(r,g,b);
    glVertex3f(x0,y1,z0);glVertex3f(x0,y1,z1);glVertex3f(x1,y1,z1);glVertex3f(x1,y1,z0);
    glColor3f(r*.8f,g*.8f,b*.8f);
    glVertex3f(x1,y0,z0);glVertex3f(x1,y1,z0);glVertex3f(x1,y1,z1);glVertex3f(x1,y0,z1);
    glVertex3f(x0,y0,z1);glVertex3f(x0,y1,z1);glVertex3f(x0,y1,z0);glVertex3f(x0,y0,z0);
    glColor3f(r*.62f,g*.62f,b*.62f);
    glVertex3f(x0,y0,z1);glVertex3f(x1,y0,z1);glVertex3f(x1,y1,z1);glVertex3f(x0,y1,z1);
    glVertex3f(x1,y0,z0);glVertex3f(x0,y0,z0);glVertex3f(x0,y1,z0);glVertex3f(x1,y1,z0);
    glEnd();
}
static void flat(float x0,float z0,float x1,float z1,float y,float r,float g,float b) {
    glBegin(GL_QUADS);glColor3f(r,g,b);
    glVertex3f(x0,y,z0);glVertex3f(x0,y,z1);glVertex3f(x1,y,z1);glVertex3f(x1,y,z0);glEnd();
}
static void shadow(float x,float z,float rad) {
    glBegin(GL_TRIANGLE_FAN);glColor3f(.06f,.12f,.07f);glVertex3f(x,.035f,z);
    for(int i=0;i<=16;i++){float a=i*2*PI/16;glVertex3f(x+cosf(a)*rad,.035f,z+sinf(a)*rad);}glEnd();
}
static void draw_arena(void) {
    flat(-200,-200,200,200,-.25f,.12f,.15f,.19f);
    for(int i=0;i<16;i++){float x0=-HX+i*(2*HX/16),x1=-HX+(i+1)*(2*HX/16);float c=(i&1)?.19f:.22f;flat(x0,-HY,x1,HY,0,c,.48f,c+.04f);}
    flat(-.12f,-HY,.12f,HY,.025f,.9f,.9f,.9f);
    flat(-HX-GD,-GZ,-HX,GZ,.03f,.12f,.28f,.8f);flat(HX,-GZ,HX+GD,GZ,.03f,.85f,.4f,.08f);
    /* Bordes y techo: la colision real ya la resuelve arena() en rlphys.h. */
    box(0,1.7f,-HY-.35f,HX,.35f,.35f,.35f,.42f,.52f);
    box(0,1.7f, HY+.35f,HX,.35f,.35f,.35f,.42f,.52f);
    float side=(HY-GZ)*.5f;
    box(-HX-.35f,1.7f,-(HY+GZ)*.5f,.35f,.35f,side,.35f,.42f,.52f);
    box(-HX-.35f,1.7f, (HY+GZ)*.5f,.35f,.35f,side,.35f,.42f,.52f);
    box(HX+.35f,1.7f,-(HY+GZ)*.5f,.35f,.35f,side,.35f,.42f,.52f);
    box(HX+.35f,1.7f, (HY+GZ)*.5f,.35f,.35f,side,.35f,.42f,.52f);
    box(-HX-GD-.35f,1.7f,0,.35f,.35f,GZ,.12f,.28f,.8f);
    box( HX+GD+.35f,1.7f,0,.35f,.35f,GZ,.85f,.4f,.08f);
    box(-HX-GD*.5f,1.7f,-GZ-.25f,GD*.5f,.35f,.25f,.12f,.28f,.8f);
    box(-HX-GD*.5f,1.7f, GZ+.25f,GD*.5f,.35f,.25f,.12f,.28f,.8f);
    box( HX+GD*.5f,1.7f,-GZ-.25f,GD*.5f,.35f,.25f,.85f,.4f,.08f);
    box( HX+GD*.5f,1.7f, GZ+.25f,GD*.5f,.35f,.25f,.85f,.4f,.08f);
    box(-HX,3.25f,0,.22f,1.55f,.22f,.25f,.48f,1);box(-HX,6.4f,0,.22f,.18f,GZ,.25f,.48f,1);
    box(HX,3.25f,0,.22f,1.55f,.22f,1,.55f,.12f);box(HX,6.4f,0,.22f,.18f,GZ,1,.55f,.12f);
}
static void draw_car(const Car *c,float r,float g,float b,float now) {
    float x=c->p.x*SCALE,z=c->p.y*SCALE,y=c->p.z*SCALE;
    shadow(x,z,1.7f);glPushMatrix();glTranslatef(x,y,z);glRotatef(-car_yaw(c)*57.29578f,0,1,0);
    box(0,.75f,0,1.18f,.35f,.84f,r,g,b);box(-.15f,1.28f,0,.62f,.22f,.68f,r*.7f,g*.7f,b*.7f);
    box(.48f,1.28f,0,.1f,.18f,.55f,.75f,.9f,1);box(-1.1f,1.3f,0,.1f,.1f,.82f,r*.5f,g*.5f,b*.5f);
    box(.8f,.38f,.88f,.28f,.38f,.16f,.06f,.06f,.06f);box(.8f,.38f,-.88f,.28f,.38f,.16f,.06f,.06f,.06f);
    box(-.8f,.38f,.88f,.28f,.38f,.16f,.06f,.06f,.06f);box(-.8f,.38f,-.88f,.28f,.38f,.16f,.06f,.06f,.06f);
    if(c->boosting){float f=.8f+.2f*sinf(now*35);box(-1.75f,.7f,0,.48f*f,.17f,.18f,1,.5f,.08f);}
    glPopMatrix();
}
static void draw_ball(const Ball *b) {
    float R=BALL_R*SCALE,x=b->p.x*SCALE,y=b->p.z*SCALE,z=b->p.y*SCALE;shadow(x,z,R*1.1f);
    glPushMatrix();glTranslatef(x,y,z);
    float m[16]={ball_rot[0],ball_rot[3],ball_rot[6],0,ball_rot[1],ball_rot[4],ball_rot[7],0,ball_rot[2],ball_rot[5],ball_rot[8],0,0,0,0,1};glMultMatrixf(m);
    const int LAT=8,LON=14;glBegin(GL_QUADS);
    for(int i=0;i<LAT;i++){float a0=-PI/2+PI*i/LAT,a1=-PI/2+PI*(i+1)/LAT;for(int j=0;j<LON;j++){float b0=2*PI*j/LON,b1=2*PI*(j+1)/LON;glColor3f(((i+j)&1)?.92f:.23f,((i+j)&1)?.92f:.26f,((i+j)&1)?.92f:.32f);
    glVertex3f(R*cosf(a0)*cosf(b0),R*sinf(a0),R*cosf(a0)*sinf(b0));glVertex3f(R*cosf(a1)*cosf(b0),R*sinf(a1),R*cosf(a1)*sinf(b0));glVertex3f(R*cosf(a1)*cosf(b1),R*sinf(a1),R*cosf(a1)*sinf(b1));glVertex3f(R*cosf(a0)*cosf(b1),R*sinf(a0),R*cosf(a0)*sinf(b1));}}
    glEnd();glPopMatrix();
}

static const char FONT_CHARS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:-!./<> ";
static const unsigned char FONT[][7] = {
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}, {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, /* A B */
    {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}, {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}, /* C D */
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}, {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}, /* E F */
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}, {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}, /* G H */
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, {0x07,0x02,0x02,0x02,0x02,0x12,0x0C}, /* I J */
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11}, {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}, /* K L */
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}, {0x11,0x19,0x15,0x13,0x11,0x11,0x11}, /* M N */
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}, /* O P */
    {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}, {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}, /* Q R */
    {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}, {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, /* S T */
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}, /* U V */
    {0x11,0x11,0x11,0x15,0x15,0x15,0x0A}, {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, /* W X */
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}, /* Y Z */
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, /* 0 1 */
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}, {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}, /* 2 3 */
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, /* 4 5 */
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, /* 6 7 */
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, /* 8 9 */
    {0x00,0x04,0x04,0x00,0x04,0x04,0x00}, {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, /* : - */
    {0x04,0x04,0x04,0x04,0x04,0x00,0x04}, {0x00,0x00,0x00,0x00,0x00,0x06,0x06}, /* ! . */
    {0x01,0x02,0x02,0x04,0x08,0x08,0x10}, {0x02,0x04,0x08,0x10,0x08,0x04,0x02}, /* / < */
    {0x08,0x04,0x02,0x01,0x02,0x04,0x08}, {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* > espacio */
};

static void begin2d(void) {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, SCR_W, SCR_H, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

static void rect2d(float x, float y, float w, float h, float r, float g, float b, float a) {
    glBegin(GL_QUADS);
    glColor4f(r, g, b, a);
    glVertex2f(x, y); glVertex2f(x + w, y); glVertex2f(x + w, y + h); glVertex2f(x, y + h);
    glEnd();
}

static float text_w(float s, const char *t) { return (float)strlen(t) * 6.0f * s - s; }

static void text_raw(float x, float y, float s, float r, float g, float b, const char *t) {
    glColor4f(r, g, b, 1.0f);
    glBegin(GL_QUADS);
    for (; *t; t++, x += 6.0f * s) {
        const char *p = strchr(FONT_CHARS, toupper((unsigned char)*t));
        if (!p) continue;
        int idx = (int)(p - FONT_CHARS);
        for (int row = 0; row < 7; row++)
            for (int col = 0; col < 5; col++)
                if (FONT[idx][row] & (1 << (4 - col))) {
                    float px = x + col * s, py = y + row * s;
                    glVertex2f(px, py); glVertex2f(px + s, py);
                    glVertex2f(px + s, py + s); glVertex2f(px, py + s);
                }
    }
    glEnd();
}

static void text(float x, float y, float s, float r, float g, float b, const char *t) {
    text_raw(x + s * 0.6f, y + s * 0.6f, s, 0.0f, 0.0f, 0.0f, t);
    text_raw(x, y, s, r, g, b, t);
}

static void text_c(float y, float s, float r, float g, float b, const char *t) {
    text((SCR_W - text_w(s, t)) * 0.5f, y, s, r, g, b, t);
}


static void ai_input(const Car *ai,const Ball *ball,int diff,Input *out) {
    memset(out,0,sizeof(*out));
    V3 target=v3(ball->p.x,ball->p.y,0), delta=vsub(target,ai->p);float len=vlen(delta);
    if(len>220.0f){target=vsub(target,vmul(vnorm(delta),180.0f));delta=vsub(target,ai->p);}
    float desired=atan2f(delta.y,delta.x),err=wrap_angle(desired-car_yaw(ai));
    out->steer=CLAMP(err*2.0f,-1.0f,1.0f);out->throttle=fabsf(err)<1.25f?1.0f:.35f;
    out->boost=(fabsf(err)<.22f&&len>1700.0f&&ai->boost>25.0f&&diff>0);
}
static void reset_round(World *w) { world_reset(w); pads_reset(w); memset(ball_rot,0,sizeof(ball_rot));ball_rot[0]=ball_rot[4]=ball_rot[8]=1; }

int main(void) {
    vglInit(0x800000);glClearColor(.42f,.60f,.85f,1);glViewport(0,0,SCR_W,SCR_H);sceCtrlSetSamplingMode(SCE_CTRL_MODE_ANALOG);
    World world;reset_round(&world);
    int state=ST_MENU,sel=0,diff=1,quit=0,score_p=0,score_a=0,last_scorer=0,ballcam=1;
    float timer=0,menu_t=0,cam_yaw=0,now=0;unsigned int prev=0,prev_nav=0;char buf[64];
    while(!quit){
        SceCtrlData pad;sceCtrlPeekBufferPositive(0,&pad,1);unsigned int pressed=pad.buttons&~prev;prev=pad.buttons;
        unsigned int nav=pad.buttons;if(pad.ly<60)nav|=SCE_CTRL_UP;if(pad.ly>196)nav|=SCE_CTRL_DOWN;if(pad.lx<60)nav|=SCE_CTRL_LEFT;if(pad.lx>196)nav|=SCE_CTRL_RIGHT;
        unsigned int npress=nav&~prev_nav;prev_nav=nav;now+=DT;
        switch(state){
        case ST_MENU: menu_t+=DT;if(npress&SCE_CTRL_UP)sel=(sel+3)%4;if(npress&SCE_CTRL_DOWN)sel=(sel+1)%4;
            if(sel==1&&(npress&SCE_CTRL_LEFT)) diff=(diff+2)%3;
            if(sel==1&&(npress&SCE_CTRL_RIGHT)) diff=(diff+1)%3;
            if(pressed&SCE_CTRL_CROSS){if(sel==0){score_p=score_a=0;reset_round(&world);state=ST_COUNT;timer=3;}else if(sel==1)diff=(diff+1)%3;else if(sel==2)state=ST_CONTROLS;else quit=1;}break;
        case ST_CONTROLS:menu_t+=DT;if(pressed&(SCE_CTRL_CIRCLE|SCE_CTRL_CROSS|SCE_CTRL_START))state=ST_MENU;break;
        case ST_COUNT:timer-=DT;if(timer<=0)state=ST_PLAY;if(pressed&SCE_CTRL_START)state=ST_MENU;break;
        case ST_PLAY:{
            Input p={0},a={0};p.steer=CLAMP(((int)pad.lx-128)/128.0f,-1,1);if(fabsf(p.steer)<.12f)p.steer=0;
            if(pad.buttons&SCE_CTRL_LEFT) p.steer=-1;
            if(pad.buttons&SCE_CTRL_RIGHT) p.steer=1;
            if(pad.buttons&(SCE_CTRL_RTRIGGER|SCE_CTRL_CROSS))p.throttle=1;
            if(pad.buttons&SCE_CTRL_LTRIGGER)p.throttle=-1;
            p.boost=(pad.buttons&SCE_CTRL_CIRCLE)!=0;p.jump=(pad.buttons&SCE_CTRL_TRIANGLE)!=0;
            p.handbrake=(pad.buttons&SCE_CTRL_SQUARE)!=0;
            if(pad.buttons&SCE_CTRL_UP) p.pitch=1;
            if(pad.buttons&SCE_CTRL_DOWN) p.pitch=-1;
            if(pressed&SCE_CTRL_SELECT)ballcam=!ballcam;
            if(pressed&SCE_CTRL_START){state=ST_MENU;break;}
            ai_input(&world.car[1],&world.ball,diff,&a);a.throttle*=AI_POWER[diff];
            int goal=world_step(&world,&p,&a,DT,1);ball_roll(&world.ball);
            if(goal){if(goal==1)score_p++;else score_a++;last_scorer=goal;state=ST_GOAL;timer=2.4f;}break;}
        case ST_GOAL:{Input zero={0};Input ai={0};ai_input(&world.car[1],&world.ball,diff,&ai);world_step(&world,&zero,&ai,DT,0);ball_roll(&world.ball);timer-=DT;
            if(timer<=0){if(score_p>=WIN_SCORE||score_a>=WIN_SCORE)state=ST_OVER;else{reset_round(&world);state=ST_COUNT;timer=3;}}break;}
        case ST_OVER:if(pressed&SCE_CTRL_TRIANGLE){score_p=score_a=0;reset_round(&world);state=ST_COUNT;timer=3;}else if(pressed&(SCE_CTRL_CROSS|SCE_CTRL_START))state=ST_MENU;break;
        }
        Car *pl=&world.car[0];Ball *ball=&world.ball;
        float px=pl->p.x*SCALE,pz=pl->p.y*SCALE,ex,ey,ez,cx,cy,cz;
        if(state==ST_MENU||state==ST_CONTROLS){float q=menu_t*.25f;ex=cosf(q)*72;ey=38;ez=sinf(q)*82;cx=0;cy=0;cz=0;}
        else{float want=car_yaw(pl);if(ballcam){float dx=ball->p.x-pl->p.x,dy=ball->p.y-pl->p.y;if(dx*dx+dy*dy>360000)want=atan2f(dy,dx);}if(state==ST_COUNT&&timer>2.9f)cam_yaw=car_yaw(pl);cam_yaw+=wrap_angle(want-cam_yaw)*.09f;
            ex=CLAMP(px-cosf(cam_yaw)*9,-HX-GD,HX+GD);ez=CLAMP(pz-sinf(cam_yaw)*9,-HY-5,HY+5);ey=4.8f+pl->p.z*SCALE*.25f;
            cx=px+cosf(cam_yaw)*13;cy=1.0f;cz=pz+sinf(cam_yaw)*13;}
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);begin3d();look_at(ex,ey,ez,cx,cy,cz);draw_arena();draw_car(&world.car[1],.95f,.48f,.1f,now);draw_car(pl,.15f,.4f,.95f,now);draw_ball(ball);
        begin2d();
        if(state==ST_MENU){rect2d(0,0,SCR_W,SCR_H,0,0,0,.35f);text_c(50,9,1,.75f,.2f,"VITA LEAGUE");text_c(125,4,.6f,.85f,1,"COCHES + FISICA 3D");const char *items[4]={"JUGAR","","CONTROLES","SALIR"};snprintf(buf,sizeof(buf),"DIFICULTAD: < %s >",DIFF_NAME[diff]);items[1]=buf;
            for(int i=0;i<4;i++){float y=215+i*62;if(i==sel){rect2d(230,y-12,500,56,.15f,.4f,.95f,.55f);text_c(y,4.5f,1,1,1,items[i]);}else text_c(y,4.5f,.8f,.85f,.95f,items[i]);}text_c(498,2.5f,.85f,.85f,.85f,"ARRIBA/ABAJO: ELEGIR   X: ACEPTAR");}
        else if(state==ST_CONTROLS){rect2d(0,0,SCR_W,SCR_H,0,0,0,.65f);text_c(40,6,1,.75f,.2f,"CONTROLES");const char *lines[6]={"STICK IZQ / PAD: GIRAR","R O X: ACELERAR","L: FRENAR / CUADRADO: DERRAPE","CIRCULO: BOOST","TRIANGULO: SALTO / DOBLE SALTO","SELECT: CAMARA PELOTA / COCHE"};for(int i=0;i<6;i++)text_c(130+i*52,3.6f,1,1,1,lines[i]);text_c(480,3,.7f,.85f,1,"START: VOLVER AL MENU");}
        else{rect2d(375,10,100,62,.1f,.3f,.9f,.85f);rect2d(485,10,100,62,.9f,.45f,.08f,.85f);snprintf(buf,sizeof(buf),"%d",score_p);text(425-text_w(6,buf)*.5f,22,6,1,1,1,buf);snprintf(buf,sizeof(buf),"%d",score_a);text(535-text_w(6,buf)*.5f,22,6,1,1,1,buf);text(30,20,3,.6f,.8f,1,"TU");text(SCR_W-30-text_w(3,"IA"),20,3,1,.75f,.4f,"IA");
            rect2d(330,494,300,26,0,0,0,.55f);rect2d(334,498,2.92f*pl->boost,18,1,.75f,.1f,1);text(335,470,2.4f,1,1,1,"BOOST");
            if(state==ST_COUNT){snprintf(buf,sizeof(buf),"%d",(int)ceilf(timer));text_c(190,14,1,.9f,.3f,buf);}else if(state==ST_GOAL)text_c(200,12,last_scorer==1?.5f:1,last_scorer==1?.75f:.65f,last_scorer==1?1:.25f,"GOOOL!");else if(state==ST_OVER){rect2d(0,0,SCR_W,SCR_H,0,0,0,.55f);text_c(150,10,1,1,1,score_p>score_a?"GANASTE!":"PERDISTE");snprintf(buf,sizeof(buf),"%d - %d",score_p,score_a);text_c(260,7,1,.8f,.3f,buf);text_c(380,3.4f,.85f,.9f,1,"TRIANGULO: REVANCHA");text_c(430,3.4f,.85f,.9f,1,"X: MENU");}}
        vglSwapBuffers(GL_FALSE);
    }
    sceKernelExitProcess(0);return 0;
}
