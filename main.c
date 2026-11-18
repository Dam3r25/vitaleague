/*
 * Vita League 3D v3 - estilo Rocket League para PS Vita (VitaSDK + vitaGL)
 * Fisica en Unreal Units (ver rlphys.h). Render: 1 uu = 0.01 unidades 3D.
 *
 *  Stick izq. X : girar (suelo) / guinada (aire)     Stick izq. Y : cabeceo (aire) / direccion del flip
 *  R : acelerar   L : freno / marcha atras           Cruz : saltar (2a vez = doble salto o flip)
 *  Circulo : boost   Cuadrado : derrape (suelo) / giro lateral "air roll" (aire)
 *  Triangulo : camara pelota / libre   Start : menu
 */
#include <vitaGL.h>
#include <psp2/ctrl.h>
#include <psp2/kernel/processmgr.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "rlphys.h"
#include "camera.h"

#define SCR_W 960
#define SCR_H 544
#define DT (1.0f / 60.0f)
#define SUB 2
#define SUBDT (DT / SUB)
#define S 0.01f
#define MATCH_TIME 300.0f

enum { ST_MENU, ST_CONTROLS, ST_COUNT, ST_PLAY, ST_GOAL, ST_OVER };
static const char *DIFF_NAME[3] = {"FACIL", "NORMAL", "DIFICIL"};

/* ======================= IA ======================= */
/*
 * Mejoras respecto a la version anterior:
 *  - Prediccion: simula la pelota con ball_step() (rebotes incluidos) y busca el primer punto
 *    alcanzable. La profundidad depende de la dificultad (12 / 30 / 45 pasos de 33 ms).
 *  - Defensa: si la pelota esta en su campo y la IA esta "por delante" (mas lejos de su porteria
 *    que la pelota), NO la empuja hacia su porteria: rodea y despeja hacia las bandas (X = +-3000).
 *  - Boost: con poco tanque busca la chapa activa que menos desvie su camino.
 *  - Flip frontal para acelerar (Normal y Dificil) y golpes aereos (Dificil).
 */
#define AI_MAXPRED 45
static const int AI_PRED_STEPS[3] = {12, 30, 45};
static int ai_pad_lock[2] = {-1, -1};
static int ai_plan[2] = {0, 0};          /* 0 nada, 1 flip de velocidad, 2 golpe aereo */

static float ai_low[2] = {0, 0};         /* tiempo casi parado */
static float ai_rev[2] = {0, 0};         /* tiempo restante de marcha atras de rescate */

static void ai_reset(void) { ai_pad_lock[0] = ai_pad_lock[1] = -1; ai_plan[0] = ai_plan[1] = 0; ai_low[0] = ai_low[1] = ai_rev[0] = ai_rev[1] = 0; }

static V3 flat3(V3 v) { return v3(v.x, v.y, 0); }
static float dist2d(V3 a, V3 b) { float dx = a.x - b.x, dy = a.y - b.y; return sqrtf(dx * dx + dy * dy); }

/* idx = 0 (ataca +Y) o 1 (ataca -Y) */
static void ai_drive(const World *w, int idx, int diff, Input *in) {
    const Car *c = &w->car[idx];
    const Ball *b = &w->ball;
    const float atk = (idx == 1) ? -1.0f : 1.0f;                 /* sentido de ataque en Y */
    memset(in, 0, sizeof(*in));
    V3 r = vcross(c->f, c->u);

    /* ---------- en el aire ---------- */
    if (!c->wheels && !c->touching) {
        if (ai_plan[idx] && c->jumped) {
            if (c->jump_hold > 0.0f) { in->jump = (c->jump_hold < 0.06f); return; }      /* toque corto del primer salto */
            if (c->can_flip) {                                                           /* segundo toque: flip */
                in->jump = 1;
                if (ai_plan[idx] == 1) in->pitch = 1.0f;                                 /* flip frontal */
                else {
                    V3 fh0 = flat3(c->f);
                    fh0 = vlen(fh0) > 0.2f ? vnorm(fh0) : v3(0, atk, 0);
                    if (vdot(vsub(flat3(b->p), flat3(c->p)), fh0) > 0.0f) in->pitch = 1.0f;
                }
                return;
            }
        }
        in->pitch = clampf(c->f.z * 3.0f, -1, 1);             /* nivelar para aterrizar sobre ruedas */
        in->handbrake = 1;
        in->steer = clampf(r.z * 3.0f, -1, 1);
        return;
    }
    if (c->wheels && !c->jumped) ai_plan[idx] = 0;

    V3 fh = flat3(c->f);
    fh = vlen(fh) > 0.2f ? vnorm(fh) : v3(0, atk, 0);
    V3 rh = vcross(fh, v3(0, 0, 1));
    V3 cf = flat3(c->p);
    float sp = vlen(c->v);

    /* ---------- prediccion de la pelota ---------- */
    /* Se simula la pelota con ball_step() (rebotes incluidos) y se elige el primer punto al que el coche
       llegue a tiempo. La estimacion es conservadora (velocidad realista, penalizacion por girar, margen
       del 15 %): una estimacion optimista hace que el coche llegue tarde y la pelota ya no este. */
    const float zmax = (diff == 2) ? 380.0f : 230.0f;
    const float spd_cons = clampf(sp + 300.0f, 800.0f, c->boost > 15.0f ? 1800.0f : 1350.0f);
    V3 icpt = b->p;
    int danger = 0, found = 0;                         /* peligro: la pelota entrara en mi porteria en el horizonte */
    const int chase_pred = dist2d(cf, b->p) > 1000.0f;   /* con la pelota cerca, persecucion directa */
    {
        Ball pred = *b;
        float best_gap = 1e9f;
        for (int k = 1; k <= AI_PRED_STEPS[diff]; k++) {
            ball_step(&pred, 0.033f);
            if (-atk * pred.p.y > ARENA_HY - 500.0f && fabsf(pred.p.x) < GOAL_HW + 400.0f) danger = 1;
            if (!chase_pred || found || pred.p.z >= zmax) continue;
            V3 dp = vsub(flat3(pred.p), cf);
            float t_need = vlen(dp) / spd_cons + fabsf(atan2f(vdot(dp, rh), vdot(dp, fh))) * 0.25f;
            float gap = t_need * 1.15f - k * 0.033f;  /* <= 0: llego antes que la pelota */
            if (gap <= 0.0f) { icpt = pred.p; found = 1; }
            else if (gap < best_gap) { best_gap = gap; icpt = pred.p; }
        }
    }
    V3 I = flat3(icpt);

    /* ---------- defensa y despeje ---------- */
    const float b_own = -atk * b->p.y;                /* > 0: pelota en el campo propio */
    const float c_own = -atk * c->p.y;
    float dist = dist2d(cf, I);
    V3 hdir = vnorm(vsub(I, cf));                     /* direccion en la que saldria la pelota si voy derecho */
    /* despeje: pelota en mi campo y yo por delante de ella; riesgo: un golpe directo la manda a mi porteria */
    const int clear_mode = (b_own > 1500.0f) && (c_own < b_own - 150.0f);
    const int risky = (-atk * hdir.y > 0.2f) && (b_own > 600.0f);
    V3 goal = v3(0, atk * ARENA_HY, 0);
    V3 a = vnorm(vsub(goal, I));                      /* direccion en la que quiero empujar la pelota */
    if (clear_mode || risky) {
        float side = (fabsf(I.x) > 1200.0f) ? (I.x > 0 ? 1.0f : -1.0f) : (c->p.x < I.x ? 1.0f : -1.0f);
        V3 T = v3(side * 3000.0f, I.y + atk * 800.0f, 0);
        a = vnorm(vsub(T, I));                        /* hacia la banda y hacia el campo rival */
    }

    /* ---------- punto de aproximacion ---------- */
    V3 tgt;
    if (clear_mode || risky) {
        float s = vdot(vsub(cf, I), a);               /* > 0: estoy delante de la pelota (empujaria hacia mi porteria) */
        if (s < -120.0f) {
            tgt = (dist < 450.0f) ? I : vsub(I, vmul(a, fminf(600.0f, dist * 0.6f)));
        } else {                                      /* rodear la pelota por mi lado, sin tocarla */
            V3 perp = v3(-a.y, a.x, 0);
            float sg = vdot(vsub(cf, I), perp) >= 0.0f ? 1.0f : -1.0f;
            tgt = vadd(vsub(I, vmul(a, 500.0f)), vmul(perp, sg * 650.0f));
        }
    } else {                                          /* ataque: ir a por la pelota, alineado si hay sitio */
        float s = vdot(vsub(cf, I), a);
        if (s < -60.0f || dist < 350.0f) tgt = vsub(I, vmul(a, dist < 350.0f ? 0.0f : 200.0f));
        else {
            V3 perp = v3(-a.y, a.x, 0);
            float sg = vdot(vsub(cf, I), perp) >= 0.0f ? 1.0f : -1.0f;
            tgt = vadd(vsub(I, vmul(a, 700.0f)), vmul(perp, sg * 600.0f));
        }
    }
    tgt.x = clampf(tgt.x, -(ARENA_HX - 150.0f), ARENA_HX - 150.0f);        /* solo para puntos de paso: nunca fuera de la arena */
    tgt.y = clampf(tgt.y, -(ARENA_HY - 150.0f), ARENA_HY - 150.0f);

    /* ---------- recoger boost ---------- */
    int going_pad = 0;
    if (diff > 0) {
        if (ai_pad_lock[idx] >= 0 && (w->pad_t[ai_pad_lock[idx]] > 0.0f || c->boost >= 65.0f)) ai_pad_lock[idx] = -1;
        float ball_to_own = dist2d(flat3(b->p), v3(0, -atk * ARENA_HY, 0));
        if (ai_pad_lock[idx] < 0 && c->boost < 35.0f && ball_to_own > 3500.0f && !danger && !clear_mode && dist > 900.0f) {
            float best = 1e9f; int bi = -1;
            for (int i = 0; i < NPADS; i++) {
                if (w->pad_t[i] > 0.0f) continue;
                V3 pp = v3(PAD_XY[i][0], PAD_XY[i][1], 0);
                float dp = dist2d(cf, pp);
                if (dp > 2800.0f || dp < 120.0f) continue;
                float detour = dp + dist2d(pp, tgt) - dist2d(cf, tgt);
                float lim = PAD_IS_BIG(i) ? 1500.0f : 700.0f;
                if (detour > lim) continue;
                float cost = detour - (PAD_IS_BIG(i) ? 600.0f : 0.0f);
                if (cost < best) { best = cost; bi = i; }
            }
            ai_pad_lock[idx] = bi;
        }
        if (ai_pad_lock[idx] >= 0 && !danger) {
            tgt = v3(PAD_XY[ai_pad_lock[idx]][0], PAD_XY[ai_pad_lock[idx]][1], 0);
            going_pad = 1;
        }
    }

    /* ---------- conduccion ---------- */
    V3 d = vsub(tgt, cf);
    float ang = atan2f(vdot(d, rh), vdot(d, fh));
    float dl = vlen(d);
    in->steer = clampf(ang * 2.5f, -1, 1);
    in->throttle = (diff == 0 ? 0.6f : 1.0f) * (fabsf(ang) < 1.7f ? 1.0f : 0.8f);   /* acelerar tambien al girar: a poca velocidad el coche no gira */
    /* rescate: si lleva un rato casi parado (pegado a una pared o a un poste) da marcha atras */
    ai_low[idx] = (sp < 120.0f) ? ai_low[idx] + 1.0f / 60.0f : 0.0f;
    const int in_goal = fabsf(c->p.y) > ARENA_HY - 150.0f && fabsf(c->p.x) < GOAL_HW + 100.0f;
    if (ai_low[idx] > 1.0f) { ai_rev[idx] = 0.9f; ai_low[idx] = 0.0f; }
    if (c->u.z < 0.4f && (ai_rev[idx] > 0.0f || in_goal)) {       /* pegado a una pared sin gravedad: salta para despegarse */
        ai_rev[idx] = 0.0f; in->jump = 1; in->steer = 0; return;
    }
    if (ai_rev[idx] > 0.0f || (in_goal && fabsf(ang) > 1.5f)) {   /* marcha atras para salir (o de la porteria) */
        ai_rev[idx] -= 1.0f / 60.0f;
        in->throttle = -1.0f; in->steer = -in->steer; in->boost = 0;
        return;
    }
    in->boost = (diff > 0 && !going_pad && fabsf(ang) < 0.2f && dl > 1100.0f && dist > 900.0f && c->boost > 40.0f);
    in->handbrake = (fabsf(ang) > 1.3f && sp > 700.0f);

    /* ---------- flip frontal para ganar velocidad (Normal y Dificil) ---------- */
    if (diff > 0 && c->wheels && !c->jumped && c->boost < 35.0f && fabsf(ang) < 0.12f && dl > 1800.0f &&
        dist > 2200.0f && sp > 700.0f && sp < 1950.0f) {
        float pd, nx, ny;
        V3 ahead = vadd(cf, vmul(fh, 1300.0f));
        plan_dist(ahead.x, ahead.y, &pd, &nx, &ny);
        if (pd > 500.0f && c->p.z < 60.0f) { ai_plan[idx] = 1; in->jump = 1; in->steer = 0; return; }
    }
    /* ---------- golpe aereo a pelota alta (Dificil) ---------- */
    if (diff == 2 && c->wheels && !c->jumped && b->p.z > 330.0f && dist2d(c->p, b->p) < 350.0f) {
        ai_plan[idx] = 2; in->jump = 1; return;
    }
}

static void ai_input(const World *w, int diff, Input *in) { ai_drive(w, 1, diff, in); }

/* ======================= malla de la arena ======================= */
#define ARC_N 6
#define ARC2 5
typedef struct { float x, y, nx, ny; } PP;
typedef struct { float a, b, na, nb; } PR;
static PP plan[64]; static int nplan;
static PR prof[40]; static int nprof;

static void build_mesh(void) {
    float cx = ARENA_HX - PLAN_R, cy = ARENA_HY - PLAN_R;
    const float cxs[4] = {cx, -cx, -cx, cx}, cys[4] = {cy, cy, -cy, -cy};
    nplan = 0;
    for (int k = 0; k < 4; k++) {
        for (int i = 0; i <= ARC_N; i++) {
            float th = (k * 90.0f + 90.0f * i / ARC_N) * PI_F / 180.0f;
            PP p = {cxs[k] + PLAN_R * cosf(th), cys[k] + PLAN_R * sinf(th), -cosf(th), -sinf(th)};
            plan[nplan++] = p;
        }
        if (k == 0) { PP a = {GOAL_HW, ARENA_HY, 0, -1}, b = {-GOAL_HW, ARENA_HY, 0, -1}; plan[nplan++] = a; plan[nplan++] = b; }
        if (k == 2) { PP a = {-GOAL_HW, -ARENA_HY, 0, 1}, b = {GOAL_HW, -ARENA_HY, 0, 1}; plan[nplan++] = a; plan[nplan++] = b; }
    }
    nprof = 0;
    for (int i = 0; i <= ARC2; i++) {
        float ph = (-90.0f - 90.0f * i / ARC2) * PI_F / 180.0f;
        PR p = {RAMP_R + RAMP_R * cosf(ph), RAMP_R + RAMP_R * sinf(ph), -cosf(ph), -sinf(ph)};
        prof[nprof++] = p;
    }
    { PR p = {0, GOAL_H, 1, 0}; prof[nprof++] = p; }
    for (int i = 0; i <= ARC2; i++) {
        float ph = (180.0f - 90.0f * i / ARC2) * PI_F / 180.0f;
        PR p = {RAMP_R + RAMP_R * cosf(ph), ARENA_H - RAMP_R + RAMP_R * sinf(ph), -cosf(ph), -sinf(ph)};
        prof[nprof++] = p;
    }
}

/* ======================= dibujo 3D ======================= */
/* 1 uu = 0.01 unidades 3D; mundo (x, y, z) -> OpenGL (x, z, -y) */
#define GV(x, y, z) glVertex3f((x) * S, (z) * S, -(y) * S)

/*
 * OPTIMIZACION DE RENDER (vitaGL)
 *  - En vitaGL cada glEnd() es un envio de dibujado completo (recarga de estado + sceGxmDraw), y eso
 *    pesa mucho mas que cada glVertex. Antes se hacian ~145 pares glBegin/glEnd por frame; ahora ~7.
 *  - Toda la geometria ESTATICA de la arena se calcula UNA vez al iniciar (build_geometry) en un
 *    buffer; cada frame solo se emite ese buffer, en un unico bloque, sin trigonometria ni bucles
 *    de calculo. La pelota usa una tabla de vertices precalculada.
 *  - Chapas de boost, sombras y las cajas de cada coche van agrupadas en un bloque cada una.
 *
 *  Display lists (USE_DLISTS=1): vitaGL las implementa grabando llamadas, pero SOLO si se compilo con
 *  HAVE_DLISTS. La libreria que instala vdpm NO lo trae: glBegin/glVertex no se graban y la lista queda
 *  vacia (la arena desapareceria), y ademas solo hay 1 hueco. Por eso esta desactivado por defecto.
 *  Activalo solo con una vitaGL compilada con "make HAVE_DLISTS=1 install".
 */
#ifndef USE_DLISTS
#define USE_DLISTS 0
#endif

#define NSEG16 16
#define NSEG14 14
static float c16[NSEG16 + 1], s16[NSEG16 + 1], c14[NSEG14 + 1], s14[NSEG14 + 1];

static void build_tables(void) {
    for (int i = 0; i <= NSEG16; i++) { float a = i * 2.0f * PI_F / NSEG16; c16[i] = cosf(a); s16[i] = sinf(a); }
    for (int i = 0; i <= NSEG14; i++) { float a = i * 2.0f * PI_F / NSEG14; c14[i] = cosf(a); s14[i] = sinf(a); }
}

static void begin3d(void) {
    const float nr = 0.5f, fr = 800.0f;
    const float t = 0.390647f;                         /* nr * tan(38 grados) */
    float r = t * (float)SCR_W / (float)SCR_H;
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-r, r, -t, t, nr, fr);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

static void look_at(V3 e, V3 c) {   /* e, c en coordenadas del mundo (uu) */
    float ex = e.x * S, ey = e.z * S, ez = -e.y * S, cx = c.x * S, cy = c.z * S, cz = -c.y * S;
    float fx = cx - ex, fy = cy - ey, fz = cz - ez, fl = sqrtf(fx * fx + fy * fy + fz * fz);
    fx /= fl; fy /= fl; fz /= fl;
    float sx = -fz, sy = 0.0f, sz = fx, sl = sqrtf(sx * sx + sz * sz);
    if (sl < 1e-4f) { sx = 1.0f; sz = 0.0f; sl = 1.0f; }   /* vista vertical: base estable */
    sx /= sl; sz /= sl;
    float ux = sy * fz - sz * fy, uy = sz * fx - sx * fz, uz = sx * fy - sy * fx;
    float m[16] = {sx, ux, -fx, 0, sy, uy, -fy, 0, sz, uz, -fz, 0, 0, 0, 0, 1};
    glMultMatrixf(m);
    glTranslatef(-ex, -ey, -ez);
}

/* caja con 5 caras (sin la de abajo) EMITIDA dentro de un glBegin(GL_QUADS) ya abierto */
static void ebox(float cx, float cy, float cz, float hx, float hy, float hz, float r, float g, float b) {
    float x0 = cx - hx, x1 = cx + hx, y0 = cy - hy, y1 = cy + hy, z0 = cz - hz, z1 = cz + hz;
    glColor3f(r, g, b);
    glVertex3f(x0, y1, z0); glVertex3f(x0, y1, z1); glVertex3f(x1, y1, z1); glVertex3f(x1, y1, z0);
    glColor3f(r * 0.8f, g * 0.8f, b * 0.8f);
    glVertex3f(x1, y0, z0); glVertex3f(x1, y1, z0); glVertex3f(x1, y1, z1); glVertex3f(x1, y0, z1);
    glVertex3f(x0, y0, z1); glVertex3f(x0, y1, z1); glVertex3f(x0, y1, z0); glVertex3f(x0, y0, z0);
    glColor3f(r * 0.62f, g * 0.62f, b * 0.62f);
    glVertex3f(x0, y0, z1); glVertex3f(x1, y0, z1); glVertex3f(x1, y1, z1); glVertex3f(x0, y1, z1);
    glVertex3f(x1, y0, z0); glVertex3f(x0, y0, z0); glVertex3f(x0, y1, z0); glVertex3f(x1, y1, z0);
}

/* ---------- geometria estatica de la arena: se construye una vez ---------- */
typedef struct { float x, y, z, r, g, b; } GVtx;
#define GEO_MAX 6000
static GVtx geo[GEO_MAX];
static int geo_n = 0;

static void gv(float x, float y, float z, float r, float g, float b) {      /* coordenadas OpenGL */
    if (geo_n >= GEO_MAX) return;
    GVtx *v = &geo[geo_n++];
    v->x = x; v->y = y; v->z = z; v->r = r; v->g = g; v->b = b;
}
static void gq(V3 a, V3 b, V3 c, V3 d, float r, float g, float bl) {        /* quad en coordenadas del mundo */
    gv(a.x * S, a.z * S, -a.y * S, r, g, bl); gv(b.x * S, b.z * S, -b.y * S, r, g, bl);
    gv(c.x * S, c.z * S, -c.y * S, r, g, bl); gv(d.x * S, d.z * S, -d.y * S, r, g, bl);
}
static void gbox(float cx, float cy, float cz, float hx, float hy, float hz, float r, float g, float b) {
    float x0 = cx - hx, x1 = cx + hx, y0 = cy - hy, y1 = cy + hy, z0 = cz - hz, z1 = cz + hz;
    float r8 = r * 0.8f, g8 = g * 0.8f, b8 = b * 0.8f, r6 = r * 0.62f, g6 = g * 0.62f, b6 = b * 0.62f;
    gv(x0, y1, z0, r, g, b); gv(x0, y1, z1, r, g, b); gv(x1, y1, z1, r, g, b); gv(x1, y1, z0, r, g, b);
    gv(x1, y0, z0, r8, g8, b8); gv(x1, y1, z0, r8, g8, b8); gv(x1, y1, z1, r8, g8, b8); gv(x1, y0, z1, r8, g8, b8);
    gv(x0, y0, z1, r8, g8, b8); gv(x0, y1, z1, r8, g8, b8); gv(x0, y1, z0, r8, g8, b8); gv(x0, y0, z0, r8, g8, b8);
    gv(x0, y0, z1, r6, g6, b6); gv(x1, y0, z1, r6, g6, b6); gv(x1, y1, z1, r6, g6, b6); gv(x0, y1, z1, r6, g6, b6);
    gv(x1, y0, z0, r6, g6, b6); gv(x0, y0, z0, r6, g6, b6); gv(x0, y1, z0, r6, g6, b6); gv(x1, y1, z0, r6, g6, b6);
}

static void wall_color(float y, float shade, float *r, float *g, float *b) {
    float t = clampf(0.5f + y / 1400.0f, 0.0f, 1.0f);
    *r = lerpf(0.20f, 0.92f, t) * shade; *g = lerpf(0.40f, 0.52f, t) * shade; *b = lerpf(0.88f, 0.14f, t) * shade;
}

static void build_geometry(void) {
    geo_n = 0;
    /* suelo exterior */
    gv(-900, -0.5f, -900, 0.05f, 0.06f, 0.09f); gv(-900, -0.5f, 900, 0.05f, 0.06f, 0.09f);
    gv(900, -0.5f, 900, 0.05f, 0.06f, 0.09f); gv(900, -0.5f, -900, 0.05f, 0.06f, 0.09f);
    /* cesped: el abanico de triangulos se guarda como quads degenerados (centro, Pi, Pi+1, Pi+1) */
    for (int i = 0; i < nplan; i++) {
        const PP *p0 = &plan[i], *p1 = &plan[(i + 1) % nplan];
        float x0 = (p0->x + p0->nx * RAMP_R) * S, z0 = -(p0->y + p0->ny * RAMP_R) * S;
        float x1 = (p1->x + p1->nx * RAMP_R) * S, z1 = -(p1->y + p1->ny * RAMP_R) * S;
        gv(0, 0, 0, 0.10f, 0.36f, 0.18f); gv(x0, 0, z0, 0.10f, 0.36f, 0.18f);
        gv(x1, 0, z1, 0.10f, 0.36f, 0.18f); gv(x1, 0, z1, 0.10f, 0.36f, 0.18f);
    }
    {   /* franjas */
        float x0 = -(ARENA_HX - RAMP_R), x1 = ARENA_HX - RAMP_R, y0 = -(ARENA_HY - PLAN_R), band = 2.0f * (ARENA_HY - PLAN_R) / 13.0f;
        for (int k = 1; k < 13; k += 2)
            gq(v3(x0, y0 + k * band, 2), v3(x1, y0 + k * band, 2), v3(x1, y0 + (k + 1) * band, 2), v3(x0, y0 + (k + 1) * band, 2), 0.13f, 0.42f, 0.21f);
    }
    /* marcas: linea central y circulo central */
    gq(v3(-(ARENA_HX - RAMP_R), -12, 4), v3(ARENA_HX - RAMP_R, -12, 4), v3(ARENA_HX - RAMP_R, 12, 4), v3(-(ARENA_HX - RAMP_R), 12, 4), 0.85f, 0.9f, 0.85f);
    for (int i = 0; i < 40; i++) {
        float a0 = i * 2 * PI_F / 40, a1 = (i + 1) * 2 * PI_F / 40;
        gq(v3(cosf(a0) * 790, sinf(a0) * 790, 4), v3(cosf(a0) * 830, sinf(a0) * 830, 4),
           v3(cosf(a1) * 830, sinf(a1) * 830, 4), v3(cosf(a1) * 790, sinf(a1) * 790, 4), 0.85f, 0.9f, 0.85f);
    }
    /* paredes, rampas y esquinas */
    for (int i = 0; i < nplan; i++) {
        const PP *p0 = &plan[i], *p1 = &plan[(i + 1) % nplan];
        int back = fabsf(p0->y) > ARENA_HY - 1.0f && fabsf(p1->y) > ARENA_HY - 1.0f &&
                   fabsf(p0->x) < GOAL_HW + 1.0f && fabsf(p1->x) < GOAL_HW + 1.0f;
        float par = (i & 1) ? 0.93f : 1.0f;
        for (int j = 0; j + 1 < nprof; j++) {
            const PR *q0 = &prof[j], *q1 = &prof[j + 1];
            if (back && fmaxf(q0->b, q1->b) <= GOAL_H + 1.0f) continue;
            const PP *pp[4] = {p0, p1, p1, p0};
            const PR *qq[4] = {q0, q0, q1, q1};
            for (int k = 0; k < 4; k++) {
                float nx = pp[k]->nx * qq[k]->na, ny = pp[k]->ny * qq[k]->na, nz = qq[k]->nb;
                float lit = fmaxf(0.0f, nx * 0.3f + ny * 0.2f + nz * 0.9f);
                float cr, cg, cb;
                wall_color(pp[k]->y, (0.55f + 0.45f * lit) * par, &cr, &cg, &cb);
                float wx = pp[k]->x + pp[k]->nx * qq[k]->a, wy = pp[k]->y + pp[k]->ny * qq[k]->a;
                gv(wx * S, qq[k]->b * S, -wy * S, cr, cg, cb);
            }
        }
    }
    /* porterias */
    for (int s = -1; s <= 1; s += 2) {
        float y0 = s * ARENA_HY, y1 = s * (ARENA_HY + GOAL_D);
        float cr = s > 0 ? 0.95f : 0.2f, cg = s > 0 ? 0.55f : 0.42f, cb = s > 0 ? 0.12f : 0.95f;
        gq(v3(-GOAL_HW, y0, 3), v3(GOAL_HW, y0, 3), v3(GOAL_HW, y1, 3), v3(-GOAL_HW, y1, 3), cr * 0.5f, cg * 0.5f, cb * 0.5f);
        gq(v3(-GOAL_HW, y1, 0), v3(GOAL_HW, y1, 0), v3(GOAL_HW, y1, GOAL_H), v3(-GOAL_HW, y1, GOAL_H), cr * 0.7f, cg * 0.7f, cb * 0.7f);
        gq(v3(-GOAL_HW, y0, GOAL_H), v3(GOAL_HW, y0, GOAL_H), v3(GOAL_HW, y1, GOAL_H), v3(-GOAL_HW, y1, GOAL_H), cr * 0.45f, cg * 0.45f, cb * 0.45f);
        gq(v3(GOAL_HW, y0, 0), v3(GOAL_HW, y1, 0), v3(GOAL_HW, y1, GOAL_H), v3(GOAL_HW, y0, GOAL_H), cr * 0.6f, cg * 0.6f, cb * 0.6f);
        gq(v3(-GOAL_HW, y0, 0), v3(-GOAL_HW, y1, 0), v3(-GOAL_HW, y1, GOAL_H), v3(-GOAL_HW, y0, GOAL_H), cr * 0.6f, cg * 0.6f, cb * 0.6f);
        gbox(-GOAL_HW * S, GOAL_H * S * 0.5f, -y0 * S, 0.18f, GOAL_H * S * 0.5f, 0.18f, cr, cg, cb);
        gbox(GOAL_HW * S, GOAL_H * S * 0.5f, -y0 * S, 0.18f, GOAL_H * S * 0.5f, 0.18f, cr, cg, cb);
        gbox(0, GOAL_H * S, -y0 * S, GOAL_HW * S, 0.18f, 0.18f, cr, cg, cb);
    }
}

/* emite todo el buffer en UN solo bloque (el color solo se cambia cuando cambia) */
static void emit_geometry(void) {
    float lr = -1, lg = -1, lb = -1;
    glBegin(GL_QUADS);
    for (int i = 0; i < geo_n; i++) {
        const GVtx *v = &geo[i];
        if (v->r != lr || v->g != lg || v->b != lb) { glColor3f(v->r, v->g, v->b); lr = v->r; lg = v->g; lb = v->b; }
        glVertex3f(v->x, v->y, v->z);
    }
    glEnd();
}

/* ---------- pelota: tabla de vertices precalculada ---------- */
#define BALL_LAT 8
#define BALL_LON 14
static float ball_v[BALL_LAT * BALL_LON * 4][3];

static void build_ball_mesh(void) {
    float R = BALL_R * S;
    int n = 0;
    for (int i = 0; i < BALL_LAT; i++) {
        float a0 = -PI_F / 2 + PI_F * i / BALL_LAT, a1 = -PI_F / 2 + PI_F * (i + 1) / BALL_LAT;
        for (int j = 0; j < BALL_LON; j++) {
            float b0 = 2 * PI_F * j / BALL_LON, b1 = 2 * PI_F * (j + 1) / BALL_LON;
            float v[4][3] = {{R * cosf(a0) * cosf(b0), R * sinf(a0), R * cosf(a0) * sinf(b0)},
                             {R * cosf(a1) * cosf(b0), R * sinf(a1), R * cosf(a1) * sinf(b0)},
                             {R * cosf(a1) * cosf(b1), R * sinf(a1), R * cosf(a1) * sinf(b1)},
                             {R * cosf(a0) * cosf(b1), R * sinf(a0), R * cosf(a0) * sinf(b1)}};
            memcpy(ball_v[n], v, sizeof(v)); n += 4;
        }
    }
}

static void emit_ball(void) {
    glBegin(GL_QUADS);
    for (int q = 0; q < BALL_LAT * BALL_LON; q++) {
        if (((q / BALL_LON) + (q % BALL_LON)) & 1) glColor3f(0.95f, 0.95f, 0.95f); else glColor3f(0.25f, 0.28f, 0.34f);
        for (int k = 0; k < 4; k++) glVertex3f(ball_v[q * 4 + k][0], ball_v[q * 4 + k][1], ball_v[q * 4 + k][2]);
    }
    glEnd();
}

#if USE_DLISTS
static GLuint dl_arena = 0, dl_ball = 0;
static void build_display_lists(void) {
    GLuint l = glGenLists(1);
    if (l) { glNewList(l, GL_COMPILE); emit_geometry(); glEndList(); dl_arena = l; }
    l = glGenLists(1);                                 /* la libreria estandar solo tiene 1 hueco: la pelota usa la tabla */
    if (l) { glNewList(l, GL_COMPILE); emit_ball(); glEndList(); dl_ball = l; }
}
#endif

static void draw_arena(void) {
#if USE_DLISTS
    if (dl_arena) { glCallList(dl_arena); return; }
#endif
    emit_geometry();
}

/* ---------- chapas de boost: todas en un solo bloque ---------- */
static void emit_disc(float x, float y, float z, float rad, float r, float g, float b) {   /* 16-gono como 8 "cometas" */
    glColor3f(r, g, b);
    for (int i = 0; i < NSEG16; i += 2) {
        GV(x, y, z);
        GV(x + c16[i] * rad, y + s16[i] * rad, z);
        GV(x + c16[i + 1] * rad, y + s16[i + 1] * rad, z);
        GV(x + c16[i + 2] * rad, y + s16[i + 2] * rad, z);
    }
}

static void draw_pads(const World *w, float t) {
    float bob = 0.55f + 0.1f * sinf(t * 3);
    glBegin(GL_QUADS);
    for (int i = 0; i < NPADS; i++) {
        int big = PAD_IS_BIG(i), act = w->pad_t[i] <= 0.0f;
        float x = PAD_XY[i][0], y = PAD_XY[i][1];
        if (big) {
            emit_disc(x, y, 3, 190, act ? 1.0f : 0.3f, act ? 0.75f : 0.25f, act ? 0.1f : 0.2f);
            if (act) ebox(x * S, bob, -y * S, 0.35f, 0.35f, 0.35f, 1.0f, 0.85f, 0.25f);
        } else {
            emit_disc(x, y, 3, 110, act ? 1.0f : 0.3f, act ? 0.6f : 0.25f, act ? 0.1f : 0.2f);
        }
    }
    glEnd();
}

/* ---------- sombras: las tres en un solo bloque con mezcla ---------- */
static float shadow_k(V3 p, float rad) {                /* 0 = no se dibuja */
    float pd, nx, ny;
    plan_dist(p.x, p.y, &pd, &nx, &ny);
    if (p.z > 900.0f || pd < RAMP_R + rad) return 0.0f;
    return clampf(1.0f - p.z / 900.0f, 0.2f, 1.0f);
}
static void emit_shadow(V3 p, float rad, float k) {
    glColor4f(0, 0, 0, 0.35f * k);
    for (int i = 0; i < NSEG14; i += 2) {
        GV(p.x, p.y, 5);
        GV(p.x + c14[i] * rad * k, p.y + s14[i] * rad * k, 5);
        GV(p.x + c14[i + 1] * rad * k, p.y + s14[i + 1] * rad * k, 5);
        GV(p.x + c14[i + 2] * rad * k, p.y + s14[i + 2] * rad * k, 5);
    }
}
static void draw_shadows(const World *w) {
    V3 pos[3] = {w->car[1].p, w->car[0].p, w->ball.p};
    float rad[3] = {95.0f, 95.0f, BALL_R * 1.1f}, k[3];
    int any = 0;
    for (int i = 0; i < 3; i++) { k[i] = shadow_k(pos[i], rad[i]); any |= k[i] > 0.0f; }
    if (!any) return;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_QUADS);
    for (int i = 0; i < 3; i++) if (k[i] > 0.0f) emit_shadow(pos[i], rad[i], k[i]);
    glEnd();
    glDisable(GL_BLEND);
}

/* ---------- coche: todas sus cajas en un solo bloque ---------- */
static void draw_car(const Car *c, float r, float g, float b, float t) {
    V3 rt = vcross(c->f, c->u);
    float m[16] = {c->f.x, c->f.z, -c->f.y, 0, c->u.x, c->u.z, -c->u.y, 0, rt.x, rt.z, -rt.y, 0,
                   c->p.x * S, c->p.z * S, -c->p.y * S, 1};
    glPushMatrix();
    glMultMatrixf(m);
    glBegin(GL_QUADS);
    ebox(0.0f, -0.04f, 0.0f, 0.59f, 0.12f, 0.40f, r, g, b);                    /* chasis */
    ebox(-0.08f, 0.17f, 0.0f, 0.27f, 0.11f, 0.33f, r * 0.6f, g * 0.6f, b * 0.6f); /* cabina */
    ebox(0.22f, 0.17f, 0.0f, 0.04f, 0.09f, 0.30f, 0.7f, 0.85f, 1.0f);          /* parabrisas */
    ebox(0.52f, 0.0f, 0.0f, 0.08f, 0.08f, 0.38f, 1.0f, 1.0f, 1.0f);            /* morro */
    ebox(-0.56f, 0.21f, 0.0f, 0.04f, 0.04f, 0.40f, r * 0.4f, g * 0.4f, b * 0.4f); /* aleron */
    for (int i = 0; i < 4; i++)
        ebox((i & 1) ? 0.36f : -0.36f, -0.12f, (i & 2) ? 0.43f : -0.43f, 0.15f, 0.15f, 0.07f, 0.06f, 0.06f, 0.06f);
    if (c->boosting) {
        float f = 0.8f + 0.4f * sinf(t * 40.0f);
        ebox(-0.6f - 0.45f * f, -0.04f, 0.0f, 0.45f * f, 0.07f, 0.1f, 1.0f, 0.5f, 0.1f);
        ebox(-0.6f - 0.3f * f, -0.04f, 0.0f, 0.3f * f, 0.04f, 0.05f, 1.0f, 0.95f, 0.5f);
    }
    glEnd();
    glPopMatrix();
}

static float ball_rot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static void ball_spin(float ax, float ay, float az, float ang) {   /* eje unitario en coords GL */
    float c = cosf(ang), s = sinf(ang), t = 1.0f - c;
    float d[9] = {c + ax * ax * t, ax * ay * t - az * s, ax * az * t + ay * s,
                  ay * ax * t + az * s, c + ay * ay * t, ay * az * t - ax * s,
                  az * ax * t - ay * s, az * ay * t + ax * s, c + az * az * t};
    float r[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r[i * 3 + j] = d[i * 3] * ball_rot[j] + d[i * 3 + 1] * ball_rot[3 + j] + d[i * 3 + 2] * ball_rot[6 + j];
    memcpy(ball_rot, r, sizeof(r));
}

static void draw_ball(const Ball *b) {
    glPushMatrix();
    glTranslatef(b->p.x * S, b->p.z * S, -b->p.y * S);
    float m[16] = {ball_rot[0], ball_rot[3], ball_rot[6], 0, ball_rot[1], ball_rot[4], ball_rot[7], 0,
                   ball_rot[2], ball_rot[5], ball_rot[8], 0, 0, 0, 0, 1};
    glMultMatrixf(m);
#if USE_DLISTS
    if (dl_ball) glCallList(dl_ball); else
#endif
    emit_ball();
    glPopMatrix();
}

/* ======================= dibujo 2D (HUD y menu) ======================= */
static const char FONT_CHARS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:-!./<> ";
static const unsigned char FONT[][7] = {
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}, {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}, {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}, {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}, {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}, {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
    {0x07,0x02,0x02,0x02,0x02,0x12,0x0C}, {0x11,0x12,0x14,0x18,0x14,0x12,0x11}, {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}, {0x11,0x19,0x15,0x13,0x11,0x11,0x11}, {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}, {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}, {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}, {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}, {0x11,0x11,0x11,0x15,0x15,0x15,0x0A}, {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
    {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}, {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
    {0x00,0x04,0x04,0x00,0x04,0x04,0x00}, {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, {0x04,0x04,0x04,0x04,0x04,0x00,0x04},
    {0x00,0x00,0x00,0x00,0x00,0x06,0x06}, {0x01,0x02,0x02,0x04,0x08,0x08,0x10}, {0x02,0x04,0x08,0x10,0x08,0x04,0x02},
    {0x08,0x04,0x02,0x01,0x02,0x04,0x08}, {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
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

/*
 * Interfaz 2D en bloques GL_QUADS. Cada glEnd es un envio de dibujado, asi que:
 *  - text() dibuja la sombra Y el texto en el MISMO glBegin/glEnd (antes eran dos).
 *  - Los pixeles contiguos de una fila de cada letra se funden en un unico rectangulo.
 *  - main() abre un lote para toda la interfaz de un frame (ui_begin/ui_end): un solo glBegin.
 *    vitaGL solo tiene indices para 8192 quads por glBegin, asi que ui_quad() cierra y reabre el
 *    bloque antes de llegar a ese limite.
 */
#define UI_FLUSH 6000
static int ui_open = 0, ui_quads = 0;
static float ui_col[4] = {1, 1, 1, 1};

static int ui_begin(void) {                  /* devuelve 1 si abre el bloque (y por tanto debe cerrarlo) */
    if (ui_open) return 0;
    glBegin(GL_QUADS); ui_open = 1; ui_quads = 0;
    return 1;
}
static void ui_end(int own) { if (own) { glEnd(); ui_open = 0; } }
static void ui_color(float r, float g, float b, float a) {
    ui_col[0] = r; ui_col[1] = g; ui_col[2] = b; ui_col[3] = a;
    glColor4f(r, g, b, a);
}
static void ui_quad(float x, float y, float w, float h) {
    glVertex2f(x, y); glVertex2f(x + w, y); glVertex2f(x + w, y + h); glVertex2f(x, y + h);
    if (++ui_quads >= UI_FLUSH) {            /* salvaguarda del limite de indices de vitaGL */
        glEnd(); glBegin(GL_QUADS);
        glColor4f(ui_col[0], ui_col[1], ui_col[2], ui_col[3]);
        ui_quads = 0;
    }
}

static void rect2d(float x, float y, float w, float h, float r, float g, float b, float a) {
    int own = ui_begin();
    ui_color(r, g, b, a);
    ui_quad(x, y, w, h);
    ui_end(own);
}
static float text_w(float s, const char *t) { return (float)strlen(t) * 6.0f * s - s; }

/* emite los rectangulos de una cadena en el color indicado (el bloque ya debe estar abierto) */
static void text_raw(float x, float y, float s, float r, float g, float b, const char *t) {
    ui_color(r, g, b, 1.0f);
    for (; *t; t++, x += 6.0f * s) {
        const char *p = strchr(FONT_CHARS, toupper((unsigned char)*t));
        if (!p) continue;
        const unsigned char *gl = FONT[p - FONT_CHARS];
        for (int row = 0; row < 7; row++) {
            unsigned bits = gl[row];
            int col = 0;
            while (col < 5) {
                if (bits & (1u << (4 - col))) {
                    int e = col;
                    while (e < 5 && (bits & (1u << (4 - e)))) e++;
                    ui_quad(x + col * s, y + row * s, (e - col) * s, s);
                    col = e;
                } else col++;
            }
        }
    }
}
static void text(float x, float y, float s, float r, float g, float b, const char *t) {
    int own = ui_begin();                                  /* sombra + texto en el mismo bloque */
    text_raw(x + s * 0.6f, y + s * 0.6f, s, 0, 0, 0, t);
    text_raw(x, y, s, r, g, b, t);
    ui_end(own);
}
static void text_c(float y, float s, float r, float g, float b, const char *t) {
    text((SCR_W - text_w(s, t)) * 0.5f, y, s, r, g, b, t);
}

/* ======================= programa principal ======================= */
static void kickoff(World *w) { world_reset(w); ai_reset(); }

int main(void) {
    vglInit(0x800000);
    glClearColor(0.04f, 0.05f, 0.09f, 1.0f);
    glViewport(0, 0, SCR_W, SCR_H);
    sceCtrlSetSamplingMode(SCE_CTRL_MODE_ANALOG);
    build_mesh();
    build_tables();
    build_geometry();
    build_ball_mesh();
#if USE_DLISTS
    build_display_lists();
#endif

    static World w;
    world_reset(&w); pads_reset(&w);
    int state = ST_MENU, sel = 0, diff = 1, quit = 0, ballcam = 1;
    int score_p = 0, score_a = 0, last_scorer = 0, overtime = 0;
    float timer = 0.0f, clock = MATCH_TIME, menu_t = 0.0f, now = 0.0f;
    Cam cam; cam_init(&cam);
    int cam_snap = 1;
    unsigned int prev = 0, prev_nav = 0;
    char buf[64];

    while (!quit) {
        SceCtrlData pad;
        sceCtrlPeekBufferPositive(0, &pad, 1);
        unsigned int pressed = pad.buttons & ~prev;
        prev = pad.buttons;
        unsigned int nav = pad.buttons;
        if (pad.ly < 60) nav |= SCE_CTRL_UP;
        if (pad.ly > 196) nav |= SCE_CTRL_DOWN;
        if (pad.lx < 60) nav |= SCE_CTRL_LEFT;
        if (pad.lx > 196) nav |= SCE_CTRL_RIGHT;
        unsigned int npress = nav & ~prev_nav;
        prev_nav = nav;
        now += DT;

        Input pin, ain, zero;
        memset(&pin, 0, sizeof pin); memset(&ain, 0, sizeof ain); memset(&zero, 0, sizeof zero);

        switch (state) {
        case ST_MENU:
            menu_t += DT;
            if (npress & SCE_CTRL_UP) sel = (sel + 3) % 4;
            if (npress & SCE_CTRL_DOWN) sel = (sel + 1) % 4;
            if (sel == 1 && (npress & SCE_CTRL_LEFT)) diff = (diff + 2) % 3;
            if (sel == 1 && (npress & SCE_CTRL_RIGHT)) diff = (diff + 1) % 3;
            if (pressed & SCE_CTRL_CROSS) {
                if (sel == 0) {
                    score_p = score_a = 0; overtime = 0; clock = MATCH_TIME;
                    pads_reset(&w); kickoff(&w); state = ST_COUNT; timer = 3.0f; cam_snap = 1;
                } else if (sel == 1) diff = (diff + 1) % 3;
                else if (sel == 2) state = ST_CONTROLS;
                else quit = 1;
            }
            break;
        case ST_CONTROLS:
            menu_t += DT;
            if (pressed & (SCE_CTRL_CIRCLE | SCE_CTRL_CROSS | SCE_CTRL_START)) state = ST_MENU;
            break;
        case ST_COUNT:
            timer -= DT;
            if (timer <= 0.0f) state = ST_PLAY;
            if (pressed & SCE_CTRL_START) state = ST_MENU;
            break;
        case ST_PLAY: {
            float sx = ((int)pad.lx - 128) / 128.0f, sy = (128 - (int)pad.ly) / 128.0f;
            if (fabsf(sx) < 0.15f) sx = 0.0f;
            if (fabsf(sy) < 0.15f) sy = 0.0f;
            if (pad.buttons & SCE_CTRL_LEFT) sx = -1.0f;
            if (pad.buttons & SCE_CTRL_RIGHT) sx = 1.0f;
            if (pad.buttons & SCE_CTRL_UP) sy = 1.0f;
            if (pad.buttons & SCE_CTRL_DOWN) sy = -1.0f;
            pin.steer = sx; pin.pitch = sy;
            pin.throttle = ((pad.buttons & SCE_CTRL_RTRIGGER) ? 1.0f : 0.0f) - ((pad.buttons & SCE_CTRL_LTRIGGER) ? 1.0f : 0.0f);
            pin.boost = (pad.buttons & SCE_CTRL_CIRCLE) != 0;
            pin.jump = (pad.buttons & SCE_CTRL_CROSS) != 0;
            pin.handbrake = (pad.buttons & SCE_CTRL_SQUARE) != 0;
            if (pressed & SCE_CTRL_TRIANGLE) ballcam = !ballcam;
            if (pressed & SCE_CTRL_START) { state = ST_MENU; break; }

            if (!overtime) { clock -= DT; if (clock <= 0.0f) { clock = 0.0f; if (score_p != score_a) { state = ST_OVER; break; } overtime = 1; } }
            else clock += DT;

            ai_input(&w, diff, &ain);
            int g = 0;
            for (int i = 0; i < SUB && !g; i++) g = world_step(&w, &pin, &ain, SUBDT, 1);
            if (g) {
                if (g == 1) score_p++; else score_a++;
                last_scorer = g; state = ST_GOAL; timer = 3.0f;
            }
            break;
        }
        case ST_GOAL:
            for (int i = 0; i < SUB; i++) world_step(&w, &zero, &zero, SUBDT, 0);
            timer -= DT;
            if (timer <= 0.0f) {
                if (overtime || (clock <= 0.0f && score_p != score_a)) state = ST_OVER;
                else { kickoff(&w); state = ST_COUNT; timer = 3.0f; cam_snap = 1; }
            }
            break;
        case ST_OVER:
            if (pressed & SCE_CTRL_TRIANGLE) {
                score_p = score_a = 0; overtime = 0; clock = MATCH_TIME;
                pads_reset(&w); kickoff(&w); state = ST_COUNT; timer = 3.0f; cam_snap = 1;
            } else if (pressed & (SCE_CTRL_CROSS | SCE_CTRL_START)) state = ST_MENU;
            break;
        }

        /* ---------- giro visual de la pelota ---------- */
        {
            float d; V3 n;
            arena(w.ball.p, &d, &n);
            if (d < BALL_R + 20.0f && state != ST_MENU && state != ST_CONTROLS) {
                V3 om = vmul(vcross(n, w.ball.v), 1.0f / BALL_R);
                float wl = vlen(om);
                if (wl > 1e-3f) ball_spin(om.x / wl, om.z / wl, -om.y / wl, wl * DT);
            }
        }

        /* ---------- camara ---------- */
        V3 eye, tgt;
        if (state == ST_MENU || state == ST_CONTROLS) {
            float a = menu_t * 0.2f;
            eye = v3(cosf(a) * 6500.0f, sinf(a) * 8200.0f, 2600.0f);
            tgt = v3(0, 0, 200);
        } else {
            cam_update(&cam, &w.car[0], &w.ball, ballcam, DT, cam_snap);
            cam_snap = 0;
            eye = cam.out_pos;
            tgt = vadd(eye, vmul(cam.out_fwd, 1000.0f));
        }

        /* ---------- render ---------- */
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        begin3d();
        look_at(eye, tgt);
        draw_arena();
        draw_pads(&w, now);
        draw_shadows(&w);
        draw_car(&w.car[1], 0.95f, 0.5f, 0.1f, now);
        draw_car(&w.car[0], 0.15f, 0.4f, 0.95f, now);
        draw_ball(&w.ball);

        begin2d();
        int ui_own = ui_begin();                 /* toda la interfaz de este frame en un solo glBegin */
        if (state == ST_MENU) {
            rect2d(0, 0, SCR_W, SCR_H, 0, 0, 0, 0.4f);
            text_c(48, 9.0f, 1.0f, 0.75f, 0.2f, "VITA LEAGUE");
            text_c(122, 4.0f, 0.6f, 0.85f, 1.0f, "COCHES + PELOTA 3D");
            const char *items[4];
            items[0] = "JUGAR 1 VS 1";
            snprintf(buf, sizeof(buf), "DIFICULTAD: < %s >", DIFF_NAME[diff]);
            items[1] = buf; items[2] = "CONTROLES"; items[3] = "SALIR";
            for (int i = 0; i < 4; i++) {
                float y = 215.0f + i * 62.0f;
                if (i == sel) { rect2d(200, y - 12, 560, 56, 0.15f, 0.4f, 0.95f, 0.55f); text_c(y, 4.5f, 1, 1, 1, items[i]); }
                else text_c(y, 4.5f, 0.8f, 0.85f, 0.95f, items[i]);
            }
            text_c(498, 2.5f, 0.85f, 0.85f, 0.85f, "ARRIBA/ABAJO: ELEGIR   X: ACEPTAR");
        } else if (state == ST_CONTROLS) {
            rect2d(0, 0, SCR_W, SCR_H, 0, 0, 0, 0.72f);
            text_c(24, 5.5f, 1.0f, 0.75f, 0.2f, "CONTROLES");
            const char *lines[9] = {
                "STICK IZQ X: GIRAR / GUINADA EN EL AIRE", "STICK IZQ Y: CABECEO / DIRECCION DEL FLIP",
                "R: ACELERAR    L: FRENO / MARCHA ATRAS", "X: SALTAR (OTRA VEZ: DOBLE SALTO O FLIP)",
                "O: BOOST (TANQUE DE 100, DURA 3 SEG)", "CUADRADO: DERRAPE / GIRO LATERAL EN AIRE",
                "TRIANGULO: BALL CAM / CAR CAM", "START: VOLVER AL MENU", "CHAPAS: PEQUENA +12   GRANDE +100"};
            for (int i = 0; i < 9; i++) text_c(100 + i * 44.0f, 2.9f, 1, 1, 1, lines[i]);
            text_c(505, 2.6f, 0.7f, 0.85f, 1.0f, "O: VOLVER");
        } else {
            /* marcador y reloj */
            rect2d(372, 8, 92, 58, 0.1f, 0.3f, 0.9f, 0.85f);
            rect2d(496, 8, 92, 58, 0.9f, 0.45f, 0.08f, 0.85f);
            snprintf(buf, sizeof(buf), "%d", score_p); text(418 - text_w(6.0f, buf) * 0.5f, 18, 6.0f, 1, 1, 1, buf);
            snprintf(buf, sizeof(buf), "%d", score_a); text(542 - text_w(6.0f, buf) * 0.5f, 18, 6.0f, 1, 1, 1, buf);
            int secs = (int)(clock + 0.99f);
            if (overtime) snprintf(buf, sizeof(buf), "+%d:%02d", (int)clock / 60, (int)clock % 60);
            else snprintf(buf, sizeof(buf), "%d:%02d", secs / 60, secs % 60);
            text_c(72, 3.4f, 1, 1, 1, buf);
            if (overtime) text_c(100, 2.6f, 1.0f, 0.8f, 0.3f, "PRORROGA");
            /* boost y velocidad */
            const Car *pc = &w.car[0];
            rect2d(690, 478, 240, 34, 0, 0, 0, 0.55f);
            rect2d(694, 482, 2.32f * pc->boost, 26, 1.0f, pc->boosting ? 0.9f : 0.65f, 0.1f, 1.0f);
            snprintf(buf, sizeof(buf), "%d", (int)(pc->boost + 0.5f));
            text(700, 486, 3.2f, 1, 1, 1, buf);
            snprintf(buf, sizeof(buf), "%d KM/H", (int)(vlen(pc->v) * 0.036f));
            text(30, 495, 3.0f, 1, 1, 1, buf);
            if (pc->supersonic) text(30, 465, 2.6f, 1.0f, 0.85f, 0.3f, "SUPERSONICO!");

            if (state == ST_COUNT) {
                snprintf(buf, sizeof(buf), "%d", (int)ceilf(timer));
                text_c(170, 14.0f, 1.0f, 0.9f, 0.3f, buf);
            } else if (state == ST_GOAL) {
                text_c(190, 12.0f, last_scorer == 1 ? 0.5f : 1.0f, last_scorer == 1 ? 0.75f : 0.65f, last_scorer == 1 ? 1.0f : 0.25f, "GOOOL!");
            } else if (state == ST_OVER) {
                rect2d(0, 0, SCR_W, SCR_H, 0, 0, 0, 0.55f);
                text_c(130, 10.0f, 1, 1, 1, score_p > score_a ? "GANASTE!" : "PERDISTE");
                snprintf(buf, sizeof(buf), "%d - %d", score_p, score_a);
                text_c(250, 7.0f, 1.0f, 0.8f, 0.3f, buf);
                text_c(380, 3.4f, 0.85f, 0.9f, 1, "TRIANGULO: REVANCHA");
                text_c(430, 3.4f, 0.85f, 0.9f, 1, "X: MENU");
            }
        }
        ui_end(ui_own);
        vglSwapBuffers(GL_FALSE);
    }
    sceKernelExitProcess(0);
    return 0;
}
