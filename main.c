/*
 * Vita League - mini "coches con pelota" para PS Vita (VitaSDK + vita2d)
 *
 * Controles:
 *   Stick izq. / D-pad izq-der : girar
 *   R o Cruz                    : acelerar
 *   L o Cuadrado                : marcha atras / freno
 *   Circulo                     : boost
 *   Triangulo                   : reiniciar partido (al terminar)
 *   Start                       : salir
 */
#include <psp2/ctrl.h>
#include <psp2/kernel/processmgr.h>
#include <vita2d.h>
#include <math.h>
#include <stdio.h>

#define SCR_W 960
#define SCR_H 544
#define FL 50.0f      /* campo: izquierda */
#define FR 910.0f     /* derecha */
#define FT 50.0f      /* arriba */
#define FB 494.0f     /* abajo */
#define GOAL_T 192.0f /* porteria: borde superior */
#define GOAL_B 352.0f /* porteria: borde inferior */
#define DT (1.0f / 60.0f)
#define WIN_SCORE 5
#define PI 3.14159265f

enum { ST_PLAY, ST_GOAL, ST_OVER };

typedef struct { float x, y, vx, vy, r, m; } Body;
typedef struct { Body b; float ang; float boost; int boosting; } Car;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static float wrap_angle(float a) {
    while (a > PI) a -= 2 * PI;
    while (a < -PI) a += 2 * PI;
    return a;
}

/* Colision circulo-circulo con impulso segun masas */
static int collide(Body *a, Body *b, float rest) {
    float dx = b->x - a->x, dy = b->y - a->y;
    float d2 = dx * dx + dy * dy, rr = a->r + b->r;
    if (d2 >= rr * rr || d2 < 0.0001f) return 0;
    float d = sqrtf(d2), nx = dx / d, ny = dy / d;
    float tot = a->m + b->m, pen = rr - d;
    a->x -= nx * pen * (b->m / tot);
    a->y -= ny * pen * (b->m / tot);
    b->x += nx * pen * (a->m / tot);
    b->y += ny * pen * (a->m / tot);
    float rv = (b->vx - a->vx) * nx + (b->vy - a->vy) * ny;
    if (rv < 0) {
        float j = -(1.0f + rest) * rv / (1.0f / a->m + 1.0f / b->m);
        a->vx -= j * nx / a->m; a->vy -= j * ny / a->m;
        b->vx += j * nx / b->m; b->vy += j * ny / b->m;
    }
    return 1;
}

/* Paredes del campo. La pelota puede entrar por la boca de la porteria. */
static void walls(Body *b, int is_ball, float rest) {
    if (b->y - b->r < FT) { b->y = FT + b->r; if (b->vy < 0) b->vy = -b->vy * rest; }
    if (b->y + b->r > FB) { b->y = FB - b->r; if (b->vy > 0) b->vy = -b->vy * rest; }
    int in_goal = is_ball && b->y > GOAL_T && b->y < GOAL_B;
    if (!in_goal) {
        if (b->x - b->r < FL) { b->x = FL + b->r; if (b->vx < 0) b->vx = -b->vx * rest; }
        if (b->x + b->r > FR) { b->x = FR - b->r; if (b->vx > 0) b->vx = -b->vx * rest; }
    }
}

static void update_car(Car *c, float steer, float throttle, int boost) {
    float fx = cosf(c->ang), fy = sinf(c->ang);
    float vf = c->b.vx * fx + c->b.vy * fy;

    /* giro: depende de la velocidad, se invierte al ir marcha atras */
    float turn = clampf(fabsf(vf) / 120.0f, 0.0f, 1.0f);
    c->ang += steer * 3.4f * turn * (vf < -5.0f ? -1.0f : 1.0f) * DT;
    c->ang = wrap_angle(c->ang);

    fx = cosf(c->ang); fy = sinf(c->ang);
    float sx = -fy, sy = fx;
    vf = c->b.vx * fx + c->b.vy * fy;
    float vs = c->b.vx * sx + c->b.vy * sy;

    c->boosting = boost && c->boost > 0.0f;
    vf += throttle * 520.0f * DT;
    if (c->boosting) {
        vf += 1100.0f * DT;
        c->boost -= 33.0f * DT;
        if (c->boost < 0.0f) c->boost = 0.0f;
    } else {
        c->boost = clampf(c->boost + 10.0f * DT, 0.0f, 100.0f);
    }

    float limit = c->boosting ? 620.0f : 340.0f;
    if (vf > limit) vf -= (vf - limit) * clampf(3.0f * DT, 0.0f, 1.0f);
    if (vf < -220.0f) vf = -220.0f;

    vf *= 1.0f - (throttle == 0.0f ? 1.5f : 0.5f) * DT;   /* rozamiento */
    vs *= 1.0f - 7.0f * DT;                                /* agarre lateral */

    c->b.vx = fx * vf + sx * vs;
    c->b.vy = fy * vf + sy * vs;
    c->b.x += c->b.vx * DT;
    c->b.y += c->b.vy * DT;
}

static void ai_control(const Car *ai, const Body *ball, float *steer, float *thr, int *boost) {
    /* la IA ataca la porteria izquierda */
    float gx = FL, gy = (GOAL_T + GOAL_B) * 0.5f;
    float dx = gx - ball->x, dy = gy - ball->y;
    float dl = sqrtf(dx * dx + dy * dy) + 0.001f;
    float tx, ty;

    float adx = ball->x - ai->b.x, ady = ball->y - ai->b.y;
    float dist = sqrtf(adx * adx + ady * ady);

    if (ai->b.x > ball->x) {
        /* estoy detras de la pelota respecto a mi objetivo: colocarme y empujar */
        tx = ball->x - dx / dl * 40.0f;
        ty = ball->y - dy / dl * 40.0f;
        if (dist < 110.0f) { tx = ball->x; ty = ball->y; }
    } else {
        /* estoy delante: rodear la pelota para no meter autogol */
        tx = ball->x + 60.0f;
        ty = ball->y + (ai->b.y < ball->y ? -90.0f : 90.0f);
    }

    float diff = wrap_angle(atan2f(ty - ai->b.y, tx - ai->b.x) - ai->ang);
    *steer = clampf(diff * 2.0f, -1.0f, 1.0f);
    *thr = fabsf(diff) < 1.3f ? 0.9f : 0.2f;
    *boost = (fabsf(diff) < 0.15f && dist > 260.0f && ai->boost > 25.0f);
}

static void reset_round(Car *p, Car *ai, Body *ball) {
    p->b = (Body){FL + 150.0f, 272.0f, 0, 0, 18.0f, 3.0f};
    p->ang = 0.0f; p->boost = 100.0f; p->boosting = 0;
    ai->b = (Body){FR - 150.0f, 272.0f, 0, 0, 18.0f, 3.0f};
    ai->ang = PI; ai->boost = 100.0f; ai->boosting = 0;
    *ball = (Body){480.0f, 272.0f, 0, 0, 14.0f, 1.0f};
}

static void text_center(vita2d_pgf *pgf, int y, float s, unsigned int col, const char *t) {
    int w = vita2d_pgf_text_width(pgf, s, t);
    vita2d_pgf_draw_text(pgf, (SCR_W - w) / 2, y, col, s, t);
}

static void draw_car(const Car *c, unsigned int col, unsigned int nose) {
    float fx = cosf(c->ang), fy = sinf(c->ang);
    if (c->boosting)
        vita2d_draw_fill_circle(c->b.x - fx * (c->b.r + 6), c->b.y - fy * (c->b.r + 6), 8, RGBA8(255, 160, 30, 255));
    vita2d_draw_fill_circle(c->b.x, c->b.y, c->b.r, col);
    vita2d_draw_fill_circle(c->b.x + fx * 11.0f, c->b.y + fy * 11.0f, 8.0f, nose);
}

static void draw_field(void) {
    unsigned int white = RGBA8(240, 240, 240, 255);
    vita2d_draw_rectangle(FL, FT, FR - FL, FB - FT, RGBA8(34, 120, 60, 255));
    /* porterias */
    vita2d_draw_rectangle(FL - 34, GOAL_T, 34, GOAL_B - GOAL_T, RGBA8(40, 90, 220, 255));
    vita2d_draw_rectangle(FR, GOAL_T, 34, GOAL_B - GOAL_T, RGBA8(230, 120, 30, 255));
    /* bordes */
    vita2d_draw_rectangle(FL - 4, FT - 4, FR - FL + 8, 4, white);
    vita2d_draw_rectangle(FL - 4, FB, FR - FL + 8, 4, white);
    vita2d_draw_rectangle(FL - 4, FT - 4, 4, GOAL_T - FT + 4, white);
    vita2d_draw_rectangle(FL - 4, GOAL_B, 4, FB - GOAL_B + 4, white);
    vita2d_draw_rectangle(FR, FT - 4, 4, GOAL_T - FT + 4, white);
    vita2d_draw_rectangle(FR, GOAL_B, 4, FB - GOAL_B + 4, white);
    /* linea central + circulo */
    vita2d_draw_rectangle(478, FT, 4, FB - FT, RGBA8(255, 255, 255, 120));
    for (int i = 0; i < 40; i++) {
        float a0 = i * 2 * PI / 40, a1 = (i + 1) * 2 * PI / 40;
        vita2d_draw_line(480 + cosf(a0) * 70, 272 + sinf(a0) * 70,
                         480 + cosf(a1) * 70, 272 + sinf(a1) * 70, RGBA8(255, 255, 255, 120));
    }
}

int main(void) {
    vita2d_init();
    vita2d_set_clear_color(RGBA8(20, 24, 32, 255));
    vita2d_pgf *pgf = vita2d_load_default_pgf();
    sceCtrlSetSamplingMode(SCE_CTRL_MODE_ANALOG);

    Car pl, ai;
    Body ball;
    int score_p = 0, score_a = 0, state = ST_PLAY, last_scorer = 0;
    float timer = 0.0f;
    unsigned int prev = 0;
    char buf[64];

    reset_round(&pl, &ai, &ball);

    for (;;) {
        SceCtrlData pad;
        sceCtrlPeekBufferPositive(0, &pad, 1);
        unsigned int pressed = pad.buttons & ~prev;
        prev = pad.buttons;
        if (pad.buttons & SCE_CTRL_START) break;

        if (state == ST_PLAY) {
            float steer = ((int)pad.lx - 128) / 128.0f;
            if (fabsf(steer) < 0.15f) steer = 0.0f;
            if (pad.buttons & SCE_CTRL_LEFT) steer = -1.0f;
            if (pad.buttons & SCE_CTRL_RIGHT) steer = 1.0f;

            float thr = 0.0f;
            if (pad.buttons & (SCE_CTRL_RTRIGGER | SCE_CTRL_CROSS)) thr += 1.0f;
            if (pad.buttons & (SCE_CTRL_LTRIGGER | SCE_CTRL_SQUARE)) thr -= 1.0f;
            int boost = (pad.buttons & SCE_CTRL_CIRCLE) != 0;

            update_car(&pl, steer, thr, boost);

            float as, at; int ab;
            ai_control(&ai, &ball, &as, &at, &ab);
            update_car(&ai, as, at, ab);

            /* pelota */
            ball.vx *= 1.0f - 0.35f * DT;
            ball.vy *= 1.0f - 0.35f * DT;
            float sp = sqrtf(ball.vx * ball.vx + ball.vy * ball.vy);
            if (sp > 1000.0f) { ball.vx *= 1000.0f / sp; ball.vy *= 1000.0f / sp; }
            ball.x += ball.vx * DT;
            ball.y += ball.vy * DT;

            collide(&pl.b, &ai.b, 0.5f);
            collide(&pl.b, &ball, 0.8f);
            collide(&ai.b, &ball, 0.8f);

            walls(&pl.b, 0, 0.3f);
            walls(&ai.b, 0, 0.3f);
            walls(&ball, 1, 0.8f);

            if (ball.x < FL - ball.r) { score_a++; last_scorer = 2; state = ST_GOAL; timer = 1.5f; }
            else if (ball.x > FR + ball.r) { score_p++; last_scorer = 1; state = ST_GOAL; timer = 1.5f; }
            if (state == ST_GOAL && (score_p >= WIN_SCORE || score_a >= WIN_SCORE)) state = ST_OVER;
        } else if (state == ST_GOAL) {
            timer -= DT;
            if (timer <= 0.0f) { reset_round(&pl, &ai, &ball); state = ST_PLAY; }
        } else if (state == ST_OVER) {
            if (pressed & SCE_CTRL_TRIANGLE) {
                score_p = score_a = 0;
                reset_round(&pl, &ai, &ball);
                state = ST_PLAY;
            }
        }

        /* ---- dibujo ---- */
        vita2d_start_drawing();
        vita2d_clear_screen();

        draw_field();
        draw_car(&ai, RGBA8(230, 120, 30, 255), RGBA8(255, 210, 150, 255));
        draw_car(&pl, RGBA8(40, 90, 220, 255), RGBA8(160, 200, 255, 255));
        vita2d_draw_fill_circle(ball.x, ball.y, ball.r + 2, RGBA8(60, 60, 60, 255));
        vita2d_draw_fill_circle(ball.x, ball.y, ball.r, RGBA8(250, 250, 250, 255));

        /* HUD */
        snprintf(buf, sizeof(buf), "%d  -  %d", score_p, score_a);
        text_center(pgf, 34, 1.6f, RGBA8(255, 255, 255, 255), buf);

        vita2d_draw_rectangle(FL, 510, 204, 18, RGBA8(0, 0, 0, 160));
        vita2d_draw_rectangle(FL + 2, 512, 2.0f * pl.boost, 14, RGBA8(255, 190, 40, 255));
        vita2d_pgf_draw_text(pgf, FL + 212, 525, RGBA8(255, 255, 255, 255), 1.0f, "BOOST");

        if (state == ST_GOAL) {
            text_center(pgf, 280, 2.4f,
                        last_scorer == 1 ? RGBA8(120, 170, 255, 255) : RGBA8(255, 170, 90, 255), "GOOOL!");
        } else if (state == ST_OVER) {
            text_center(pgf, 250, 2.2f, RGBA8(255, 255, 255, 255),
                        score_p > score_a ? "GANASTE!" : "PERDISTE");
            text_center(pgf, 310, 1.1f, RGBA8(220, 220, 220, 255), "Triangulo: revancha   Start: salir");
        }

        vita2d_end_drawing();
        vita2d_swap_buffers();
    }

    vita2d_fini();
    vita2d_free_pgf(pgf);
    sceKernelExitProcess(0);
    return 0;
}
