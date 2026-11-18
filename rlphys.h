/*
 * rlphys.h - nucleo de fisica estilo Rocket League (Unreal Units, Z arriba)
 * Sin dependencias de consola: se puede probar en cualquier PC.
 *
 * Ejes del mundo: X = ancho, Y = largo (porterias en Y = +-5120), Z = altura.
 * Coche: f = adelante, u = arriba, r = cross(f, u) = derecha.
 */
#ifndef RLPHYS_H
#define RLPHYS_H
#include <math.h>
#include <string.h>

/* ---------------- medidas oficiales (uu) ---------------- */
#define ARENA_HX   4096.0f      /* mitad del ancho (8192)  */
#define ARENA_HY   5120.0f      /* mitad del largo (10240) */
#define ARENA_H    2044.0f      /* techo                   */
#define GOAL_HW     893.0f      /* mitad ancho porteria (1786) */
#define GOAL_H      642.0f      /* alto porteria           */
#define GOAL_D      880.0f      /* profundidad porteria    */
#define PLAN_R     1000.0f      /* radio de las esquinas (vista cenital) */
#define RAMP_R      280.0f      /* radio de la rampa suelo/pared/techo   */

#define GRAV       (-650.0f)
#define BALL_R       91.25f
#define BALL_E        0.6f
#define BALL_MAXV  6000.0f
#define BALL_DRAG     0.03f

#define CAR_HL   59.005f        /* Octane 118.01 x 84.20 x 36.16 */
#define CAR_HW   42.10f
#define CAR_HH   18.08f
#define CAR_REST 17.0f
#define CAR_MAXV 2300.0f

#define PI_F 3.14159265f

/* ---------------- vectores ---------------- */
typedef struct { float x, y, z; } V3;
static inline V3 v3(float x, float y, float z) { V3 r = {x, y, z}; return r; }
static inline V3 vadd(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 vsub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 vmul(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float vdot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 vcross(V3 a, V3 b) {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline float vlen(V3 a) { return sqrtf(vdot(a, a)); }
static inline V3 vnorm(V3 a) { float l = vlen(a); return l > 1e-6f ? vmul(a, 1.0f / l) : v3(0, 0, 1); }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static inline V3 vrot(V3 v, V3 k, float a) {   /* Rodrigues: rota v alrededor del eje unitario k */
    float c = cosf(a), s = sinf(a);
    return vadd(vadd(vmul(v, c), vmul(vcross(k, v), s)), vmul(k, vdot(k, v) * (1.0f - c)));
}

/* ---------------- arena (campo de distancias con signo) ---------------- */
/* plan: distancia interior al borde del rectangulo redondeado y normal 2D hacia dentro */
static void plan_dist(float px, float py, float *d, float *nx, float *ny) {
    float qx = fabsf(px) - (ARENA_HX - PLAN_R), qy = fabsf(py) - (ARENA_HY - PLAN_R);
    float sx = px < 0 ? -1.0f : 1.0f, sy = py < 0 ? -1.0f : 1.0f;
    if (qx > 0 || qy > 0) {
        float mx = fmaxf(qx, 0), my = fmaxf(qy, 0), l = sqrtf(mx * mx + my * my);
        if (l < 1e-4f) l = 1e-4f;
        *d = PLAN_R - l; *nx = -sx * mx / l; *ny = -sy * my / l;
    } else if (-qx < -qy) { *d = PLAN_R - qx; *nx = -sx; *ny = 0; }
    else { *d = PLAN_R - qy; *nx = 0; *ny = -sy; }
}

/* perfil pared/suelo con rampa redondeada: a = dist. a la pared, b = dist. al suelo */
static void profile(float a, float b, float *d, float *na, float *nb) {
    if (a < RAMP_R && b < RAMP_R) {
        float dx = RAMP_R - a, dy = RAMP_R - b, l = sqrtf(dx * dx + dy * dy);
        if (l < 1e-4f) l = 1e-4f;
        *d = RAMP_R - l; *na = dx / l; *nb = dy / l;
    } else if (a < b) { *d = a; *na = 1; *nb = 0; }
    else { *d = b; *na = 0; *nb = 1; }
}

static void arena_main(V3 p, float *d, V3 *n) {
    float pd, nx, ny, d1, a1, b1, d2, a2, b2;
    plan_dist(p.x, p.y, &pd, &nx, &ny);
    profile(pd, p.z, &d1, &a1, &b1);
    profile(pd, ARENA_H - p.z, &d2, &a2, &b2);
    if (d1 <= d2) { *d = d1; *n = v3(nx * a1, ny * a1, b1); }
    else { *d = d2; *n = v3(nx * a2, ny * a2, -b2); }
}

static void goal_box(V3 p, float side, float *d, V3 *n) {
    float ay = side * p.y;
    float c[5] = {GOAL_HW - fabsf(p.x), p.z, GOAL_H - p.z, (ARENA_HY + GOAL_D) - ay, ay - (ARENA_HY - 600.0f)};
    V3 nn[5] = {v3(p.x < 0 ? 1.0f : -1.0f, 0, 0), v3(0, 0, 1), v3(0, 0, -1), v3(0, -side, 0), v3(0, side, 0)};
    int k = 0;
    for (int i = 1; i < 5; i++) if (c[i] < c[k]) k = i;
    *d = c[k]; *n = nn[k];
}

/* d > 0 dentro de la arena; n = normal de la superficie mas cercana (hacia dentro) */
static void arena(V3 p, float *d, V3 *n) {
    arena_main(p, d, n);
    for (int s = -1; s <= 1; s += 2) {
        float gd; V3 gn;
        goal_box(p, (float)s, &gd, &gn);
        if (gd > *d) { *d = gd; *n = gn; }
    }
}

/* ---------------- entidades ---------------- */
typedef struct {
    V3 p, v, f, u, w;           /* posicion, velocidad, ejes, vel. angular */
    float boost; int boosting;
    int wheels, touching; V3 gn;   /* contacto con superficie */
    float air_t, jump_hold, flip_t; V3 flip_w;
    int jumped, can_flip, prev_jump, supersonic;
} Car;

typedef struct { float steer, pitch, throttle; int boost, jump, handbrake; } Input;
typedef struct { V3 p, v; } Ball;

#define NPADS 34
static const float PAD_XY[NPADS][2] = {
    {-3072,-4096},{3072,-4096},{-3584,0},{3584,0},{-3072,4096},{3072,4096},          /* 6 grandes */
    {0,-4240},{-1792,-4184},{1792,-4184},{-940,-3308},{940,-3308},{0,-2816},
    {-3584,-2484},{3584,-2484},{-1788,-2300},{1788,-2300},{-2048,-1036},{0,-1024},
    {2048,-1036},{-1024,0},{1024,0},{-2048,1036},{0,1024},{2048,1036},
    {-1788,2300},{1788,2300},{-3584,2484},{3584,2484},{0,2816},{-940,3308},
    {940,3308},{-1792,4184},{1792,4184},{0,4240}};
#define PAD_IS_BIG(i) ((i) < 6)

typedef struct { Car car[2]; Ball ball; float pad_t[NPADS]; } World;

static void car_reset(Car *c, float x, float y, float dirx, float diry) {
    memset(c, 0, sizeof(*c));
    c->p = v3(x, y, CAR_REST);
    c->f = vnorm(v3(dirx, diry, 0));
    c->u = v3(0, 0, 1);
    c->gn = v3(0, 0, 1);
    c->boost = 33.0f;
    c->can_flip = 1;
}

static void world_reset(World *w) {
    car_reset(&w->car[0], 0, -4608, 0, 1);
    car_reset(&w->car[1], 0, 4608, 0, -1);
    w->ball.p = v3(0, 0, BALL_R);
    w->ball.v = v3(0, 0, 0);
}
static void pads_reset(World *w) { for (int i = 0; i < NPADS; i++) w->pad_t[i] = 0; }

/* ---------------- coche ---------------- */
static float curvature(float s) {
    static const float S[6] = {0, 500, 1000, 1500, 1750, 2300};
    static const float C[6] = {0.0069f, 0.00398f, 0.00235f, 0.001375f, 0.0011f, 0.00088f};
    s = fabsf(s);
    for (int i = 0; i < 5; i++)
        if (s <= S[i + 1]) return C[i] + (C[i + 1] - C[i]) * (s - S[i]) / (S[i + 1] - S[i]);
    return C[5];
}

static void car_ortho(Car *c) {
    c->f = vnorm(c->f);
    c->u = vnorm(vsub(c->u, vmul(c->f, vdot(c->u, c->f))));
}

static void car_arena(Car *c) {
    V3 r = vcross(c->f, c->u);
    int was_on = c->wheels;
    V3 gprev = c->gn;
    float s0 = vlen(c->v);
    for (int pass = 0; pass < 2; pass++)
        for (int i = 0; i < 8; i++) {
            float sx = (i & 1) ? 1.0f : -1.0f, sy = (i & 2) ? 1.0f : -1.0f, sz = (i & 4) ? 1.0f : -1.0f;
            V3 q = vadd(c->p, vadd(vmul(c->f, sx * CAR_HL), vadd(vmul(r, sy * CAR_HW), vmul(c->u, sz * CAR_HH))));
            float d; V3 n;
            arena(q, &d, &n);
            if (d < 0) {
                c->p = vadd(c->p, vmul(n, -d));
                float vn = vdot(c->v, n);
                if (vn < 0) {
                    c->v = vsub(c->v, vmul(n, vn));
                    /* sobre una rampa curva el coche "sigue" la superficie: se desvia la velocidad
                       hacia la tangente en vez de frenarla (solo si venia sobre ruedas y la normal
                       de contacto es casi la del suelo que pisaba) */
                    if (was_on && vdot(n, gprev) > 0.6f) {
                        float st = vlen(c->v);
                        if (st > 1e-3f && st < s0) c->v = vmul(c->v, s0 / st);
                    }
                }
            }
        }

    /* puntos de contacto superiores/laterales (para detectar "tocando algo") */
    int nt = 0;
    V3 sa = v3(0, 0, 0);
    for (int i = 0; i < 8; i++) {
        float sx = (i & 1) ? 1.0f : -1.0f, sy = (i & 2) ? 1.0f : -1.0f, sz = (i & 4) ? 1.0f : -1.0f;
        V3 q = vadd(c->p, vadd(vmul(c->f, sx * CAR_HL), vadd(vmul(r, sy * CAR_HW), vmul(c->u, sz * CAR_HH))));
        float d; V3 n;
        arena(q, &d, &n);
        if (d < 6.0f) { nt++; sa = vadd(sa, n); }
    }

    /* base del coche: 4 esquinas + 4 puntos medios de los bordes + centro = 9 muestras.
       Margen de 25 uu (alcance de la suspension) para no perder el contacto en la curva. */
    static const float SX[9] = {-1, 1, -1, 1, 0, 0, -1, 1, 0};
    static const float SY[9] = {-1, -1, 1, 1, -1, 1, 0, 0, 0};
    int nb = 0;
    V3 sb = v3(0, 0, 0);
    for (int i = 0; i < 9; i++) {
        V3 q = vadd(c->p, vadd(vmul(c->f, SX[i] * CAR_HL), vadd(vmul(r, SY[i] * CAR_HW), vmul(c->u, -CAR_HH))));
        float d; V3 n;
        arena(q, &d, &n);
        if (d < 25.0f) { nb++; sb = vadd(sb, n); }
    }

    c->touching = nt >= 2;
    c->wheels = 0;
    if (nb >= 1) {
        V3 g = vnorm(sb);
        float ug = vdot(c->u, g), vg = vdot(c->v, g);
        /* 2+ puntos de la base y el coche no se aleja de la superficie; 1 solo punto basta
           mientras ya iba pegado (transicion de la rampa) */
        if ((nb >= 2 && ug > 0.2f && vg < 250.0f) || (was_on && ug > 0.5f && vg < 150.0f)) {
            c->wheels = 1; c->gn = g;
        }
    }
    if (c->touching && !c->wheels) {
        V3 g = vnorm(sa);
        c->gn = g;
        if (vdot(c->u, g) > 0.3f && fabsf(vdot(c->f, g)) < 0.75f && c->flip_t <= 0.0f) c->wheels = 1;   /* aterrizaje inclinado */
    }
}

static void car_step(Car *c, const Input *in, float dt) {
    V3 r = vcross(c->f, c->u);
    int jump_edge = in->jump && !c->prev_jump;
    c->prev_jump = in->jump;
    int on = c->wheels;

    if (c->touching && c->jump_hold <= 0.0f && vdot(c->v, c->u) < 120.0f) {
        c->jumped = 0; c->can_flip = 1; c->air_t = 0; c->flip_t = 0;
    } else if (!c->touching) c->air_t += dt;

    /* ----- salto, doble salto y dodge ----- */
    if (jump_edge) {
        if (on && !c->jumped) {
            c->v = vadd(c->v, vmul(c->u, 292.0f));
            c->jumped = 1; c->jump_hold = 0.0001f; on = 0; c->wheels = 0;
        } else if (!on && c->can_flip && c->air_t < 1.25f) {
            c->can_flip = 0;
            float a = in->pitch, s = in->steer, m = sqrtf(a * a + s * s);
            if (m > 0.35f) {
                V3 fh = v3(c->f.x, c->f.y, 0);
                fh = vlen(fh) > 0.2f ? vnorm(fh) : v3(0, 1, 0);
                V3 rh = vcross(fh, v3(0, 0, 1));
                a /= m; s /= m;
                c->v = vadd(c->v, vadd(vmul(fh, a * 500.0f), vmul(rh, s * 500.0f)));
                if (c->v.z < 0) c->v.z = 0;
                c->flip_w = vadd(vmul(r, -a * 5.5f), vmul(c->f, s * 5.5f));
                c->flip_t = 0.65f;
            } else c->v = vadd(c->v, vmul(c->u, 292.0f));
        }
    }
    if (c->jump_hold > 0.0f) {
        if (in->jump && c->jump_hold < 0.2f) { c->v = vadd(c->v, vmul(c->u, 1458.33f * dt)); c->jump_hold += dt; }
        else c->jump_hold = -1.0f;
    }

    /* ----- boost ----- */
    c->boosting = in->boost && c->boost > 0.0f;
    if (c->boosting) c->boost = fmaxf(0.0f, c->boost - 33.3f * dt);
    float gscale = 1.0f;       /* fraccion de gravedad que se aplica (0 pegado a pared/techo) */

    if (on) {
        /* ----- conduccion en suelo / pared ----- */
        V3 n = c->gn;
        V3 ft = vsub(c->f, vmul(n, vdot(c->f, n)));
        ft = vlen(ft) > 0.2f ? vnorm(ft) : c->f;
        V3 rt = vcross(ft, n);
        float vf = vdot(c->v, ft), vr = vdot(c->v, rt), vn = vdot(c->v, n);
        float th = in->throttle;
        if (th != 0.0f) {
            if (th * vf < 0.0f) {                         /* frenar */
                float dv = 3500.0f * dt;
                if (fabsf(vf) < dv) vf = 0; else vf -= (vf > 0 ? dv : -dv);
            } else {
                float av = fabsf(vf);
                float acc = av < 1400.0f ? lerpf(1600.0f, 160.0f, av / 1400.0f) : lerpf(160.0f, 0.0f, (av - 1400.0f) / 10.0f);
                if (acc > 0) vf += th * acc * dt;
            }
        } else {
            float dv = 525.0f * dt;
            if (fabsf(vf) < dv) vf = 0; else vf -= (vf > 0 ? dv : -dv);
        }
        if (c->boosting) vf += 991.667f * dt;
        vf = clampf(vf, -CAR_MAXV, CAR_MAXV);
        /* giro segun la tabla de curvatura */
        float k = curvature(vf) * (in->handbrake ? 1.6f : 1.0f);
        float yaw = in->steer * k * vf;
        c->f = vrot(c->f, c->u, -yaw * dt);
        r = vcross(c->f, c->u);
        /* agarre lateral (derrape con freno de mano) */
        vr *= expf(-(in->handbrake ? 1.5f : 14.0f) * dt);
        if (in->handbrake) vf *= expf(-0.35f * dt);
        ft = vsub(c->f, vmul(n, vdot(c->f, n)));
        ft = vlen(ft) > 0.2f ? vnorm(ft) : c->f;
        rt = vcross(ft, n);
        c->v = vadd(vadd(vmul(ft, vf), vmul(rt, vr)), vmul(n, vn));
        c->v = vsub(c->v, vmul(n, 325.0f * dt));           /* ruedas pegajosas */
        /* gravedad: en paredes y techo las ruedas pegajosas sustituyen a la gravedad.
           Suelo/rampas suaves: gravedad normal. Pared vertical: ninguna. Techo: solo se sostiene
           si va con algo de velocidad (a <600 uu/s se cae). */
        gscale = clampf(n.z / 0.6f, 0.0f, 1.0f);
        if (n.z < -0.2f) gscale = clampf(1.0f - fabsf(vf) / 600.0f, 0.0f, 1.0f);
        /* alinear el coche con la superficie, anticipando la normal que viene (60 ms por delante) */
        V3 nt = n;
        {
            V3 vt = vsub(c->v, vmul(n, vdot(c->v, n)));
            V3 q = vsub(vadd(c->p, vmul(vt, 0.06f)), vmul(c->u, CAR_HH));
            float da; V3 na;
            arena(q, &da, &na);
            if (da < 60.0f && vdot(na, n) > 0.2f) nt = vnorm(vadd(n, na));
        }
        V3 ax = vcross(c->u, nt);
        float s = vlen(ax);
        if (s > 1e-5f) {
            float k = 1.0f - expf(-40.0f * dt);
            float ang = asinf(clampf(s, 0, 1)) * k;
            if (vdot(c->u, nt) < 0) ang = (PI_F - asinf(clampf(s, 0, 1))) * k;
            ax = vmul(ax, 1.0f / s);
            c->f = vrot(c->f, ax, ang); c->u = vrot(c->u, ax, ang);
        }
        car_ortho(c);
        c->w = v3(0, 0, 0);
    } else {
        /* ----- en el aire ----- */
        if (c->boosting) c->v = vadd(c->v, vmul(c->f, 1058.33f * dt));
        if (c->flip_t > 0.0f) { c->w = c->flip_w; c->flip_t -= dt; }
        else {
            float pitch = in->pitch, yawin = in->handbrake ? 0.0f : in->steer, roll = in->handbrake ? in->steer : 0.0f;
            float wp = vdot(c->w, r) - pitch * 12.46f * dt;
            float wy = vdot(c->w, c->u) - yawin * 9.11f * dt;
            float wr = vdot(c->w, c->f) + roll * 38.34f * dt;
            float damp = expf(-2.8f * dt);
            if (fabsf(pitch) < 0.05f) wp *= damp;
            if (fabsf(yawin) < 0.05f) wy *= damp;
            if (fabsf(roll) < 0.05f) wr *= damp;
            c->w = vadd(vadd(vmul(r, wp), vmul(c->u, wy)), vmul(c->f, wr));
            float wl = vlen(c->w);
            if (wl > 5.5f) c->w = vmul(c->w, 5.5f / wl);
        }
        float wl = vlen(c->w);
        if (wl > 1e-4f) {
            V3 k = vmul(c->w, 1.0f / wl);
            c->f = vrot(c->f, k, wl * dt); c->u = vrot(c->u, k, wl * dt);
            car_ortho(c);
        }
        /* si esta tocando algo sin ruedas (techo/costado), se endereza solo */
        if (c->touching && c->flip_t <= 0.0f) {
            V3 ax = vcross(c->u, c->gn);
            float s = vlen(ax);
            if (s > 1e-4f) {
                float ang = asinf(clampf(s, 0, 1)) * (1.0f - expf(-4.0f * dt));
                if (vdot(c->u, c->gn) < 0) ang = 1.2f * dt * 3.0f;
                ax = vmul(ax, 1.0f / s);
                c->f = vrot(c->f, ax, ang); c->u = vrot(c->u, ax, ang);
                car_ortho(c);
            }
        }
    }

    c->v.z += GRAV * gscale * dt;
    float sp = vlen(c->v);
    if (sp > CAR_MAXV) c->v = vmul(c->v, CAR_MAXV / sp);
    c->supersonic = sp > (c->supersonic ? 2190.0f : 2200.0f);
    c->p = vadd(c->p, vmul(c->v, dt));
    car_arena(c);
}

/* ---------------- pelota ---------------- */
static void ball_step(Ball *b, float dt) {
    b->v.z += GRAV * dt;
    b->v = vsub(b->v, vmul(b->v, BALL_DRAG * dt));
    float sp = vlen(b->v);
    if (sp > BALL_MAXV) b->v = vmul(b->v, BALL_MAXV / sp);
    b->p = vadd(b->p, vmul(b->v, dt));
    float d; V3 n;
    arena(b->p, &d, &n);
    if (d < BALL_R) {
        b->p = vadd(b->p, vmul(n, BALL_R - d));
        float vn = vdot(b->v, n);
        if (vn < 0) {
            V3 vt = vsub(b->v, vmul(n, vn));
            float e = (-vn > 25.0f) ? BALL_E : 0.0f;
            float dn = (1.0f + e) * (-vn);
            float vts = vlen(vt), red = fminf(vts, 0.2f * dn);
            if (vts > 1e-3f) vt = vmul(vt, 1.0f - red / vts);
            b->v = vadd(vt, vmul(n, -e * vn));
        }
    }
}

static float hit_scale(float rs) {            /* impulso extra de Psyonix segun velocidad relativa */
    if (rs < 2300.0f) return lerpf(0.65f, 0.55f, rs / 2300.0f);
    return lerpf(0.55f, 0.30f, clampf((rs - 2300.0f) / 2300.0f, 0, 1));
}

static void ball_car(Ball *b, Car *c) {
    V3 r = vcross(c->f, c->u), d = vsub(b->p, c->p);
    float lx = clampf(vdot(d, c->f), -CAR_HL, CAR_HL);
    float ly = clampf(vdot(d, r), -CAR_HW, CAR_HW);
    float lz = clampf(vdot(d, c->u), -CAR_HH, CAR_HH);
    V3 q = vadd(c->p, vadd(vmul(c->f, lx), vadd(vmul(r, ly), vmul(c->u, lz))));
    V3 df = vsub(b->p, q);
    float dist = vlen(df);
    if (dist >= BALL_R) return;
    V3 n = dist > 1e-3f ? vmul(df, 1.0f / dist) : vnorm(vsub(b->p, c->p));
    b->p = vadd(b->p, vmul(n, BALL_R - dist));
    V3 rv = vsub(b->v, c->v);
    float vn = vdot(rv, n);
    if (vn < 0) {
        float mb = 30.0f, mc = 180.0f, j = -vn / (1.0f / mb + 1.0f / mc);
        b->v = vadd(b->v, vmul(n, j / mb));
        c->v = vsub(c->v, vmul(n, j / mc));
        V3 hn = vsub(b->p, c->p); hn.z *= 0.35f; hn = vnorm(hn);
        float rs = vlen(rv);
        b->v = vadd(b->v, vmul(hn, rs * hit_scale(rs)));
    }
}

static void car_car(Car *a, Car *b) {
    V3 d = vsub(b->p, a->p);
    float l = vlen(d), R = 70.0f;
    if (l >= 2 * R || l < 1e-3f) return;
    V3 n = vmul(d, 1.0f / l);
    float pen = 2 * R - l;
    a->p = vsub(a->p, vmul(n, pen * 0.5f)); b->p = vadd(b->p, vmul(n, pen * 0.5f));
    float rv = vdot(vsub(b->v, a->v), n);
    if (rv < 0) { float j = -1.3f * rv * 0.5f; a->v = vsub(a->v, vmul(n, j)); b->v = vadd(b->v, vmul(n, j)); }
}

static void pads_step(World *w, float dt) {
    for (int i = 0; i < NPADS; i++) {
        if (w->pad_t[i] > 0) { w->pad_t[i] -= dt; continue; }
        for (int k = 0; k < 2; k++) {
            Car *c = &w->car[k];
            float dx = c->p.x - PAD_XY[i][0], dy = c->p.y - PAD_XY[i][1];
            float rad = PAD_IS_BIG(i) ? 208.0f : 144.0f;
            if (dx * dx + dy * dy < rad * rad && c->p.z < 170.0f) {
                c->boost = fminf(100.0f, c->boost + (PAD_IS_BIG(i) ? 100.0f : 12.0f));
                w->pad_t[i] = PAD_IS_BIG(i) ? 10.0f : 4.0f;
                break;
            }
        }
    }
}

/* un paso de simulacion; devuelve 1 si gol del coche 0 (ataca +Y), 2 si gol del coche 1 */
static int world_step(World *w, const Input *i0, const Input *i1, float dt, int live) {
    car_step(&w->car[0], i0, dt);
    car_step(&w->car[1], i1, dt);
    car_car(&w->car[0], &w->car[1]);
    ball_step(&w->ball, dt);
    ball_car(&w->ball, &w->car[0]);
    ball_car(&w->ball, &w->car[1]);
    pads_step(w, dt);
    if (live) {
        if (w->ball.p.y > ARENA_HY + BALL_R) return 1;
        if (w->ball.p.y < -ARENA_HY - BALL_R) return 2;
    }
    return 0;
}

#endif
