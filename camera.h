/*
 * camera.h - camara estilo Rocket League (brazo de resorte, Car Cam / Ball Cam,
 * cuaterniones y colision por trazado de esfera contra la arena).
 *
 * Parametros (como en el menu de camara de Rocket League):
 *   distance   : distancia del brazo detras del coche (uu)
 *   height     : desplazamiento vertical local de la camara (uu)
 *   angle      : cabeceo fijo en grados (negativo = mira hacia abajo)
 *   stiffness  : 0..1, rigidez del muelle (mayor = menos retraso)
 *   swivel     : velocidad con la que la camara sigue la rotacion objetivo
 *   transition : velocidad del Slerp entre Car Cam y Ball Cam
 */
#ifndef CAMERA_H
#define CAMERA_H
#include "rlphys.h"

typedef struct { float x, y, z, w; } Quat;

static Quat qnorm(Quat q) {
    float l = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l < 1e-8f) { Quat i = {0, 0, 0, 1}; return i; }
    Quat r = {q.x / l, q.y / l, q.z / l, q.w / l};
    return r;
}

/* cuaternion desde una base ortonormal (X, Y, Z son las columnas de la matriz) */
static Quat quat_from_basis(V3 X, V3 Y, V3 Z) {
    float m00 = X.x, m10 = X.y, m20 = X.z, m01 = Y.x, m11 = Y.y, m21 = Y.z, m02 = Z.x, m12 = Z.y, m22 = Z.z;
    float tr = m00 + m11 + m22;
    Quat q;
    if (tr > 0) {
        float s = sqrtf(tr + 1.0f) * 2.0f;
        q.w = 0.25f * s; q.x = (m21 - m12) / s; q.y = (m02 - m20) / s; q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        q.w = (m21 - m12) / s; q.x = 0.25f * s; q.y = (m01 + m10) / s; q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        q.w = (m02 - m20) / s; q.x = (m01 + m10) / s; q.y = 0.25f * s; q.z = (m12 + m21) / s;
    } else {
        float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        q.w = (m10 - m01) / s; q.x = (m02 + m20) / s; q.y = (m12 + m21) / s; q.z = 0.25f * s;
    }
    return qnorm(q);
}

static V3 qrot(Quat q, V3 v) {
    V3 u = v3(q.x, q.y, q.z);
    V3 t = vmul(vcross(u, v), 2.0f);
    return vadd(vadd(v, vmul(t, q.w)), vcross(u, t));
}

static Quat qslerp(Quat a, Quat b, float t) {
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0) { b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w; d = -d; }
    Quat r;
    if (d > 0.9995f) {
        r.x = lerpf(a.x, b.x, t); r.y = lerpf(a.y, b.y, t); r.z = lerpf(a.z, b.z, t); r.w = lerpf(a.w, b.w, t);
        return qnorm(r);
    }
    float th = acosf(d), s = sinf(th), wa = sinf((1.0f - t) * th) / s, wb = sinf(t * th) / s;
    r.x = a.x * wa + b.x * wb; r.y = a.y * wa + b.y * wb; r.z = a.z * wa + b.z * wb; r.w = a.w * wa + b.w * wb;
    return qnorm(r);
}

/* LookAt: mira hacia fwd con el "arriba" del mundo (sin alabeo) */
static Quat quat_look(V3 fwd) {
    V3 f = vnorm(fwd), up = v3(0, 0, 1);
    V3 left = vcross(up, f);
    if (vlen(left) < 1e-3f) {                 /* mirada casi vertical: eje alternativo */
        left = vcross(v3(1, 0, 0), f);
        if (vlen(left) < 1e-3f) left = v3(0, 1, 0);
    }
    left = vnorm(left);
    return quat_from_basis(f, left, vcross(f, left));
}

typedef struct {
    float distance, height, angle, stiffness, swivel, transition;   /* ajustes */
    Quat q;                 /* rotacion actual de la camara */
    V3 pos, vel;            /* posicion y velocidad del muelle (mundo) */
    float blend;            /* 0 = Car Cam, 1 = Ball Cam */
    float frac;             /* fraccion del brazo tras colisionar (1 = libre) */
    V3 last_flat;           /* ultimo "adelante" horizontal valido del coche */
    V3 last_boom;           /* horizontal suavizada del brazo (Ball Cam) */
    float yaw;              /* guinada estable de la Ball Cam */
    V3 out_pos, out_fwd;    /* resultado: ojo y direccion de mirada */
    int ready;
} Cam;

static void cam_init(Cam *c) {
    memset(c, 0, sizeof(*c));
    c->distance = 270.0f; c->height = 100.0f; c->angle = -4.0f;
    c->stiffness = 0.45f; c->swivel = 4.5f; c->transition = 1.2f;
    c->last_flat = v3(0, 1, 0);
    c->last_boom = v3(0, 1, 0);
    c->yaw = PI_F * 0.5f;
    c->frac = 1.0f;
}

/* Sphere trace desde 'a' hacia 'b': devuelve la fraccion 0..1 donde la esfera toca la arena */
static float cam_trace(V3 a, V3 b, float radius) {
    V3 d = vsub(b, a);
    float len = vlen(d);
    if (len < 1.0f) return 1.0f;
    int n = (int)(len / 20.0f) + 2;
    float lo = 0.0f, hi = -1.0f;
    for (int i = 1; i <= n; i++) {
        float t = (float)i / n;
        float dd; V3 nn;
        arena(vadd(a, vmul(d, t)), &dd, &nn);
        if (dd < radius) { hi = t; break; }
        lo = t;
    }
    if (hi < 0) return 1.0f;
    for (int i = 0; i < 6; i++) {                     /* refinar el punto de impacto */
        float m = 0.5f * (lo + hi), dd; V3 nn;
        arena(vadd(a, vmul(d, m)), &dd, &nn);
        if (dd < radius) hi = m; else lo = m;
    }
    return lo;
}

static void cam_update(Cam *c, const Car *car, const Ball *ball, int ballcam, float dt, int snap) {
    /* ---- Car Cam: adelante horizontal del coche (ignora roll y alabeo) + pitch fijo ---- */
    V3 fh = v3(car->f.x, car->f.y, 0);
    if (vlen(fh) > 0.25f) c->last_flat = vnorm(fh);
    float a = c->angle * PI_F / 180.0f;
    V3 dcar = vadd(vmul(c->last_flat, cosf(a)), v3(0, 0, sinf(a)));
    Quat qcar = quat_look(dcar);

    /* ---- Ball Cam: direccion coche -> pelota en esfericas.
       La guinada se congela cuando la pelota esta casi en la vertical (ahi el
       vector horizontal ~0 y atan2 saltaria 180 grados). La elevacion se limita
       a +-85: la pelota sigue en pantalla y se evita la singularidad de la base
       (gimbal lock) al mirar justo hacia arriba. ---- */
    V3 d = vsub(ball->p, car->p);
    float dl = vlen(d);
    V3 n = (dl > 60.0f) ? vmul(d, 1.0f / dl) : dcar;
    float hlen = sqrtf(n.x * n.x + n.y * n.y);
    float ty = c->yaw;
    if (hlen > 0.15f) ty = atan2f(n.y, n.x);          /* > ~81 grados: mantener guinada */
    if (snap || !c->ready) {
        c->yaw = (hlen > 0.15f) ? ty : atan2f(c->last_flat.y, c->last_flat.x);
    } else {
        float dy = atan2f(sinf(ty - c->yaw), cosf(ty - c->yaw));   /* giro mas corto */
        c->yaw += dy * (1.0f - expf(-c->swivel * 2.0f * dt));
        c->yaw = atan2f(sinf(c->yaw), cosf(c->yaw));
    }
    const float ELEV_LIM = 85.0f * PI_F / 180.0f;
    /* atan2(z, |xy|) es exacto tambien con la pelota en la vertical (asinf pierde
       precision al acercarse a +-1); la suma con 'a' mantiene el encuadre. */
    float elev = atan2f(n.z, hlen) + a;
    elev = clampf(elev, -ELEV_LIM, ELEV_LIM);
    V3 dball = v3(cosf(elev) * cosf(c->yaw), cosf(elev) * sinf(c->yaw), sinf(elev));
    Quat qball = quat_look(dball);

    /* ---- Transition Speed: Slerp entre Car Cam y Ball Cam ---- */
    float goal = ballcam ? 1.0f : 0.0f;
    float step = c->transition * 3.0f * dt;
    if (c->blend < goal) c->blend = fminf(goal, c->blend + step);
    else c->blend = fmaxf(goal, c->blend - step);
    float al = c->blend * c->blend * (3.0f - 2.0f * c->blend);
    Quat qt = qslerp(qcar, qball, al);

    /* ---- Swivel: la rotacion sigue al objetivo ---- */
    if (snap || !c->ready) c->q = qt;
    else c->q = qslerp(c->q, qt, 1.0f - expf(-c->swivel * 1.4f * dt));

    /* ---- Brazo de resorte: P = Pcoche - Forward*Distance + Up*Height ----
       En Ball Cam se usa la componente horizontal del brazo: con la pelota en
       la vertical la camara no se mete debajo del coche. last_boom se interpola
       para que el cambio de lado de la pelota no provoque un giro brusco. ---- */
    V3 fwd = qrot(c->q, v3(1, 0, 0)), up = qrot(c->q, v3(0, 0, 1));
    V3 hf = v3(fwd.x, fwd.y, 0);
    float hl = vlen(hf);
    V3 desired = (hl > 0.12f) ? vmul(hf, 1.0f / hl) : c->last_boom;
    if (snap || !c->ready) {
        c->last_boom = (hl > 0.12f) ? vmul(hf, 1.0f / hl) : c->last_flat;
    } else {
        float k = 1.0f - expf(-6.0f * dt);
        V3 nb = vadd(vmul(c->last_boom, 1.0f - k), vmul(desired, k));
        float nl = vlen(nb);
        if (nl > 1e-3f) c->last_boom = vmul(nb, 1.0f / nl);
    }
    V3 bb = vadd(vmul(fwd, 1.0f - al), vmul(c->last_boom, al));
    if (vlen(bb) < 1e-3f) bb = c->last_boom;
    V3 boom = vnorm(bb);
    V3 target = vadd(vsub(car->p, vmul(boom, c->distance)), vmul(up, c->height));
    if (snap || !c->ready) { c->pos = target; c->vel = v3(0, 0, 0); c->frac = 1.0f; }
    else {
        float w = lerpf(8.0f, 60.0f, c->stiffness);              /* constante elastica k = w^2 */
        V3 e = vsub(c->pos, target);
        V3 acc = vsub(vmul(e, -w * w), vmul(c->vel, 2.0f * w));  /* muelle critico amortiguado */
        c->vel = vadd(c->vel, vmul(acc, dt));
        c->pos = vadd(c->pos, vmul(c->vel, dt));
        e = vsub(c->pos, target);
        float el = vlen(e);
        if (el > 140.0f) c->pos = vadd(target, vmul(e, 140.0f / el));   /* limite del estiramiento */
    }
    c->ready = 1;

    /* ---- Colision: sphere trace desde el coche hasta la camara ---- */
    float fr = cam_trace(car->p, c->pos, 14.0f);
    if (fr < c->frac) c->frac = fr;                                       /* nunca pierde la vista */
    else c->frac = fminf(1.0f, c->frac + (fr - c->frac) * (1.0f - expf(-7.0f * dt)));   /* recupera suave */
    V3 pos = vadd(car->p, vmul(vsub(c->pos, car->p), c->frac));
    float dd; V3 nn;
    arena(pos, &dd, &nn);
    if (dd < 40.0f) pos = vadd(pos, vmul(nn, 40.0f - dd));

    c->out_pos = pos;
    c->out_fwd = fwd;
}

#endif
