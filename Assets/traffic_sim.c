/*
 * 4-way intersection traffic simulation (raylib, C99)
 *
 *  - Cars spawn at runtime on all 4 roads with a random intention (straight / left / right)
 *  - One road is green at a time (N -> E -> S -> W). Each road's green duration comes from
 *    External_GetGreenTime(), which is re-queried for every road after each full cycle.
 *  - Yellow is fixed at 5 s. Cars that can still stop comfortably will stop, the rest go through.
 *  - Cars are heap allocated (linked list) and free()'d as soon as they leave the screen.
 *
 * Build (Linux):   gcc traffic_sim.c -o traffic_sim -lraylib -lm -lGL -lpthread -ldl -lrt -lX11
 * Build (macOS):   gcc traffic_sim.c -o traffic_sim -lraylib -framework OpenGL -framework Cocoa -framework IOKit
 * Build (Windows): gcc traffic_sim.c -o traffic_sim.exe -lraylib -lopengl32 -lgdi32 -lwinmm
 *
 * Controls: UP/DOWN = traffic density, SPACE = pause
 *
 * Traffic drives on the right. Screen coordinates: +x right, +y down.
 */

#include "raylib.h"
#include "raymath.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* ------------------------------------------------------------------------- */
/*  Tunables                                                                 */
/* ------------------------------------------------------------------------- */
#define WIN_W          900
#define WIN_H          900

#define LANE_W         50.0f            /* one lane per direction                 */
#define HALF_ROAD      LANE_W           /* half of the road width (= 1 lane)      */
#define STOP_OFFSET    10.0f            /* stop line distance from the box edge   */
#define SPAWN_DIST     480.0f           /* spawn / despawn distance from centre   */

#define YELLOW_TIME    5.0f             /* seconds, as requested                  */
#define ALL_RED_TIME   2.0f             /* clearance after yellow (set 0 to drop) */
#define MIN_GREEN      8.0f             /* only used by the placeholder           */
#define MAX_GREEN      30.0f

#define ACCEL          140.0f           /* px/s^2                                 */
#define BRAKE_DECEL    260.0f           /* px/s^2                                 */
#define MIN_GAP        8.0f             /* standstill bumper gap (px)             */
#define TURN_SPEED     75.0f            /* max speed while turning (px/s)         */
#define LOOKAHEAD      160.0f

#define SPAWN_MIN_INT  3.0f             /* random spawn interval per road (s)     */
#define SPAWN_MAX_INT  7.0f

#define BEZ_N          16               /* segments used to approximate a turn    */
#define PATH_PTS       (BEZ_N + 3)

/* ------------------------------------------------------------------------- */
/*  Types                                                                    */
/* ------------------------------------------------------------------------- */
typedef enum { ROAD_NORTH = 0, ROAD_EAST, ROAD_SOUTH, ROAD_WEST, ROAD_COUNT } Road;
typedef enum { TURN_STRAIGHT = 0, TURN_LEFT, TURN_RIGHT, TURN_COUNT } Turn;
typedef enum { LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN } LightState;
typedef enum { STAGE_GREEN, STAGE_YELLOW, STAGE_ALL_RED } Stage;

/* Polyline path a car follows: spawn -> stop line -> through the box -> exit */
typedef struct {
    Vector2 pts[PATH_PTS];
    float   cum[PATH_PTS];   /* cumulative length at each point                    */
    float   total;
    float   sEntry;          /* s where the path enters the intersection box       */
    float   sExit;           /* s where the path leaves the box                    */
    float   sStop;           /* s of the stop line (measured at the front bumper)  */
    int     exitRoad;
} Path;

typedef struct Car {
    int         road;        /* road the car arrives from                          */
    Turn        turn;        /* decided at spawn                                   */
    const Path *path;
    float       s, v, vmax;  /* distance along path (centre), speed, preferred     */
    float       len, wid;
    Color       color;
    Vector2     pos, heading;
    int         yellowDecision;  /* 0 = undecided, 1 = stop, 2 = go                */
    struct Car *next;
} Car;

/* Data handed to the external controller so it can decide the green durations */
typedef struct {
    int   waiting[ROAD_COUNT];    /* cars that have not entered the box yet        */
    float lastGreen[ROAD_COUNT];  /* green time used in the previous cycle         */
    int   cycle;                  /* number of completed cycles                    */
} TrafficSnapshot;

typedef struct {
    int   active;                 /* road that currently owns the green/yellow     */
    Stage stage;
    float timer;                  /* seconds left in current stage                 */
    float greenTime[ROAD_COUNT];  /* current cycle's green durations               */
    int   cycle;
} Controller;

/* ------------------------------------------------------------------------- */
/*  >>>  EXTERNAL HOOK - replace the body of this function  <<<              */
/* ------------------------------------------------------------------------- */
/* Called once per road at start-up and again for every road after each full
 * cycle. Return how many seconds the given road should stay GREEN.
 * The yellow phase (5 s) is added automatically afterwards.
 *
 * PLACEHOLDER: green time grows with the queue length (5 s + 1 s per waiting car),
 * clamped to [MIN_GREEN, MAX_GREEN]. */
float External_GetGreenTime(int road, const TrafficSnapshot *snap)
{
    float t = 5.0f + 1.0f * (float)snap->waiting[road];
    if (t < MIN_GREEN) t = MIN_GREEN;
    if (t > MAX_GREEN) t = MAX_GREEN;
    return t;
}

/* ------------------------------------------------------------------------- */
/*  Globals                                                                  */
/* ------------------------------------------------------------------------- */
static const Vector2 CENTER_PT = { WIN_W / 2.0f, WIN_H / 2.0f };
static const Vector2 DIR_OUT[ROAD_COUNT] = { {0, -1}, {1, 0}, {0, 1}, {-1, 0} }; /* centre -> road */
static const char   *ROAD_NAME[ROAD_COUNT] = { "North", "East", "South", "West" };

static Path gPaths[ROAD_COUNT][TURN_COUNT];
static Car *gCars     = NULL;
static int  gCarCount = 0;

/* ------------------------------------------------------------------------- */
/*  Small helpers                                                            */
/* ------------------------------------------------------------------------- */
static float RandF(float a, float b) { return a + (b - a) * ((float)GetRandomValue(0, 10000) / 10000.0f); }

/* right-hand side of a heading, in y-down screen coordinates */
static Vector2 RightOf(Vector2 h) { return (Vector2){ -h.y, h.x }; }

/* o + a*da + b*db */
static Vector2 Pt(Vector2 o, Vector2 a, float da, Vector2 b, float db)
{
    return (Vector2){ o.x + a.x * da + b.x * db, o.y + a.y * da + b.y * db };
}

/* ------------------------------------------------------------------------- */
/*  Path construction                                                        */
/* ------------------------------------------------------------------------- */
static void BuildPaths(void)
{
    const float half = LANE_W * 0.5f;

    for (int d = 0; d < ROAD_COUNT; d++) {
        Vector2 u   = DIR_OUT[d];                 /* from centre towards the arrival road */
        Vector2 hin = Vector2Scale(u, -1.0f);     /* heading of an arriving car           */
        Vector2 rin = RightOf(hin);

        for (int t = 0; t < TURN_COUNT; t++) {
            Vector2 want = (t == TURN_STRAIGHT) ? hin
                         : (t == TURN_RIGHT)    ? rin
                                                : Vector2Scale(rin, -1.0f);
            int e = 0;
            for (int k = 0; k < ROAD_COUNT; k++)
                if (Vector2DotProduct(DIR_OUT[k], want) > 0.5f) e = k;

            Vector2 hout = DIR_OUT[e];
            Vector2 rout = RightOf(hout);

            Vector2 p0   = Pt(CENTER_PT, u,    SPAWN_DIST, rin,  half);
            Vector2 p1   = Pt(CENTER_PT, u,    HALF_ROAD,  rin,  half);   /* box entry  */
            Vector2 p2   = Pt(CENTER_PT, hout, HALF_ROAD,  rout, half);   /* box exit   */
            Vector2 pEnd = Pt(CENTER_PT, hout, SPAWN_DIST, rout, half);

            Vector2 k;
            if (t == TURN_STRAIGHT) {
                k = Vector2Lerp(p1, p2, 0.5f);
            } else {
                float along = Vector2DotProduct(Vector2Subtract(p2, p1), hin);
                k = Pt(p1, hin, along, hin, 0.0f);      /* corner where lane lines meet */
            }

            Path *P = &gPaths[d][t];
            P->exitRoad = e;
            P->pts[0] = p0;
            for (int i = 0; i <= BEZ_N; i++) {
                float tt = (float)i / BEZ_N, om = 1.0f - tt;
                P->pts[1 + i].x = om * om * p1.x + 2 * om * tt * k.x + tt * tt * p2.x;
                P->pts[1 + i].y = om * om * p1.y + 2 * om * tt * k.y + tt * tt * p2.y;
            }
            P->pts[PATH_PTS - 1] = pEnd;

            P->cum[0] = 0.0f;
            for (int i = 1; i < PATH_PTS; i++)
                P->cum[i] = P->cum[i - 1] + Vector2Distance(P->pts[i - 1], P->pts[i]);

            P->total  = P->cum[PATH_PTS - 1];
            P->sEntry = P->cum[1];
            P->sExit  = P->cum[BEZ_N + 1];
            P->sStop  = P->sEntry - STOP_OFFSET;
        }
    }
}

static Vector2 PosAt(const Path *p, float s)
{
    if (s <= 0.0f)      return p->pts[0];
    if (s >= p->total)  return p->pts[PATH_PTS - 1];
    int i = 0;
    while (i < PATH_PTS - 2 && s > p->cum[i + 1]) i++;
    float seg = p->cum[i + 1] - p->cum[i];
    float t   = (seg > 0.0f) ? (s - p->cum[i]) / seg : 0.0f;
    return Vector2Lerp(p->pts[i], p->pts[i + 1], t);
}

static void CarPose(const Path *p, float s, Vector2 *pos, Vector2 *heading)
{
    *pos = PosAt(p, s);
    float sc = fminf(fmaxf(s, 4.0f), p->total - 4.0f);   /* chord keeps the heading smooth */
    *heading = Vector2Normalize(Vector2Subtract(PosAt(p, sc + 4.0f), PosAt(p, sc - 4.0f)));
}

/* ------------------------------------------------------------------------- */
/*  Signal controller                                                        */
/* ------------------------------------------------------------------------- */
static LightState SignalFor(const Controller *ctl, int road)
{
    if (road != ctl->active) return LIGHT_RED;
    switch (ctl->stage) {
        case STAGE_GREEN:  return LIGHT_GREEN;
        case STAGE_YELLOW: return LIGHT_YELLOW;
        default:           return LIGHT_RED;
    }
}

static void RefreshGreenTimes(Controller *ctl)
{
    TrafficSnapshot snap = { 0 };
    snap.cycle = ctl->cycle;
    for (const Car *c = gCars; c; c = c->next)
        if (c->s + 0.5f * c->len < c->path->sEntry) snap.waiting[c->road]++;
    for (int r = 0; r < ROAD_COUNT; r++) snap.lastGreen[r] = ctl->greenTime[r];

    for (int r = 0; r < ROAD_COUNT; r++) {
        float t = External_GetGreenTime(r, &snap);
        ctl->greenTime[r] = (t > 0.0f) ? t : 0.0f;       /* guard against NaN / negatives */
    }
}

static void UpdateController(Controller *ctl, float dt)
{
    ctl->timer -= dt;
    while (ctl->timer <= 0.0f) {
        switch (ctl->stage) {
            case STAGE_GREEN:
                ctl->stage = STAGE_YELLOW;
                ctl->timer += YELLOW_TIME;
                break;
            case STAGE_YELLOW:
                ctl->stage = STAGE_ALL_RED;
                ctl->timer += ALL_RED_TIME;
                break;
            case STAGE_ALL_RED:
                ctl->active = (ctl->active + 1) % ROAD_COUNT;
                if (ctl->active == 0) {                  /* a full cycle just finished */
                    ctl->cycle++;
                    RefreshGreenTimes(ctl);              /* ask the external function  */
                }
                ctl->stage = STAGE_GREEN;
                ctl->timer += ctl->greenTime[ctl->active];
                break;
        }
    }
}

/* ------------------------------------------------------------------------- */
/*  Cars                                                                     */
/* ------------------------------------------------------------------------- */
static const Color CAR_COLORS[] = {
    { 220,  60,  60, 255 }, {  60, 120, 220, 255 }, { 240, 200,  50, 255 }, {  80, 200, 120, 255 },
    { 230, 230, 235, 255 }, { 160,  90, 210, 255 }, { 240, 140,  50, 255 }, {  70, 200, 210, 255 },
};

static void SpawnCar(int road)
{
    /* keep a clear gap behind the last spawned car on this road */
    for (const Car *o = gCars; o; o = o->next)
        if (o->road == road && o->s < 90.0f) return;

    Car *c = (Car *)calloc(1, sizeof(Car));
    if (!c) return;

    int r = GetRandomValue(0, 99);
    c->turn  = (r < 50) ? TURN_STRAIGHT : (r < 75) ? TURN_LEFT : TURN_RIGHT;
    c->road  = road;
    c->path  = &gPaths[road][c->turn];
    c->len   = RandF(32.0f, 44.0f);
    c->wid   = 20.0f;
    c->vmax  = RandF(100.0f, 150.0f);
    c->v     = 0.5f * c->vmax;
    c->s     = 0.0f;
    c->color = CAR_COLORS[GetRandomValue(0, (int)(sizeof(CAR_COLORS) / sizeof(CAR_COLORS[0])) - 1)];
    CarPose(c->path, c->s, &c->pos, &c->heading);

    c->next = gCars;
    gCars   = c;
    gCarCount++;
}

/* Does the car have to stop at its stop line right now? */
static int MustStop(Car *c, const Controller *ctl, float gapStop)
{
    switch (SignalFor(ctl, c->road)) {
        case LIGHT_GREEN:
            c->yellowDecision = 0;
            return 0;
        case LIGHT_YELLOW:
            if (c->yellowDecision == 0) {                 /* decide once, when yellow starts */
                float brakeDist = c->v * c->v / (2.0f * BRAKE_DECEL);
                c->yellowDecision = (gapStop + 1.0f >= brakeDist) ? 1 : 2;
            }
            return c->yellowDecision == 1;
        default: /* red */
            return c->yellowDecision != 2;                /* "go" cars committed on yellow */
    }
}

/* Nearest obstacle ahead. Returns 1 and fills gap / speed if there is one. */
static int FindLeader(const Car *c, float *gapOut, float *vOut)
{
    float best = 1e9f, vLead = 0.0f;
    int found = 0;

    for (const Car *o = gCars; o; o = o->next) {
        if (o == c) continue;
        float gap;

        if (o->path == c->path || (o->road == c->road && c->s <= c->path->sEntry)) {
            /* same lane / same path -> compare distance along the path */
            if (o->s <= c->s) continue;
            gap = (o->s - c->s) - 0.5f * (c->len + o->len);
        } else {
            /* anything else (merging into an exit lane, leftovers in the box) -> geometric check */
            Vector2 d  = Vector2Subtract(o->pos, c->pos);
            float fwd  = Vector2DotProduct(d, c->heading);
            if (fwd <= 0.0f || fwd > LOOKAHEAD) continue;
            float lat  = fabsf(d.x * c->heading.y - d.y * c->heading.x);
            if (lat > LANE_W * 0.4f) continue;
            gap = fwd - 0.5f * (c->len + o->len);
        }
        if (gap < best) { best = gap; vLead = o->v; found = 1; }
    }
    *gapOut = best;
    *vOut   = vLead;
    return found;
}

static void UpdateCar(Car *c, float dt, const Controller *ctl)
{
    const Path *p = c->path;
    float vTarget = c->vmax;

    /* slow down for turns */
    if (c->turn != TURN_STRAIGHT && c->s > p->sEntry - 60.0f && c->s < p->sExit + c->len)
        vTarget = fminf(vTarget, TURN_SPEED);

    /* traffic light */
    float gapStop = p->sStop - (c->s + 0.5f * c->len);
    if (gapStop > -2.0f && MustStop(c, ctl, gapStop))
        vTarget = fminf(vTarget, sqrtf(2.0f * BRAKE_DECEL * fmaxf(gapStop, 0.0f)));

    /* car in front */
    float gap, vLead;
    if (FindLeader(c, &gap, &vLead)) {
        float g = fmaxf(gap - MIN_GAP, 0.0f);
        vTarget = fminf(vTarget, sqrtf(vLead * vLead + 2.0f * BRAKE_DECEL * g));
    }

    if (c->v < vTarget) c->v = fminf(vTarget, c->v + ACCEL * dt);
    else                c->v = fmaxf(vTarget, c->v - 1.5f * BRAKE_DECEL * dt);

    c->s += c->v * dt;
    CarPose(p, c->s, &c->pos, &c->heading);
}

static int OffScreen(Vector2 p, float m)
{
    return p.x < -m || p.x > WIN_W + m || p.y < -m || p.y > WIN_H + m;
}

/* Free every car that has left the observation space */
static void UpdateCars(float dt, const Controller *ctl)
{
    Car **pp = &gCars;
    while (*pp) {
        Car *c = *pp;
        UpdateCar(c, dt, ctl);

        int gone = c->s >= c->path->total ||
                   (c->s > c->path->sExit && OffScreen(c->pos, 24.0f));
        if (gone) {
            *pp = c->next;
            free(c);
            gCarCount--;
        } else {
            pp = &c->next;
        }
    }
}

static void FreeAllCars(void)
{
    while (gCars) {
        Car *n = gCars->next;
        free(gCars);
        gCars = n;
    }
    gCarCount = 0;
}

/* ------------------------------------------------------------------------- */
/*  Drawing                                                                  */
/* ------------------------------------------------------------------------- */
static void DrawRoads(void)
{
    const Color grass   = { 34, 85, 45, 255 };
    const Color asphalt = { 55, 58, 64, 255 };
    ClearBackground(grass);

    DrawRectangle((int)(CENTER_PT.x - HALF_ROAD), 0, (int)(2 * HALF_ROAD), WIN_H, asphalt);
    DrawRectangle(0, (int)(CENTER_PT.y - HALF_ROAD), WIN_W, (int)(2 * HALF_ROAD), asphalt);

    for (int d = 0; d < ROAD_COUNT; d++) {
        Vector2 u = DIR_OUT[d], r = RightOf(u);
        Vector2 a = Pt(CENTER_PT, u, HALF_ROAD, r, 0), b = Pt(CENTER_PT, u, 700.0f, r, 0);
        DrawLineEx(Pt(a, r, 2, r, 0), Pt(b, r, 2, r, 0), 2.0f, GOLD);        /* double yellow */
        DrawLineEx(Pt(a, r, -2, r, 0), Pt(b, r, -2, r, 0), 2.0f, GOLD);

        /* stop line across the arriving lane */
        Vector2 rin  = RightOf(Vector2Scale(u, -1.0f));
        Vector2 base = Pt(CENTER_PT, u, HALF_ROAD + STOP_OFFSET, rin, 0);
        DrawLineEx(base, Pt(base, rin, LANE_W, rin, 0), 4.0f, WHITE);
    }
}

static void DrawCar(const Car *c)
{
    float ang = atan2f(c->heading.y, c->heading.x) * RAD2DEG;

    Rectangle body = { c->pos.x, c->pos.y, c->len, c->wid };
    DrawRectanglePro(body, (Vector2){ c->len / 2, c->wid / 2 }, ang, c->color);

    Vector2 wp = Vector2Add(c->pos, Vector2Scale(c->heading, c->len * 0.18f));
    Rectangle ws = { wp.x, wp.y, c->len * 0.22f, c->wid * 0.7f };
    DrawRectanglePro(ws, (Vector2){ ws.width / 2, ws.height / 2 }, ang, (Color){ 30, 40, 55, 255 });

    /* blinkers while the car still has to turn */
    if (c->turn != TURN_STRAIGHT && c->s < c->path->sExit && fmodf((float)GetTime(), 0.6f) < 0.3f) {
        float side = (c->turn == TURN_RIGHT) ? 1.0f : -1.0f;
        Vector2 r  = RightOf(c->heading);
        for (int e = -1; e <= 1; e += 2) {
            Vector2 q = Pt(c->pos, c->heading, e * (c->len * 0.5f - 2.0f), r, side * (c->wid * 0.5f - 1.0f));
            DrawCircleV(q, 2.5f, ORANGE);
        }
    }
}

static Vector2 SignalPos(int d)
{
    Vector2 u   = DIR_OUT[d];
    Vector2 rin = RightOf(Vector2Scale(u, -1.0f));
    return Pt(CENTER_PT, u, 100.0f, rin, 88.0f);
}

static void DrawSignal(int d, const Controller *ctl)
{
    Vector2 c = SignalPos(d);
    LightState st = SignalFor(ctl, d);

    DrawRectangleRounded((Rectangle){ c.x - 12, c.y - 32, 24, 64 }, 0.3f, 6, (Color){ 20, 20, 24, 255 });
    DrawCircleV((Vector2){ c.x, c.y - 20 }, 8, st == LIGHT_RED    ? (Color){ 235,  50,  50, 255 } : (Color){ 70, 25, 25, 255 });
    DrawCircleV((Vector2){ c.x, c.y      }, 8, st == LIGHT_YELLOW ? (Color){ 250, 200,  40, 255 } : (Color){ 70, 60, 20, 255 });
    DrawCircleV((Vector2){ c.x, c.y + 20 }, 8, st == LIGHT_GREEN  ? (Color){  50, 220,  90, 255 } : (Color){ 20, 60, 30, 255 });

    const char *txt = (d == ctl->active && ctl->stage != STAGE_ALL_RED)
        ? TextFormat("%s %ds", ROAD_NAME[d], (int)ceilf(ctl->timer))
        : ROAD_NAME[d];
    int w = MeasureText(txt, 18);
    int x = (c.x < CENTER_PT.x) ? (int)c.x - 18 - w : (int)c.x + 18;
    DrawText(txt, x, (int)c.y - 9, 18, RAYWHITE);
}

static void DrawHud(const Controller *ctl, float density, int paused)
{
    DrawRectangle(8, 8, 450, 88, Fade(BLACK, 0.6f));
    const char *stage = ctl->stage == STAGE_GREEN ? "GREEN" : ctl->stage == STAGE_YELLOW ? "YELLOW" : "ALL RED";
    DrawText(TextFormat("Cycle %d | %s: %s %.1fs", ctl->cycle + 1, ROAD_NAME[ctl->active], stage, ctl->timer),
             16, 14, 18, RAYWHITE);
    DrawText(TextFormat("Green: N %.0fs  E %.0fs  S %.0fs  W %.0fs",
             ctl->greenTime[0], ctl->greenTime[1], ctl->greenTime[2], ctl->greenTime[3]), 16, 38, 18, LIGHTGRAY);
    DrawText(TextFormat("Cars alive: %d | Density x%.2f [UP/DOWN] | [SPACE] %s",
             gCarCount, density, paused ? "resume" : "pause"), 16, 62, 16, LIGHTGRAY);
}

/* ------------------------------------------------------------------------- */
/*  Main                                                                     */
/* ------------------------------------------------------------------------- */
int main(void)
{
    InitWindow(WIN_W, WIN_H, "4-way intersection simulation");
    SetTargetFPS(60);
    SetRandomSeed((unsigned int)time(NULL));

    BuildPaths();

    Controller ctl = { 0 };
    RefreshGreenTimes(&ctl);                         /* first set of green durations */
    ctl.active = ROAD_NORTH;
    ctl.stage  = STAGE_GREEN;
    ctl.timer  = ctl.greenTime[ROAD_NORTH];

    float spawnTimer[ROAD_COUNT];
    for (int r = 0; r < ROAD_COUNT; r++) spawnTimer[r] = RandF(0.2f, 3.0f);

    float density = 1.0f;
    int   paused  = 0;

    while (!WindowShouldClose()) {
        float dt = fminf(GetFrameTime(), 0.05f);

        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_UP))    density = fminf(density * 1.25f, 4.0f);
        if (IsKeyPressed(KEY_DOWN))  density = fmaxf(density * 0.80f, 0.25f);

        if (!paused) {
            UpdateController(&ctl, dt);

            for (int r = 0; r < ROAD_COUNT; r++) {
                spawnTimer[r] -= dt;
                if (spawnTimer[r] <= 0.0f) {
                    SpawnCar(r);
                    spawnTimer[r] = RandF(SPAWN_MIN_INT, SPAWN_MAX_INT) / density;
                }
            }
            UpdateCars(dt, &ctl);
        }

        BeginDrawing();
            DrawRoads();
            for (const Car *c = gCars; c; c = c->next) DrawCar(c);
            for (int d = 0; d < ROAD_COUNT; d++) DrawSignal(d, &ctl);
            DrawHud(&ctl, density, paused);
        EndDrawing();
    }

    FreeAllCars();
    CloseWindow();
    return 0;
}
