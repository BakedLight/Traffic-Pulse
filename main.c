// 4-way intersection traffic simulation (left-hand traffic)
// Needs raylib and raygui.h (keep raygui.h in the same folder as this file)
// Build (Linux): gcc traffic_sim.c -o sim -lraylib -lm -lGL -lpthread -ldl -lrt -lX11

#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include <math.h>
#include <time.h>

// ------------------------------------------------------------
// Settings
// ------------------------------------------------------------

#define e 2.718281

#define SCREEN      900     // window is SCREEN x SCREEN pixels
#define CENTER      450     // centre of the intersection (x and y)
#define ROAD_HALF   30      // half of the road width = edge of the intersection box
#define LANE_WIDTH  30      // one lane for each direction
#define STOP_GAP    10      // stop line is this far before the intersection box
#define SPAWN_DIST  480     // cars appear / disappear this far from the centre (off screen)

#define YELLOW_TIME   2.0f  // seconds of yellow light
#define ALL_RED_TIME  0.0f  // short all-red pause so the box can clear
#define MIN_GREEN     3.0f  // limits used by the placeholder function only
#define MAX_GREEN     15.0f
#define CLAMP_GREEN   10.0f
#define TOTAL_TIME (YELLOW_TIME + ALL_RED_TIME + MAX_GREEN) * 4

#define ACCEL       80.0f  // car acceleration (pixels/s^2)
#define BRAKE       200.0f  // car braking (pixels/s^2)
#define MIN_GAP     8.0f    // gap kept between stopped cars
#define TURN_SPEED  75.0f   // max speed while turning
#define MEDIAN_VEL  75.0f
#define SPAWN_TIME  7.0f
#define AVG_LENGTH  25.0f
#define CAR_WIDTH   12.0f
#define LOOK_AHEAD  160.0f  // how far a car looks for other cars
#define NOT_AHEAD   1000000000.0f

#define CURVE_STEPS 16                  // a turn is drawn as a curve made of 16 small lines
#define PATH_POINTS (CURVE_STEPS + 3)   // spawn point + curve points + end point
#define MAX_CARS    120

// ------------------------------------------------------------
// Enums and structs
// ------------------------------------------------------------
typedef enum { ROAD_NORTH, ROAD_EAST, ROAD_SOUTH, ROAD_WEST, NUM_ROADS } Road;
typedef enum { GO_STRAIGHT, TURN_LEFT, TURN_RIGHT, NUM_TURNS } Turn;
typedef enum { LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN } Light;
typedef enum { STAGE_GREEN, STAGE_YELLOW, STAGE_ALL_RED } Stage;
typedef enum { YELLOW_UNDECIDED, YELLOW_STOP, YELLOW_GO } YellowChoice;

// The route a car follows: from spawn point, through the intersection, to the far end of the exit road
typedef struct
{
    Vector2 points[PATH_POINTS];
    float dist[PATH_POINTS];    // distance from the start up to each point
    float total;                // total length of the path
    float entryS;               // distance at which the path enters the intersection box
    float exitS;                // distance at which the path leaves the box
    float stopS;                // distance of the stop line
} Path;

typedef struct
{
    int active;                 // 1 = car exists, 0 = free slot
    int road;                   // road the car comes from
    int turn;                   // decided at spawn: straight / left / right
    float s;                    // distance travelled along the path (centre of the car)
    float speed;
    float maxSpeed;
    float length;
    float width;
    Color color;
    Vector2 pos;                // current position on screen
    Vector2 dir;                // direction the car is facing (length 1)
    YellowChoice yellowChoice;  // what the car decided when the light turned yellow
} Car;

// Information given to the external function
typedef struct
{
    int waiting[NUM_ROADS];         // cars on each road that have not entered the box yet
    float lastGreen[NUM_ROADS];     // green time each road got in the previous cycle
    int cycle;                      // number of completed cycles
} TrafficInfo;

typedef struct
{
    int activeRoad;                 // road that currently has green / yellow
    Stage stage;
    float timer;                    // seconds left in the current stage
    float greenTime[NUM_ROADS];     // green duration of every road for this cycle
    int cycle;
} Controller;

// ------------------------------------------------------------
// Global data
// ------------------------------------------------------------
int InstantDataSet[4][4] = {{0}, {0}, {0}, {0}};

int DynamicTiming = 1;

Path paths[NUM_ROADS][NUM_TURNS];
Car cars[MAX_CARS];
int carCount = 0;
Controller control;
float density[NUM_ROADS] = { 0.7, 0.7, 0.7, 0.7};     // set by the sliders, 1 = normal traffic
const char *roadNames[NUM_ROADS] = { "North", "East", "South", "West" };
Color carColors[6] = { {220,60,60,255}, {60,120,220,255}, {240,200,50,255},
                       {80,200,120,255}, {230,230,235,255}, {160,90,210,255} };

// ------------------------------------------------------------
// EXTERNAL FUNCTION (placeholder) - replace the inside of this function.
// Returns how many seconds 'road' should stay green. Yellow (5 s) is added automatically.
// It is called for every road at start and again after every full cycle.
// Placeholder rule: 5 seconds + 1 second per waiting car, kept between MIN_GREEN and MAX_GREEN.
// ------------------------------------------------------------
void calculate_times(const int *Side_units, int Total_Time, int Yellow_Timer, float *output) {
    float Total_Green_Time = (float)(Total_Time - 4 * Yellow_Timer);
    if (!DynamicTiming) {
        for (int i = 0; i < 4; i++) output[i] = Total_Green_Time / 4.0f;
        return;
    }
    int sum = 0;
    for (int i = 0; i < 4; i++) {
        sum += Side_units[i];
    }
    if (sum == 0) {
        for (int i = 0; i < 4; i++) output[i] = Total_Green_Time / 4.0f;
        return;
    }
    float weights[4];
    float weight_sum = 0.0f;
    for (int i = 0; i < 4; i++) {
        weights[i] = (Side_units[i] > 0) ? (1.0f - expf(-(float)Side_units[i] / sum)) : 0.0f;
        weight_sum += weights[i];
    }
    int locked[4] = {0, 0, 0, 0};
    float remaining_green = Total_Green_Time;
    while (1) {
        int clamped_any = 0;
        weight_sum = 0.0f;
        for (int i = 0; i < 4; i++) {
            if (!locked[i]) weight_sum += weights[i];
        }
        if (weight_sum <= 0.0f) break;
        for (int i = 0; i < 4; i++) {
            if (!locked[i]) {
                output[i] = remaining_green * (weights[i] / weight_sum);
                if (output[i] < MIN_GREEN) {
                    output[i] = MIN_GREEN;
                    locked[i] = 1;
                    remaining_green -= MIN_GREEN;
                    clamped_any = 1;
                    break;
                }
            }
        }
        if (!clamped_any) break;
    }
}

// ------------------------------------------------------------
// Helper functions
// ------------------------------------------------------------
float RandomFloat(float min, float max)
{
    return min + (max - min) * GetRandomValue(0, 10000) / 10000.0f;
}

// Everything is first described for the NORTH road, then turned by 90 degrees
// (once for East, twice for South, three times for West).
// dx, dy = position relative to the centre, as seen for the North road.
Vector2 RotatePoint(float dx, float dy, int times)
{
    int i;
    float temp;
    Vector2 p;

    for (i = 0; i < times; i++)
    {
        temp = dx;
        dx = -dy;
        dy = temp;
    }
    p.x = CENTER + dx;
    p.y = CENTER + dy;
    return p;
}

// ------------------------------------------------------------
// Building the paths
// ------------------------------------------------------------
void BuildPaths(void)
{
    int road, turn, i;
    float lane = LANE_WIDTH / 2.0f;     // cars drive in the middle of their lane

    for (road = 0; road < NUM_ROADS; road++)
    {
        for (turn = 0; turn < NUM_TURNS; turn++)
        {
            // Key points for a car coming from the North road (driving down the screen).
            // In left-hand traffic it uses the lane on the left of its driving direction.
            float cornerX, cornerY, exitX, exitY, endX, endY;

            if (turn == GO_STRAIGHT)
            {
                cornerX = lane;  cornerY = 0;
                exitX = lane;    exitY = ROAD_HALF;
                endX = lane;     endY = SPAWN_DIST;
            }
            else if (turn == TURN_LEFT)         // short turn
            {
                cornerX = lane;  cornerY = -lane;
                exitX = ROAD_HALF;  exitY = -lane;
                endX = SPAWN_DIST;  endY = -lane;
            }
            else                                // long turn across the box
            {
                cornerX = lane;  cornerY = lane;
                exitX = -ROAD_HALF; exitY = lane;
                endX = -SPAWN_DIST; endY = lane;
            }

            Path p;
            Vector2 entry  = RotatePoint(lane, -ROAD_HALF, road);
            Vector2 corner = RotatePoint(cornerX, cornerY, road);
            Vector2 exitP  = RotatePoint(exitX, exitY, road);

            p.points[0] = RotatePoint(lane, -SPAWN_DIST, road);     // spawn point

            // The way through the box is a curve from entry to exit that bends towards 'corner'
            for (i = 0; i <= CURVE_STEPS; i++)
            {
                float t = (float)i / CURVE_STEPS;
                float m = 1 - t;
                p.points[1 + i].x = m * m * entry.x + 2 * m * t * corner.x + t * t * exitP.x;
                p.points[1 + i].y = m * m * entry.y + 2 * m * t * corner.y + t * t * exitP.y;
            }

            p.points[PATH_POINTS - 1] = RotatePoint(endX, endY, road);     // far end of exit road

            // distance from the start to every point
            p.dist[0] = 0;
            for (i = 1; i < PATH_POINTS; i++)
            {
                float dx = p.points[i].x - p.points[i - 1].x;
                float dy = p.points[i].y - p.points[i - 1].y;
                p.dist[i] = p.dist[i - 1] + sqrtf(dx * dx + dy * dy);
            }

            p.total  = p.dist[PATH_POINTS - 1];
            p.entryS = p.dist[1];
            p.exitS  = p.dist[CURVE_STEPS + 1];
            p.stopS  = p.entryS - STOP_GAP;

            paths[road][turn] = p;
        }
    }
}

// Position on the path after travelling distance s
Vector2 PositionOnPath(int road, int turn, float s)
{
    Path p = paths[road][turn];
    Vector2 result;
    int i = 0;
    float t;

    if (s <= 0) return p.points[0];
    if (s >= p.total) return p.points[PATH_POINTS - 1];

    while (s > p.dist[i + 1]) i++;      // find which small line we are on
    t = (s - p.dist[i]) / (p.dist[i + 1] - p.dist[i]);

    result.x = p.points[i].x + (p.points[i + 1].x - p.points[i].x) * t;
    result.y = p.points[i].y + (p.points[i + 1].y - p.points[i].y) * t;
    return result;
}

// Sets the position and facing direction of car i from its distance s
void UpdateCarPosition(int i)
{
    int road = cars[i].road;
    int turn = cars[i].turn;
    float total = paths[road][turn].total;
    float s = cars[i].s;
    Vector2 behind, ahead;
    float dx, dy, len;

    cars[i].pos = PositionOnPath(road, turn, s);

    // direction = from a point 4 pixels behind to a point 4 pixels ahead (keeps turns smooth)
    if (s < 4) s = 4;
    if (s > total - 4) s = total - 4;
    behind = PositionOnPath(road, turn, s - 4);
    ahead  = PositionOnPath(road, turn, s + 4);
    dx = ahead.x - behind.x;
    dy = ahead.y - behind.y;
    len = sqrtf(dx * dx + dy * dy);
    cars[i].dir.x = dx / len;
    cars[i].dir.y = dy / len;
}

// ------------------------------------------------------------
// Traffic light controller
// ------------------------------------------------------------
Light GetLight(int road)
{
    if (road != control.activeRoad) return LIGHT_RED;
    if (control.stage == STAGE_GREEN) return LIGHT_GREEN;
    if (control.stage == STAGE_YELLOW) return LIGHT_YELLOW;
    return LIGHT_RED;       // all-red pause
}

// Asks the external function for the green time of every road
void RefreshGreenTimes(void)
{
    TrafficInfo info;
    int r, i;
    float t;

    info.cycle = control.cycle;
    for (r = 0; r < NUM_ROADS; r++)
    {
        info.waiting[r] = 0;
        info.lastGreen[r] = control.greenTime[r];
    }

    // Change this to make average
    for (int i = 0;  i < 4; i++){
        int sum = 0;
        for (int j = 0; j < 4; j++) {
            sum += InstantDataSet[j][i];
        }
        info.waiting[i] = sum/3;
    }

    //--------------------------
    calculate_times(info.waiting, TOTAL_TIME, YELLOW_TIME, control.greenTime);
    for (int i = 0; ((i < NUM_ROADS) && DynamicTiming); i++) {
        if (control.greenTime[i] > CLAMP_GREEN) control.greenTime[i] = CLAMP_GREEN;
    }
}

// Order of stages: GREEN -> YELLOW -> ALL RED -> next road's GREEN ...
void UpdateControl(float dt)
{
    control.timer -= dt;

    while (control.timer <= 0)
    {
        if (control.stage == STAGE_GREEN)
        {
            control.stage = STAGE_YELLOW;
            control.timer += YELLOW_TIME;
        }
        else if (control.stage == STAGE_YELLOW)
        {
            control.stage = STAGE_ALL_RED;
            control.timer += ALL_RED_TIME;
        }
        else
        {
            control.activeRoad = (control.activeRoad + 1) % NUM_ROADS;
            if (control.activeRoad == ROAD_NORTH)       // a full cycle just finished
            {
                control.cycle++;
                RefreshGreenTimes();
            }
            // GETTING DATA FOR TRAFFIC AT EACH SIGNAL INSTANT FOR AVG CALCULATION
            for (int i = 0; i < NUM_ROADS; i++) {
                InstantDataSet[control.activeRoad][i] = 0;
            }
            for (int i = 0;  i < MAX_CARS; i++){
                if (control.activeRoad != cars[i].road) {
                    if (cars[i].active && cars[i].s + cars[i].length / 2 < paths[cars[i].road][cars[i].turn].entryS)
                        InstantDataSet[control.activeRoad][cars[i].road]++;
                }
        
            }

            control.stage = STAGE_GREEN;
            control.timer += control.greenTime[control.activeRoad];
        }
    }
}

// ------------------------------------------------------------
// Cars
// ------------------------------------------------------------
void SpawnCar(int road)
{
    int i, slot = -1;
    int r = GetRandomValue(0, 99);
    Car c = { 0 };

    // do not spawn if the last car on this road has not moved away yet
    for (i = 0; i < MAX_CARS; i++)
    {
        if (cars[i].active && cars[i].road == road && cars[i].s < 90) return;
    }

    // find a free slot
    for (i = 0; i < MAX_CARS; i++)
    {
        if (!cars[i].active) { slot = i; break; }
    }
    if (slot == -1) return;

    c.active = 1;
    c.road = road;
    if (r < 50) c.turn = GO_STRAIGHT;           // 50% straight, 25% left, 25% right
    else if (r < 75) c.turn = TURN_LEFT;
    else c.turn = TURN_RIGHT;
    c.length = RandomFloat(AVG_LENGTH-5, AVG_LENGTH+5);
    c.width = CAR_WIDTH;
    c.maxSpeed = RandomFloat(MEDIAN_VEL-25, MEDIAN_VEL+25);
    c.speed = c.maxSpeed / 2;
    c.color = carColors[GetRandomValue(0, 5)];
    c.s = 0;
    c.yellowChoice = YELLOW_UNDECIDED;

    cars[slot] = c;
    UpdateCarPosition(slot);
    carCount++;
}

// Must car i stop at its stop line? gapToLine = distance from its front bumper to the line
int ShouldStop(int i, float gapToLine)
{
    Light light = GetLight(cars[i].road);
    float brakeDistance;

    if (light == LIGHT_GREEN)
    {
        cars[i].yellowChoice = YELLOW_UNDECIDED;
        return 0;
    }

    if (light == LIGHT_YELLOW)
    {
        // decide once: stop if the car can brake before the line, otherwise drive through
        if (cars[i].yellowChoice == YELLOW_UNDECIDED)
        {
            brakeDistance = cars[i].speed * cars[i].speed / (2 * BRAKE);
            if (gapToLine + 1 >= brakeDistance) cars[i].yellowChoice = YELLOW_STOP;
            else cars[i].yellowChoice = YELLOW_GO;
        }
        return cars[i].yellowChoice == YELLOW_STOP;
    }

    // red light: only cars that already decided to go on yellow keep moving
    if (cars[i].yellowChoice == YELLOW_GO) return 0;
    return 1;
}

// Free space between car i and car j if j is in front of i, otherwise NOT_AHEAD
float GapToCar(int i, int j)
{
    Car a = cars[i];
    Car b = cars[j];
    float halfLengths = (a.length + b.length) / 2;
    float dx, dy, forward, sideways;

    // Same road and same route (or still before the box): compare distances along the path
    if (a.road == b.road && (a.turn == b.turn || a.s <= paths[a.road][a.turn].entryS))
    {
        if (b.s <= a.s) return NOT_AHEAD;
        return b.s - a.s - halfLengths;
    }

    // Any other car (for example on the exit road): check if it is right in front of us
    dx = b.pos.x - a.pos.x;
    dy = b.pos.y - a.pos.y;
    forward  = dx * a.dir.x + dy * a.dir.y;             // how far in front
    sideways = fabsf(dx * a.dir.y - dy * a.dir.x);      // how far to the side
    if (forward <= 0 || forward > LOOK_AHEAD || sideways > LANE_WIDTH * 0.4f) return NOT_AHEAD;
    return forward - halfLengths;
}

// Index of the closest car in front of car i, or -1 if there is none
int FindCarAhead(int i)
{
    int j, best = -1;
    float gap, bestGap = NOT_AHEAD;

    for (j = 0; j < MAX_CARS; j++)
    {
        if (j == i || !cars[j].active) continue;
        gap = GapToCar(i, j);
        if (gap < bestGap)
        {
            bestGap = gap;
            best = j;
        }
    }
    return best;
}

void UpdateCar(int i, float dt)
{
    Path p = paths[cars[i].road][cars[i].turn];
    float targetSpeed = cars[i].maxSpeed;
    float gapToLine, gap, safeSpeed;
    int ahead;

    // 1. slow down for turns
    if (cars[i].turn != GO_STRAIGHT && cars[i].s > p.entryS - 60 && cars[i].s < p.exitS + cars[i].length)
        targetSpeed = fminf(targetSpeed, TURN_SPEED);

    // 2. traffic light (only for cars that have not crossed the stop line yet)
    gapToLine = p.stopS - (cars[i].s + cars[i].length / 2);
    if (gapToLine > -2 && ShouldStop(i, gapToLine))
    {
        if (gapToLine < 0) gapToLine = 0;
        targetSpeed = fminf(targetSpeed, sqrtf(2 * BRAKE * gapToLine));     // speed that stops exactly at the line
    }

    // 3. car in front: keep a safe distance
    ahead = FindCarAhead(i);
    if (ahead != -1)
    {
        gap = GapToCar(i, ahead) - MIN_GAP;
        if (gap < 0) gap = 0;
        safeSpeed = sqrtf(cars[ahead].speed * cars[ahead].speed + 2 * BRAKE * gap);
        targetSpeed = fminf(targetSpeed, safeSpeed);
    }

    // 4. speed up or slow down towards the target speed
    if (cars[i].speed < targetSpeed)
    {
        cars[i].speed += ACCEL * dt;
        if (cars[i].speed > targetSpeed) cars[i].speed = targetSpeed;
    }
    else
    {
        cars[i].speed -= 1.5f * BRAKE * dt;
        if (cars[i].speed < targetSpeed) cars[i].speed = targetSpeed;
    }

    cars[i].s += cars[i].speed * dt;
    UpdateCarPosition(i);
}

// A car is removed once it has passed the intersection and is off screen.
// Its slot is cleared (active = 0) and can be reused by a new car.
int HasLeft(int i)
{
    Path p = paths[cars[i].road][cars[i].turn];
    Vector2 pos = cars[i].pos;
    int offScreen = pos.x < -24 || pos.x > SCREEN + 24 || pos.y < -24 || pos.y > SCREEN + 24;

    if (cars[i].s >= p.total) return 1;
    if (cars[i].s > p.exitS && offScreen) return 1;
    return 0;
}

void UpdateCars(float dt)
{
    int i;

    for (i = 0; i < MAX_CARS; i++)
    {
        if (!cars[i].active) continue;
        UpdateCar(i, dt);
        if (HasLeft(i))
        {
            cars[i].active = 0;
            carCount--;
        }
    }
}

// ------------------------------------------------------------
// Drawing
// ------------------------------------------------------------
void DrawRoads(void)
{
    Color grass = { 34, 85, 45, 255 };
    Color asphalt = { 55, 58, 64, 255 };
    int r;

    ClearBackground(grass);
    DrawRectangle(CENTER - ROAD_HALF, 0, 2 * ROAD_HALF, SCREEN, asphalt);      // vertical road
    DrawRectangle(0, CENTER - ROAD_HALF, SCREEN, 2 * ROAD_HALF, asphalt);      // horizontal road

    for (r = 0; r < NUM_ROADS; r++)
    {
        // single centre line, from the box edge to the screen edge
        DrawLineEx(RotatePoint(0, -ROAD_HALF, r), RotatePoint(0, -SCREEN, r), 2, GOLD);

        // white stop line across the lane of cars coming towards the intersection
        DrawLineEx(RotatePoint(0, -(ROAD_HALF + STOP_GAP), r),
                   RotatePoint(LANE_WIDTH, -(ROAD_HALF + STOP_GAP), r), 4, WHITE);
    }
}

void DrawCar(int i)
{
    Rectangle body;
    Vector2 origin;
    float angle = atan2f(cars[i].dir.y, cars[i].dir.x) * RAD2DEG;

    body.x = cars[i].pos.x;
    body.y = cars[i].pos.y;
    body.width = cars[i].length;
    body.height = cars[i].width;
    origin.x = cars[i].length / 2;      // rotate around the middle of the car
    origin.y = cars[i].width / 2;
    DrawRectanglePro(body, origin, angle, cars[i].color);
}

void DrawSignal(int road)
{
    Vector2 pos = RotatePoint(88, -100, road);      // signal box position beside the road
    int x = (int)pos.x;
    int y = (int)pos.y;
    Light light = GetLight(road);
    Rectangle box = { x - 12, y - 32, 24, 64 };
    Color black = { 20, 20, 24, 255 };
    const char *label = roadNames[road];
    int labelX;

    DrawRectangleRounded(box, 0.3f, 6, black);

    // the lit lamp is bright, the others are dim
    if (light == LIGHT_RED) DrawCircle(x, y - 20, 8, RED);
    else DrawCircle(x, y - 20, 8, Fade(RED, 0.25f));
    if (light == LIGHT_YELLOW) DrawCircle(x, y, 8, YELLOW);
    else DrawCircle(x, y, 8, Fade(YELLOW, 0.25f));
    if (light == LIGHT_GREEN) DrawCircle(x, y + 20, 8, GREEN);
    else DrawCircle(x, y + 20, 8, Fade(GREEN, 0.25f));

    // road name, plus the seconds left for the road that is green / yellow
    if (road == control.activeRoad && control.stage != STAGE_ALL_RED)
        label = TextFormat("%s %ds", roadNames[road], (int)ceilf(control.timer));

    if (x < CENTER) labelX = x - 18 - MeasureText(label, 18);
    else labelX = x + 18;
    DrawText(label, labelX, y - 9, 18, RAYWHITE);
}

void DrawInfo(int paused)
{
    const char *stageName = "ALL RED";

    if (control.stage == STAGE_GREEN) stageName = "GREEN";
    if (control.stage == STAGE_YELLOW) stageName = "YELLOW";

    DrawRectangle(8, 8, 450, 88, Fade(BLACK, 0.6f));
    DrawText(TextFormat("Cycle %d | %s: %s %.1fs", control.cycle + 1,
             roadNames[control.activeRoad], stageName, control.timer), 16, 14, 18, RAYWHITE);
    DrawText(TextFormat("Green: N %.0fs  E %.0fs  S %.0fs  W %.0fs", control.greenTime[0],
             control.greenTime[1], control.greenTime[2], control.greenTime[3]), 16, 38, 18, LIGHTGRAY);
    if (paused) DrawText(TextFormat("Cars alive: %d | [SPACE] resume", carCount), 16, 62, 16, LIGHTGRAY);
    else DrawText(TextFormat("Cars alive: %d | [SPACE] pause", carCount), 16, 62, 16, LIGHTGRAY);
}

// One slider per road: 0 = no cars, 1 = normal, 4 = very busy
void DrawSliders(void)
{
    int r;
    Rectangle bounds;

    DrawRectangle(8, 100, 250, 104, Fade(RAYWHITE, 0.85f));
    for (r = 0; r < NUM_ROADS; r++)
    {
        bounds.x = 70;
        bounds.y = 108 + 24 * r;
        bounds.width = 120;
        bounds.height = 18;
        GuiSliderBar(bounds, roadNames[r], TextFormat("%.1f", density[r]), &density[r], 0, 2);
    }
}

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------
int main(void)
{
    float spawnTimer[NUM_ROADS];    // counts down to the next car on each road
    int paused = 0;
    int r, i;
    float dt;

    InitWindow(SCREEN, SCREEN, "Intersection simulation (left-hand traffic)");
    SetTargetFPS(60);
    SetRandomSeed((unsigned int)time(NULL));
    GuiSetStyle(DEFAULT, TEXT_SIZE, 14);

    BuildPaths();

    // start with North green, using the first set of green times
    RefreshGreenTimes();
    control.activeRoad = ROAD_NORTH;
    control.stage = STAGE_GREEN;
    control.timer = control.greenTime[ROAD_NORTH];

    for (r = 0; r < NUM_ROADS; r++) spawnTimer[r] = RandomFloat(0.2f, 3.0f);

    while (!WindowShouldClose())
    {
        dt = GetFrameTime();
        if (dt > 0.05f) dt = 0.05f;
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;

        if (!paused)
        {
            UpdateControl(dt);

            for (r = 0; r < NUM_ROADS; r++)
            {
                spawnTimer[r] -= dt * density[r];       // higher density = timer runs faster
                if (spawnTimer[r] <= 0)
                {
                    SpawnCar(r);
                    spawnTimer[r] = RandomFloat(SPAWN_TIME-2, SPAWN_TIME+2);
                }
            }
            UpdateCars(dt);
        }

        BeginDrawing();
        DrawRoads();
        for (i = 0; i < MAX_CARS; i++)
        {
            if (cars[i].active) DrawCar(i);
        }
        for (r = 0; r < NUM_ROADS; r++) DrawSignal(r);
        DrawInfo(paused);
        DrawSliders();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
