#pragma once
/*
 * Screen layout: edit this file, `idf.py build`, flash. Nothing else needs to change.
 *
 *  - SCREENS[]  list of screens in the order the encoder turns through them (the MENU always comes last,
 *               entered by the 5 s hold). SCREENS[0] must be the DRIVE screen (door popup, cruise and the
 *               menu EXIT work with it).
 *  - A SK_GRID screen is cols x rows cells (cols * rows <= 6) filled from item[] left to right, top to bottom.
 *    max_scale = biggest font scale (1..4, 0 = 3); a value that does not fit its cell is shrunk automatically.
 *  - A SK_GRAPH screen plots 1..3 parameters (item[0..2]) over time: a scrolling line over the last span_s
 *    seconds (0 = 60) and the current values on the left. One parameter: min/max of the window at the right,
 *    warn/alarm limits as dashed lines, the line coloured by level. Two or three: each line is scaled to its
 *    own min..max (units differ) and has its own colour (white, green, pink), no scale numbers. It records while the board is powered; OBD parameters too,
 *    since they are polled non-stop.
 *  - A SK_ECO screen is the eco-driving assistant (fixed layout, see eco.h): estimated gear, shift up/down advice,
 *    coasting hint and the instant consumption. It takes no cols/rows/items.
 *  - PARAMS[]   per parameter: the label printed above the value (incl. unit) and warning thresholds.
 *    Thresholds are in the parameter's own raw unit (see the comment on each enum value); the value turns
 *    AMBER outside warn_lo..warn_hi and RED outside alarm_lo..alarm_hi. NO_LO / NO_HI switch a limit off.
 *  - Labels may use only the characters of font[] in main.c (digits . % - : / A-Z subset); a missing letter
 *    is silently skipped, so add it to font[] first.
 *  - Parameters marked [OBD] come from OBD-II requests to the PCM (read-only Mode 01). Every [OBD] parameter used
 *    by any screen is polled non-stop while CAN is alive, those of the screen shown twice as often; they read
 *    "--" until the PCM answers.
 */
#include <limits.h>
#include <stdint.h>

enum {
    P_NONE,         /* empty cell */
    P_SPEED,        /* km/h */
    P_RPM,          /* rpm */
    P_COOL,         /* degC, coolant (MS-CAN) */
    P_VOLT,         /* 0.1 V */
    P_GAS,          /* % pedal */
    P_TRIP_KM,      /* 0.1 km */
    P_TRIP_TIME,    /* s, shown h:mm */
    P_AVG,          /* km/h, trip average */
    P_MAX_SPEED,    /* km/h */
    P_MAX_RPM,      /* rpm */
    P_CLOCK,        /* min since midnight, shown hh:mm (car clock) */
    P_FUEL_L,       /* 0.1 L burned this trip */
    P_L100,         /* 0.1 L/100 km, trip average */
    P_LEFT_L,       /* 0.1 L, tank estimate from 0x499 (zero point assumed) */
    P_LPH,          /* 0.1 L/h, instant */
    P_L100I,        /* 0.1 L/100 km, instant (needs >= 10 km/h) */
    P_MAP,          /* kPa, intake pressure (MS-CAN 0x420 d7) */
    P_IAT,          /* degC [OBD] */
    P_TIMING,       /* 0.1 deg [OBD], ignition advance */
    P_STFT,         /* 0.1 % [OBD], short-term fuel trim */
    P_LTFT,         /* 0.1 % [OBD], long-term fuel trim */
    P_TRIM,         /* 0.1 % [OBD], STFT + LTFT */
    P_O2A,          /* 0.01 V [OBD], lambda sensor before the catalyst (B1S1) */
    P_O2B,          /* 0.01 V [OBD], lambda sensor after the catalyst (B1S2) */
    P_LOAD,         /* % [OBD], calculated engine load */
    P_ECT,          /* degC [OBD], coolant as the PCM sees it */
    P_THR,          /* % [OBD], throttle position */
    P_FSYS,         /* [OBD] CL = closed loop, OL = open loop */
    P_DTC,          /* [OBD] stored fault codes; "n MIL" when the check-engine lamp is on */
    NPARAM
};

#define NO_LO INT_MIN
#define NO_HI INT_MAX

typedef struct { const char *label; int warn_lo, warn_hi, alarm_lo, alarm_hi; } param_cfg_t;

/* every parameter has to be listed; the numbers below are examples for this Fiesta 1.4 */
static const param_cfg_t PARAMS[NPARAM] = {
    [P_SPEED]     = { "KM/H",      NO_LO, NO_HI, NO_LO, NO_HI },
    [P_RPM]       = { "RPM",       NO_LO, 5500,  NO_LO, 6200  },
    [P_COOL]      = { "COOL C",    NO_LO, 100,   NO_LO, 108   },
    [P_VOLT]      = { "VOLT",      125,   148,   118,   155   },
    [P_GAS]       = { "GAS %",     NO_LO, NO_HI, NO_LO, NO_HI },
    [P_TRIP_KM]   = { "TRIP KM",   NO_LO, NO_HI, NO_LO, NO_HI },
    [P_TRIP_TIME] = { "TIME",      NO_LO, NO_HI, NO_LO, NO_HI },
    [P_AVG]       = { "AVG KM/H",  NO_LO, NO_HI, NO_LO, NO_HI },
    [P_MAX_SPEED] = { "MAX KM/H",  NO_LO, NO_HI, NO_LO, NO_HI },
    [P_MAX_RPM]   = { "MAX RPM",   NO_LO, NO_HI, NO_LO, NO_HI },
    [P_CLOCK]     = { "CLOCK",     NO_LO, NO_HI, NO_LO, NO_HI },
    [P_FUEL_L]    = { "FUEL L",    NO_LO, NO_HI, NO_LO, NO_HI },
    [P_L100]      = { "L/100",     NO_LO, NO_HI, NO_LO, NO_HI },
    [P_LEFT_L]    = { "LEFT L",    80,    NO_HI, 50,    NO_HI },   /* amber under 8 L, red under 5 L */
    [P_LPH]       = { "L/H",       NO_LO, NO_HI, NO_LO, NO_HI },
    [P_L100I]     = { "L/100 NOW", NO_LO, 90,    NO_LO, 130   },   /* amber over 9, red over 13 L/100 */
    [P_MAP]       = { "MAP KPA",   NO_LO, NO_HI, NO_LO, NO_HI },
    [P_IAT]       = { "IAT C",     NO_LO, 60,    NO_LO, 80    },
    [P_TIMING]    = { "TIMING",    NO_LO, NO_HI, NO_LO, NO_HI },
    [P_STFT]      = { "STFT %",    -100,  100,   -250,  250   },
    [P_LTFT]      = { "LTFT %",    -100,  100,   -200,  200   },
    [P_TRIM]      = { "TRIM %",    -150,  150,   -250,  250   },
    [P_O2A]       = { "O2 B1S1 V", NO_LO, NO_HI, NO_LO, NO_HI },
    [P_O2B]       = { "O2 B1S2 V", NO_LO, NO_HI, NO_LO, NO_HI },
    [P_LOAD]      = { "LOAD %",    NO_LO, NO_HI, NO_LO, NO_HI },
    [P_ECT]       = { "ECT C",     NO_LO, 100,   NO_LO, 108   },
    [P_THR]       = { "THR %",     NO_LO, NO_HI, NO_LO, NO_HI },
    [P_FSYS]      = { "FUEL SYS",  NO_LO, NO_HI, NO_LO, NO_HI },
    [P_DTC]       = { "DTC",       NO_LO, NO_HI, NO_LO, 0     },   /* red as soon as one code is stored */
};

enum { SK_DRIVE, SK_GRID, SK_GRAPH, SK_ECO };

typedef struct {
    const char *name;           /* for the log only */
    uint8_t kind;               /* SK_DRIVE (fixed gauge layout) or SK_GRID */
    uint8_t cols, rows;         /* SK_GRID: 1..3 x 1..2 and so on, cols * rows <= 6 */
    uint8_t max_scale;          /* SK_GRID: biggest font scale, 0 = 3 */
    uint8_t item[6];            /* SK_GRID: P_* per cell; SK_GRAPH: item[0..2] are the plotted parameters */
    uint16_t span_s;            /* SK_GRAPH: time window in seconds, 0 = 60 */
} screen_cfg_t;

static const screen_cfg_t SCREENS[] = {
    { "DRIVE", SK_DRIVE, 0, 0, 0, { 0 }, 0 },
    { "ECO",   SK_ECO, 0, 0, 0, { 0 }, 0 },
    { "ECON",  SK_GRID, 2, 2, 3, { P_L100I, P_L100, P_LPH, P_AVG }, 0 },              /* everyday economy: now, trip, per hour, average speed */
    { "TRIP",  SK_GRID, 3, 2, 3, { P_TRIP_KM, P_TRIP_TIME, P_AVG, P_MAX_SPEED, P_MAX_RPM, P_CLOCK }, 0 },
    { "FUEL",  SK_GRID, 3, 1, 4, { P_FUEL_L, P_L100, P_LEFT_L }, 0 },
    { "G L100", SK_GRAPH, 0, 0, 0, { P_L100I, P_SPEED, P_GAS }, 60 },                 /* what the pedal does to the consumption */
    { "G TEMP", SK_GRAPH, 0, 0, 0, { P_COOL, P_IAT }, 600 },                          /* warm-up curve */
    { "HEALTH", SK_GRID, 3, 2, 3, { P_COOL, P_VOLT, P_IAT, P_LOAD, P_TRIM, P_DTC }, 0 },   /* quick check: temps, battery, load, fuel trim, fault codes */
    { "ENG",   SK_GRID, 3, 2, 3, { P_MAP, P_IAT, P_TIMING, P_STFT, P_LTFT, P_O2A }, 0 },
    { "ENG 2", SK_GRID, 3, 2, 3, { P_LOAD, P_ECT, P_THR, P_O2B, P_FSYS, P_DTC }, 0 },
    { "G SPEED", SK_GRAPH, 0, 0, 0, { P_SPEED }, 120 },
    { "G STFT",  SK_GRAPH, 0, 0, 0, { P_STFT }, 60 },
};
