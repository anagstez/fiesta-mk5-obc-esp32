#pragma once
/*
 * Eco-driving assistant (SK_ECO screen): estimates the gear from rpm / speed and advises when to shift for the
 * lowest fuel use. There is no gear signal on CAN, so the gear is the nearest of these ratios (engine rpm per
 * km/h). Measured on this Fiesta 1.4 from the logs in dumps/ (rpm / speed histogram, 2026-11-22): peaks at 137, 73.5,
 * 49 and 36; the 5th (~29) follows from the gearbox ratios 3.58/1.93/1.28/0.95/0.76 (few samples, not verified).
 * Other car / tyres: change ECO_RATIO, it is the only thing that depends on them.
 */
#define ECO_GEARS 5
static const float ECO_RATIO[ECO_GEARS] = { 137.0f, 73.5f, 49.0f, 36.3f, 29.0f };

#define ECO_MIN_KMH      15      /* below this the gear is not estimated (speed has 1 km/h resolution) */
#define ECO_TOL          0.09f   /* ratio further than this from every gear = clutch in / shifting */
#define ECO_RPM_UP       2600    /* shift up at or above this */
#define ECO_RPM_UP_LIGHT 2200    /* ... or above this with a light pedal (gas < ECO_GAS_LIGHT) */
#define ECO_GAS_LIGHT    45
#define ECO_RPM_LOW      1300    /* lugging: below this under load (gas >= ECO_GAS_LOAD) shift down */
#define ECO_GAS_LOAD     35
#define ECO_RPM_FLOOR    1500    /* target gear = highest one that keeps at least this many rpm */
#define ECO_RPM_NEXT_MIN 1300    /* do not advise an upshift that would drop below this */
#define ECO_GAS_HEAVY    75      /* pedal above this: "EASY" (full-load enrichment burns fuel) */
#define ECO_SLOW_KMH     110     /* above this: "SLOW" */
#define ECO_IDLE_STOP_MS 30000   /* idling this long: advise to switch the engine off */
