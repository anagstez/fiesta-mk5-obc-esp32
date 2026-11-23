#pragma once
/*
 * Short explanations of OBD-II fault codes for the DTC screen. The display font has no Cyrillic and no
 * punctuation beyond . % - : /, so the text is plain upper-case English (letters of font[] only).
 * A code that is not listed is shown as "SEE CODE TABLE". Add rows freely: { "P0xxx", "TEXT" }.
 */
typedef struct { const char *code; const char *text; } dtc_desc_t;

static const dtc_desc_t DTC_DESC[] = {
    { "P0100", "MAF SENSOR CIRCUIT" },            { "P0101", "MAF SENSOR RANGE" },
    { "P0102", "MAF SENSOR LOW" },                { "P0103", "MAF SENSOR HIGH" },
    { "P0105", "MAP SENSOR CIRCUIT" },            { "P0106", "MAP SENSOR RANGE" },
    { "P0107", "MAP SENSOR LOW" },                { "P0108", "MAP SENSOR HIGH" },
    { "P0110", "AIR TEMP SENSOR CIRCUIT" },       { "P0111", "AIR TEMP SENSOR RANGE" },
    { "P0112", "AIR TEMP SENSOR LOW" },           { "P0113", "AIR TEMP SENSOR HIGH" },
    { "P0115", "COOLANT TEMP SENSOR CIRCUIT" },   { "P0116", "COOLANT TEMP SENSOR RANGE" },
    { "P0117", "COOLANT TEMP SENSOR LOW" },       { "P0118", "COOLANT TEMP SENSOR HIGH" },
    { "P0120", "THROTTLE SENSOR CIRCUIT" },       { "P0121", "THROTTLE SENSOR RANGE" },
    { "P0122", "THROTTLE SENSOR LOW" },           { "P0123", "THROTTLE SENSOR HIGH" },
    { "P0125", "COOLANT TOO COLD FOR CLOSED LOOP" },
    { "P0128", "THERMOSTAT COOLANT TOO COLD" },
    { "P0130", "O2 SENSOR B1S1 CIRCUIT" },        { "P0131", "O2 SENSOR B1S1 LOW" },
    { "P0132", "O2 SENSOR B1S1 HIGH" },           { "P0133", "O2 SENSOR B1S1 SLOW" },
    { "P0134", "O2 SENSOR B1S1 NO ACTIVITY" },    { "P0135", "O2 SENSOR B1S1 HEATER" },
    { "P0136", "O2 SENSOR B1S2 CIRCUIT" },        { "P0137", "O2 SENSOR B1S2 LOW" },
    { "P0138", "O2 SENSOR B1S2 HIGH" },           { "P0140", "O2 SENSOR B1S2 NO ACTIVITY" },
    { "P0141", "O2 SENSOR B1S2 HEATER" },
    { "P0171", "MIXTURE TOO LEAN BANK 1" },       { "P0172", "MIXTURE TOO RICH BANK 1" },
    { "P0201", "INJECTOR CYL 1 CIRCUIT" },        { "P0202", "INJECTOR CYL 2 CIRCUIT" },
    { "P0203", "INJECTOR CYL 3 CIRCUIT" },        { "P0204", "INJECTOR CYL 4 CIRCUIT" },
    { "P0230", "FUEL PUMP RELAY CIRCUIT" },
    { "P0300", "RANDOM MISFIRE" },                { "P0301", "MISFIRE CYL 1" },
    { "P0302", "MISFIRE CYL 2" },                 { "P0303", "MISFIRE CYL 3" },
    { "P0304", "MISFIRE CYL 4" },
    { "P0325", "KNOCK SENSOR CIRCUIT" },
    { "P0335", "CRANK SENSOR CIRCUIT" },          { "P0336", "CRANK SENSOR RANGE" },
    { "P0340", "CAM SENSOR CIRCUIT" },            { "P0341", "CAM SENSOR RANGE" },
    { "P0350", "IGNITION COIL CIRCUIT" },
    { "P0351", "IGNITION COIL A CIRCUIT" },       { "P0352", "IGNITION COIL B CIRCUIT" },
    { "P0353", "IGNITION COIL C CIRCUIT" },       { "P0354", "IGNITION COIL D CIRCUIT" },
    { "P0420", "CATALYST EFFICIENCY LOW" },
    { "P0442", "EVAP SMALL LEAK" },               { "P0443", "EVAP PURGE VALVE CIRCUIT" },
    { "P0455", "EVAP LARGE LEAK" },
    { "P0480", "COOLING FAN 1 CIRCUIT" },
    { "P0500", "VEHICLE SPEED SENSOR" },          { "P0501", "VEHICLE SPEED SENSOR RANGE" },
    { "P0505", "IDLE CONTROL SYSTEM" },           { "P0506", "IDLE SPEED TOO LOW" },
    { "P0507", "IDLE SPEED TOO HIGH" },
    { "P0562", "SYSTEM VOLTAGE LOW" },            { "P0563", "SYSTEM VOLTAGE HIGH" },
    { "P0601", "PCM MEMORY CHECKSUM" },           { "P0602", "PCM PROGRAMMING" },
    { "P0603", "PCM KEEP ALIVE MEMORY" },         { "P0605", "PCM ROM ERROR" },
    { "P0620", "GENERATOR CONTROL CIRCUIT" },
    { "P0627", "FUEL PUMP CONTROL CIRCUIT" },
    { "P0638", "THROTTLE ACTUATOR RANGE" },
    { "P0645", "A/C CLUTCH RELAY CIRCUIT" },
    { "P1000", "OBD READINESS TESTS NOT COMPLETE" },
    { "U0100", "LOST COMMUNICATION WITH PCM" },   { "U0121", "LOST COMMUNICATION WITH ABS" },
};

static const char *dtc_text(const char *code)
{
    for (size_t i = 0; i < sizeof(DTC_DESC) / sizeof(DTC_DESC[0]); i++)
        if (!strcmp(DTC_DESC[i].code, code)) return DTC_DESC[i].text;
    return "SEE CODE TABLE";
}
