/*
 * left_servo_probe — find the NEW left door servo's true orientation safely.
 *
 * Drives ONLY the left servo (D44). The right servo is never touched, so the
 * one known-good servo cannot be disturbed and two SG90s can never stall
 * together (~700 mA each browns out the shared 5 V rail).
 *
 * ── EVERY RULE HERE WAS PAID FOR WITH A DESTROYED SERVO ──────────────────
 *
 * 1. setup() DOES NOT TOUCH THE SERVO. Every flash and every reset re-runs
 *    setup(); an arm driven from there goes at the SG90's full ~600 deg/s with
 *    no easing. That destroyed servo #1. The calibration sketch
 *    (full_servo_door.ino) still does exactly this — it calls attachBoth()
 *    and then writeAngle(leftClose) — which is why it must not be flashed
 *    with an arm coupled to a door.
 *
 * 2. Servo::attach() begins pulsing at DEFAULT_PULSE_WIDTH (1500 us = 90 deg)
 *    and ignores the stored pulse value. So "attach then write" lunges to
 *    90 deg for at least one frame no matter what. That destroyed servos #2
 *    and #3. writeMicroseconds() IS honoured while detached (Servo assigns
 *    servoIndex in its constructor, not in attach()), so this sketch always
 *    writes the pulse BEFORE attaching.
 *
 * 3. The first attach deliberately targets 90 deg — identical to attach()'s
 *    own default — so the horn does not move at all on the first attach.
 *    That is why 'A' is safe even though the horn's position is unknown.
 *
 * 4. Motion is eased on TIME: one update per 20 ms servo frame (the SG90
 *    latches exactly one pulse per frame, finer is wasted), commanded in
 *    microseconds so steps are sub-degree, with a smoothstep p*p*(3-2p) so the
 *    long arm cannot bounce at the ends.
 *
 * 5. Speed is capped. The bench rig proved ~38 deg/s; a smoothstep peaks at
 *    1.5x its average, so travel time is chosen as
 *        travel_ms = 1000 * 1.5 * delta / 38
 *    An earlier build used 1200 ms for an 85 deg sweep, peaked near 106 deg/s,
 *    and stripped the gears.
 *
 * 6. Pulses are clamped to 600-2350 us so a mis-typed angle cannot park the
 *    servo against its internal end stop, where it grinds and stalls.
 *
 * 7. The servo is released ~400 ms after it settles, so nothing holds torque
 *    (and draws current) while we think about the next step.
 *
 * ── PROTOCOL (115200 baud, newline-terminated) ───────────────────────────
 *   A        attach at 90 deg  — no movement, safe first step
 *   G<deg>   eased move to <deg>, e.g. G130
 *   S<deg>   declare "the horn is physically at <deg>" WITHOUT moving.
 *            Use after moving the horn by hand.
 *   D        detach (servo goes limp)
 *   ?        report belief + attached state
 * Replies are prefixed P, for easy parsing.
 */
#include <Servo.h>

#define SERVO_L        44
#define SERVO_US_MIN  600
#define SERVO_US_MAX 2350
#define FRAME_MS       20UL
#define HOLD_MS       400UL
#define MAX_DEG_PER_S  38.0f   // proven on the bench rig
#define SMOOTH_PEAK     1.5f   // smoothstep peak / average

// Deliberately conservative working band. The firmware's left door lives
// between 90 (open) and 175 (closed); 175 is only 5 deg off the library's
// 2400 us end and SG90s often hit their internal stop before that.
#define PROBE_DEG_MIN   85.0f
#define PROBE_DEG_MAX  178.0f

Servo servo_l;

float pos_deg      = 90.0f;   // what we BELIEVE the horn is at
bool  have_belief  = false;   // false until 'A' or 'S' establishes it
float from_deg     = 90.0f;
float to_deg       = 90.0f;
unsigned long move_start_ms = 0;
unsigned long move_ms       = 0;
bool  moving       = false;
unsigned long settled_ms = 0;
unsigned long last_frame_ms = 0;

char buf[24];
uint8_t buf_len = 0;

int angleToUs(float deg) {
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    int us = (int)(544.0f + deg * (2400.0f - 544.0f) / 180.0f + 0.5f);
    if (us < SERVO_US_MIN) us = SERVO_US_MIN;
    if (us > SERVO_US_MAX) us = SERVO_US_MAX;
    return us;
}

// Write the pulse FIRST, then attach. Never the other way round — see rule 2.
void attachAt(float deg) {
    servo_l.writeMicroseconds(angleToUs(deg));
    if (!servo_l.attached()) servo_l.attach(SERVO_L);
}

void report(const char *what) {
    Serial.print(F("P,"));
    Serial.print(what);
    Serial.print(F(",pos="));
    Serial.print(pos_deg, 1);
    Serial.print(F(",us="));
    Serial.print(angleToUs(pos_deg));
    Serial.print(F(",attached="));
    Serial.print(servo_l.attached() ? 1 : 0);
    Serial.print(F(",belief="));
    Serial.println(have_belief ? 1 : 0);
}

void startMove(float target) {
    if (!have_belief) { Serial.println(F("P,ERR,no belief yet - send A or S first")); return; }
    if (target < PROBE_DEG_MIN || target > PROBE_DEG_MAX) {
        Serial.print(F("P,ERR,refusing "));
        Serial.print(target, 1);
        Serial.print(F(" - outside probe band "));
        Serial.print(PROBE_DEG_MIN, 0); Serial.print(F("-"));
        Serial.println(PROBE_DEG_MAX, 0);
        return;
    }
    float delta = fabs(target - pos_deg);
    if (delta < 0.2f) { report("ALREADY"); return; }

    from_deg = pos_deg;
    to_deg   = target;
    // Rate cap — rule 5.
    move_ms  = (unsigned long)(1000.0f * SMOOTH_PEAK * delta / MAX_DEG_PER_S);
    if (move_ms < FRAME_MS * 3) move_ms = FRAME_MS * 3;
    move_start_ms = millis();
    moving = true;
    attachAt(pos_deg);          // pulse-then-attach at where we already are
    Serial.print(F("P,MOVE,from="));
    Serial.print(from_deg, 1);
    Serial.print(F(",to="));
    Serial.print(to_deg, 1);
    Serial.print(F(",ms="));
    Serial.println(move_ms);
}

void tick() {
    unsigned long now = millis();
    if (moving) {
        if (now - last_frame_ms < FRAME_MS) return;
        last_frame_ms = now;
        float p = (float)(now - move_start_ms) / (float)move_ms;
        if (p >= 1.0f) p = 1.0f;
        float e = p * p * (3.0f - 2.0f * p);            // smoothstep
        pos_deg = from_deg + (to_deg - from_deg) * e;
        servo_l.writeMicroseconds(angleToUs(pos_deg));
        if (p >= 1.0f) {
            moving = false;
            settled_ms = now;
            report("ARRIVED");
        }
    } else if (servo_l.attached() && settled_ms && now - settled_ms > HOLD_MS) {
        servo_l.detach();                               // rule 7
        settled_ms = 0;
        report("RELEASED");
    }
}

void handle(char *s) {
    switch (s[0]) {
        case 'A':
            // Rule 3: 90 deg equals attach()'s own default, so this cannot move
            // the horn even though we do not know where it is.
            pos_deg = 90.0f;
            have_belief = true;
            attachAt(90.0f);
            settled_ms = millis();
            report("ATTACHED_AT_90");
            break;
        case 'G': startMove(atof(s + 1)); break;
        case 'S':
            pos_deg = atof(s + 1);
            have_belief = true;
            report("BELIEF_SET");       // moves nothing
            break;
        case 'D':
            if (servo_l.attached()) servo_l.detach();
            settled_ms = 0;
            report("DETACHED");
            break;
        case '?': report("STATE"); break;
        default:  Serial.println(F("P,ERR,unknown - use A G<deg> S<deg> D ?"));
    }
}

void setup() {
    // RULE 1: nothing but Serial. The servo is not attached and not commanded.
    Serial.begin(115200);
    delay(50);
    Serial.println(F("P,BOOT,left_servo_probe D44 - servo NOT attached"));
    Serial.println(F("P,HELP,A=attach@90 G<deg>=move S<deg>=set-belief D=detach ?=state"));
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (buf_len) { buf[buf_len] = 0; handle(buf); buf_len = 0; }
        } else if (buf_len < sizeof(buf) - 1) {
            buf[buf_len++] = c;
        }
    }
    tick();
}
