/**
  ******************************************************************************
  * @file    arm_config.h
  * @brief   All tunable parameters for the robot arm port.
  *
  *          Migration source : ftobler/robotArm (Arduino Mega 2560 @ 16 MHz)
  *          Target           : STM32F407VET6 + STM32 HAL + FreeRTOS (CMSIS_V2)
  *
  *          Values tagged [KEEP] are hardware constants taken from the
  *          original firmware and MUST NOT be changed, otherwise the PC
  *          host (Delphi) will no longer match the machine.
  ******************************************************************************
  */

#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

/*============================================================================*/
/* 0. Feature switches                                                        */
/*============================================================================*/

/** 1 = legacy speed law v = sqrt(dist) * 10 (original interpolation.cpp:69).
  *     Keep 1 to match the feel of the PC host.                                    */
#define ARM_KEEP_LEGACY_SPEED            1

/** 1 = send the "start" banner that robotArm.ino:69 sent.
  *
  *  DEFAULT 0, deliberately.  The original firmware printed "start" once after
  *  setup(), and the handover document carried that over.  Reading the Delphi
  *  host closely shows that it is unsafe, though:
  *
  *    UnitRobotArm.pas:174-188  addPhraseToInbuf() only ever APPENDS to inbuf,
  *                              it is never cleared.
  *    UnitRobotArm.pas:162-166  the stop-and-wait lock is released only when
  *                              pos('ok', inbuf.Strings[0]) = 1, i.e. when the
  *                              FIRST line ever received starts with "ok".
  *
  *  So if "start" is the first line the host sees, inbuf[0] stays "start" for
  *  the rest of the session, pos('ok','start') is 0, and canWriteCommand never
  *  becomes true again - the host sends exactly one command and then locks up
  *  forever.  That happens whenever the PC is already connected and the board
  *  is reset or powered up.
  *
  *  The banner is not part of the handshake and carries no information the host
  *  acts on, so omitting it costs nothing.  Set to 1 only if you are driving
  *  the board from a plain terminal and want the old parity.                  */
#define ARM_EMIT_START_BANNER            0

/** 1 = check the software angle limits before a move is accepted.           */
#define ARM_SOFT_LIMIT_ENABLE            1

/** 1 = build the debug log on USART3 (PB10/PB11). 0 = USART3 stays silent.  */
#define ARM_DEBUG_UART_ENABLE            1

/** 1 = emit a short "boot report" on the debug UART.                        */
#define ARM_DEBUG_BOOT_REPORT            1

/*============================================================================*/
/* 1. Mechanism constants  [KEEP]  (robotArm.ino:54-57, RampsStepper.cpp:88)  */
/*============================================================================*/

#define ARM_GEAR_RATIO                   (32.0f / 9.0f)   /* big 32 / small 9  */

/**
  * Micro-step setting of the TB6600 DIP switches.  MUST match the hardware:
  *
  *   16 -> SW1 OFF / SW2 OFF / SW3 ON   (3200 pulses/rev, the value this
  *                                       project was calibrated with)
  *   32 -> SW1 OFF / SW2 OFF / SW3 OFF  (6400 pulses/rev, finer, quieter at
  *                                       low speed, half the step size)
  *
  * Only the pulse count changes: every angle in this file is in radians, so the
  * kinematics, the soft limits and the G-code interface stay identical.  If the
  * firmware and the DIP disagree, every commanded angle comes out at twice or
  * half the intended size - check with one 30 degree base rotation.
  */
#define ARM_MICROSTEPS                   16

#define ARM_STEPS_PER_REV                (200 * ARM_MICROSTEPS)
#define ARM_FACTOR_DIVISOR               (2.0f * 3.14159265358979323846f)

/** radToStepFactor = (32/9) * steps_per_rev / 2pi    [3200 -> 1811.44 steps/rad] */
#define ARM_RAD_TO_STEP                  (ARM_GEAR_RATIO * (float)ARM_STEPS_PER_REV / ARM_FACTOR_DIVISOR)

/** Upper / lower arm shank length, robotGeometry.cpp:54  [KEEP = 120 mm]    */
#define ARM_SHANK_MM                     120.0f

/** Power-on pose, robotArm.ino:60-63.
  *
  *  SIGN CONVENTION - this is not the value the Arduino original used.
  *
  *  The Arduino wrote stepperHigher.setPositionRad(+PI/2) and then, on the very
  *  first loop() iteration, called stepperHigher.stepToPositionRad(
  *  geometry.getHighRad()), which for that same pose is -PI/2.  So the original
  *  actually turns the upper arm half a turn at power up; it gets away with it
  *  because it has no soft limits.
  *
  *  This firmware is delta based - every MotionTask cycle compares the geometry
  *  output against the stored logical position - so both sides MUST share one
  *  sign convention.  Otherwise the first G1 is either rejected as a soft limit
  *  violation (what used to happen) or swings the arm 180 degrees.
  *
  *  The geometry is the authoritative side:
  *      arm_geometry_solve(0, 120, 120) -> rot = 0, low = 0, high = -PI/2.
  */
#define ARM_HOME_ROT_RAD                 0.0f
#define ARM_HOME_LOW_RAD                 0.0f
#define ARM_HOME_HIGH_RAD                (-3.14159265358979323846f / 2.0f)   /* -90 deg */

/** Power-on interpolation start/target, robotArm.ino:67  [KEEP = no move]   */
#define ARM_BOOT_X_MM                    0.0f
#define ARM_BOOT_Y_MM                    120.0f
#define ARM_BOOT_Z_MM                    120.0f

/*============================================================================*/
/* 2. Step pulse engine  (handover doc section 4.4)  -- LEGACY, SEE 2c        */
/*============================================================================*/

/* NOTE (2026-11-05): the TIM2/TIM3 PWM + DMA ping-pong engine described by the
   macros below has been replaced by the software generator of section 2c; the
   pulse timing now comes from ARM_PULSE_TICK_US.  The values are kept because
   the calibration constants they document (30 us per pulse at full speed, 6/26
   us active high/low for the two wiring styles) still describe the TB6600
   interface, and because removing them would make the handover document's
   history unreadable.  Nothing in the firmware reads them any more.          */

/** On-chip step generator clock after prescaler.  TIM2/TIM3 sit on APB1
  * timer clock = 84 MHz, PSC = 83  ->  1 MHz  ->  1 tick == 1 us.           */
#define ARM_STEP_TIMER_HZ                1000000u

/** Tick period held during a pulse.  ARR = 29  ->  30 us  ->  33.33 kHz,
  * which reproduces the original 5 us + 20 us + 5 us delayMicroseconds
  * budget exactly (RampsStepper.cpp:66-85).                                 */
#define ARM_STEP_TICKS_PER_PULSE         30u

/** High time inside one pulse period: 6 ticks = 6 us (matches CCR = 6).     */
#define ARM_STEP_PULSE_HIGH_TICKS        6u

/** Speed regulation window, section 4.4:
  *   ARR = 1000000 / step_rate_Hz - 1
  *   STEP_ARR_MIN = 24   -> 40 kHz max  (DRV8825 recommendation)
  *   STEP_ARR_MAX = 4999 -> 200 Hz min
  * ARR_TICKS = ARR + 1 is what this firmware actually uses.                 */
#define ARM_STEP_ARR_MIN                 24u
#define ARM_STEP_ARR_MAX                 4999u
#define ARM_STEP_ARR_TICKS_MIN           (ARM_STEP_ARR_MIN + 1u)   /* 25  -> 40.0 kHz */
#define ARM_STEP_ARR_TICKS_MAX           (ARM_STEP_ARR_MAX + 1u)   /* 5000 -> 200 Hz  */

/** Default period.  Pinned at 30 us so the byte-for-byte pulse timing of the
  * original firmware is preserved at full speed.                            */
#define ARM_STEP_DEFAULT_TICKS           ARM_STEP_TICKS_PER_PULSE

/** Ping-pong DMA depth.  1024 half-words == 512 pulse periods == 15.4 ms at
  * 33.33 kHz.  Two halves per axis, three axes -> 12 KB total.              */
#define ARM_STEP_PP_ENTRIES              1024u
#define ARM_STEP_PP_HALVES               2u
#define ARM_STEP_PP_TOTAL                (ARM_STEP_PP_ENTRIES * ARM_STEP_PP_HALVES)

/** Fractional step accumulator resolution.                                   */
#define ARM_STEP_FRAC_ONE                65536u

/*============================================================================*/
/* 2c. Software pulse generator  (2026-11-05)                                 */
/*============================================================================*/

/** Period of the pulse generator tick, in microseconds.
  *
  * The DMA ping-pong engine of section 2 is no longer used: the two arm joints
  * had to share one TIM3 DMA request (so one of them always queued up and then
  * dumped its backlog in a burst) and the fixed 1024 entry table covered
  * anything between 31 ms and 5 s depending on speed (so a partly filled half
  * buffer played a burst and then went silent).  Both effects are visible on the
  * machine as "the base stops, the big arm jerks, the base carries on".
  *
  * The pulses are now produced by TIM7 in software, one tick per step of a
  * small state machine per axis.  This value is the resolution of the pulse
  * timing AND the width of one STEP pulse:
  *
  *   20 us -> 50 kHz tick, 20 us pulse, +-20 us quantisation,
  *            shortest pulse interval 40 us = 25 kHz,
  *           about 9 % CPU at -O0 while all three axes are moving.
  *
  * It must divide 1 MHz, i.e. it must be a whole number of microseconds.
  * Raising it lowers the CPU load and the resolution; lowering it does the
  * opposite.  20 us is the point where the quantisation (+-3 % at the 1.7 kHz
  * the base rotates at) is inaudible and the CPU cost is still small.        */
#define ARM_PULSE_TICK_US                20u

/*============================================================================*/
/* 2b. Driver interface: TB6600                                               */
/*============================================================================*/

/** TB6600 input polarity.
  *
  *  0 = COMMON CATHODE / differential
  *      PUL+, DIR+, ENA+ carry the MCU signal, PUL-, DIR-, ENA- go to MCU GND.
  *      Everything is active high except ENA, which the TB6600 defines as
  *      active low.  This is the wiring a RAMPS 1.4 + DRV8825/A4988 stack used.
  *
  *  1 = COMMON ANODE  <-- current build
  *      PUL+, DIR+, ENA+ are tied to +5 V and PUL-, DIR-, ENA- go to the MCU.
  *      The opto-coupler cathode now faces the MCU, so the MCU has to SINK
  *      current to assert a signal and every input becomes ACTIVE LOW.
  *
  *      Consequences handled by this firmware:
  *        - STEP must be an active low pulse.  The timer channels are switched
  *          from PWM mode 1 to PWM mode 2 by arm_stepper_configure_step_mode().
  *          The DMA table still holds "this period carries a pulse", so the
  *          ping-pong refill and the pulse counting are untouched.
  *        - DIR is inverted.
  *        - ENA is inverted (ENA was already active low, so it becomes active
  *          high: drive it HIGH to disable the driver at power up).
  *
  *      This switch must stay in step with the CubeMX CHx polarity setting.
  *      See the .ioc checklist in the delivery notes: if the .ioc polarity is
  *      also inverted the two cancel out and the motor stays silent.
  */
#define ARM_DRIVER_COMMON_ANODE          1

/** Length of the active part of one step period, in 1 us ticks.
  *
  *  The period itself is unchanged at ARM_STEP_TICKS_PER_PULSE, so the STEP
  *  frequency - and with it the entire step/mm and step/rad calibration - is
  *  identical to the original firmware.  Only the mark/space split inside the
  *  period differs.
  *
  *  Common cathode: 6 us active, 24 us rest, same as the Arduino original.
  *  Common anode : the pulse is now the LOW part, so the value below is the
  *                 LOW time.  It is lengthened to 26 us because the TB6600
  *                 opto-coupler is much slower than the DRV8825 logic input:
  *                 the data sheet asks for >= 2.2 us and the coupler needs
  *                 roughly 1 us to turn on and up to 5 us to release, so a 6 us
  *                 pulse would sit uncomfortably close to the release time.
  *                 With PWM mode 2 a CCR of 0 keeps the output HIGH between
  *                 moves, which is the idle state for common anode, so the
  *                 longer pulse automatically shortens the resting HIGH time
  *                 rather than the other way round.
  */
#if (ARM_DRIVER_COMMON_ANODE == 1)
#define ARM_STEP_ACTIVE_TICKS            26u
#else
#define ARM_STEP_ACTIVE_TICKS            6u
#endif

/** Remaining ticks at the inactive level inside one step period. */
#define ARM_STEP_REST_TICKS              (ARM_STEP_TICKS_PER_PULSE - ARM_STEP_ACTIVE_TICKS)

/** Which logic level ENERGISES a TB6600 driver (M17).
  *
  *  Deliberately a separate switch from ARM_DRIVER_COMMON_ANODE: the ENA input
  *  is the one signal whose polarity really does differ between TB6600 module
  *  revisions, while STEP and DIR are unambiguous.
  *
  *  MEASURED ON THE REAL MACHINE (2026-11-02), this is the authoritative data:
 *
 *    ENA pin FLOATING (MCU held in reset) -> shaft STIFF  (driver energised)
 *    ENA pin driven LOW                   -> shaft LOOSE  (driver disabled)
 *
 *  A floating input therefore behaves like a HIGH on this module, and the MCU
 *  has to actively pull the pin LOW to switch the driver off.  That is exactly
 *  what arm_board_drivers_disable() does at power up, so the motor comes up
 *  loose and M17 must drive HIGH to energise.
 *
 *    0 = drive the pin HIGH to energise     <-- current build
 *    1 = pull the pin LOW  to energise
 *
 *  How to tell which one you need, once it is wired:
 *    send M17 -> the shaft must become stiff (holding torque)
 *    send M18 -> the shaft must turn freely by hand
 *  If it behaves the other way round, flip this macro and rebuild.
  */
#define ARM_ENABLE_ACTIVE_LOW            0

/* The GPIO level that follows from it is derived where the HAL is already in
   scope (arm_stepper.c and arm_board.c).  It is deliberately not defined here,
   because this header is also pulled in through arm_types.h before main.h, and
   GPIO_PIN_SET would not exist yet at that point. */

/*============================================================================*/
/* 3. Motion task  (handover doc section 4.10)                                */
/*============================================================================*/

/** Control period of MotionTask.  1 kHz -> up to 40 steps per axis per tick
  * at the 40 kHz ceiling.                                                    */
#define ARM_MOTION_PERIOD_MS             1u

/** Hard ceiling for the commanded feed rate, in mm/s.
  *
  * WHY THIS EXISTS
  * ---------------
  * `F` is a real feed rate: arm_interp_start() uses it directly as v (only a
  * value below 5 mm/s falls back to the legacy law v = sqrt(dist)*10), so a
  * single wrong character on the link - F500 instead of F50 - asks for a step
  * rate ten times higher than the mechanism can start from, and the motors
  * simply lose steps instead of reporting anything.  The payload never had this
  * problem because the Delphi host does not send F at all and the legacy law
  * tops out around 155 mm/s for a 240 mm move.
  *
  * 120 mm/s is just above the fastest speed actually verified on this machine
  * (a 60 degree base rotation in 1.09 s, i.e. 109.5 mm/s, `short` = 0 over 60
  * consecutive moves on 2026-11-05), so it never slows a validated move down.
  *
  * 2026-11-06 speed ladder (base 60 deg out + F60 return, 16 segments):
  * F60 / F80 / F100 / F120 / F140 / F160 / F180 / F200 all gave `short` = 0,
  * pos exactly 599 then 0/0/-899, no PATH LEAVES, no soft limit
  * (stage-2 doc, section 23.15).  `short` = 0 only proves the queue never
  * overflowed, so a 10-cycle drift test was run per level: at F200 the
  * PHYSICAL arm crept 6..10 deg in 11 cycles (base peak ~5200 steps/s,
  * i.e. 98 rpm at the motor, the torque knee of a 24 V TB6600), while the
  * same test at F60 returned exactly home every time.  On top of the lost
  * steps the arm is visibly flung and the base is kicked by the arm's
  * inertia, so the ceiling went BACK to 120: that is the number actually
  * proven over 60 consecutive moves, and it is the one to trust.
  * Raise it only after the higher speed has been measured with `short` = 0.   */
#define ARM_MAX_SPEED_MM_S               120.0f

/** Feed rate of the automatic return to the power-on pose (M400).
  *
  *  10 mm/s sits far below the ~77 mm/s at which the lower arm starts losing
  *  steps while it lifts, and the longest return inside the workspace is about
  *  200 mm, i.e. at most ~20 s.  The return is not part of the user's program,
  *  so it must never add lost steps of its own - it is the one move whose job
  *  is to be boring.  Do not raise this to "make it quicker" without a drift
  *  test (section 23.15.C).                                                   */
#define ARM_AUTO_HOME_F_MM_S             10.0f

/** Dwell upper bound, guards against G4 T999999.                            */
#define ARM_MAX_DWELL_MS                 3600000u

/*============================================================================*/
/* 4. Software angle limits  (section 4.9)  -- CALIBRATE ON THE REAL MACHINE  */
/*============================================================================*/

/** Software limits, expressed in the SAME convention as arm_geometry_solve().
  *
  *  These are not guesses.  They come from evaluating the original
  *  robotGeometry.cpp formulas over the whole reachable workspace
  *  (shank = 120 mm, so rside <= 240 mm), sampled on a 10 mm grid:
  *
  *      rot  : -84.8 deg ...  +84.8 deg
  *      low  : -56.4 deg ... +153.3 deg
  *      high : -153.3 deg ... +56.4 deg
  *
  *  with a little margin added.  The previous values (+0..150 for high) were
  *  written for the Arduino's "+90 deg" upper arm convention and rejected every
  *  single G1 with "// soft limit: upper arm".
  *
  *  Still to be trimmed against the real mechanism: measure how far each joint
  *  can actually travel and narrow these.  The values below only guarantee that
  *  every pose the kinematics can produce is accepted.
  *
  *  MEASURED ON THE REAL MACHINE (2026-11-05, user): the lower arm has a
  *  PHYSICAL STOP 20 degrees BEHIND the power-on pose, i.e. it may not go below
  *  low = -20 deg.  The IK can produce -56 deg and the limit used to be -60,
  *  so any command that folded the arm in (Y below about 83 mm) or turned the
  *  base further than 95 deg would have driven it into that stop.  The minimum
  *  is therefore -18 deg: 2 deg of margin for the model/hardware lag.
  *
  *  MEASURED ON THE REAL MACHINE (2026-11-07 21:30, user, corrected):
  *    - low IS the lower-arm angle measured from the horizonta  power-on pose,
  *      one to one, and POSITIVE = the arm swings DOWN towards the table.
  *      Verified twice: low = +14.4 deg tilted the arm 15 deg down, low = -9.5
  *      deg lifted it 10 deg (tk rising, so the drivers were enabled).
  *    - from that pose the lower arm travels 20 deg DOWN and 120 deg UP, so
  *      low in [-120, +20], soft limits -118 .. +18.
  *      (The earlier 2026-11-05 note "20 deg behind the power-on pose" was the
  *      same stop, seen from the other side.)
  *    - the angle between the two arms (the ELBOW angle, = low - high) spans
  *      about 120 deg (arm stretched) down to 35..40 deg, but only while the
  *      lower arm is LIFTED.  With the lower arm near horizontal the linkage
  *      hits the base/table at about 60 deg.  Hence the two-step minimum below:
  *      60 deg while low > -20 deg, 40 deg once the arm is lifted past that.
  *    - the stretched position measures 120 deg, so the max is 121 deg.
  *
  *  Consequences worth knowing:
  *    - a base rotation of +-90 deg dips the lower arm to -17.5 deg halfway
  *      along the chord, well inside both limits;
  *    - reaching OUT along the table needs low > +18 deg beyond Y about 158 mm,
  *      i.e. past the physical down stop: those targets are now refused with
  *      "// soft limit: lower arm" instead of being driven into the stop
  *      (2026-11-07: Y160 = low +19.6 deg pressed the gripper into the table and
  *      the reaction lifted the whole machine);
  *    - a straight G1 that passes close to the shoulder folds the elbow and is
  *      refused with "// soft limit: elbow".  This is why the folded "arm up +
  *      minimum elbow" rest pose cannot be the model home.                     */
#define ARM_LIM_ROT_MIN_RAD              (-90.0f  * 3.14159265358979323846f / 180.0f)
#define ARM_LIM_ROT_MAX_RAD              ( 90.0f  * 3.14159265358979323846f / 180.0f)
#define ARM_LIM_LOW_MIN_RAD              (-118.0f * 3.14159265358979323846f / 180.0f)
#define ARM_LIM_LOW_MAX_RAD              ( 18.0f  * 3.14159265358979323846f / 180.0f)
#define ARM_LIM_HIGH_MIN_RAD             (-200.0f * 3.14159265358979323846f / 180.0f)
#define ARM_LIM_HIGH_MAX_RAD             ( 100.0f * 3.14159265358979323846f / 180.0f)

/** Coupled elbow limit: elbow = -(low + high) with the corrected kinematics
  * (= 2*acos(rside / 240), see arm_geometry.c).  The two shanks cannot fold
  * past each other, and how far they may close depends on how far the lower
  * arm is lifted (measured 2026-11-07): with the lower arm near horizontal it
  * hits the base at about 60 deg, lifted it can close to 40 deg.
  * Checked in arm_limits_check_pose() together with the per-joint ranges.   */
#define ARM_LIM_ELBOW_MAX_RAD            (121.0f * 3.14159265358979323846f / 180.0f)
#define ARM_LIM_ELBOW_SPLIT_RAD          (-20.0f * 3.14159265358979323846f / 180.0f)
#define ARM_LIM_ELBOW_MIN_NEAR_RAD       ( 60.0f * 3.14159265358979323846f / 180.0f)
#define ARM_LIM_ELBOW_MIN_FAR_RAD        ( 40.0f * 3.14159265358979323846f / 180.0f)

/** Wiring style of the five end stops, PC0..PC4.
  *
  *  1 = ACTIVE LOW  (default)
  *      Normally OPEN switch between the pin and GND.  Internal pull up keeps
  *      the pin HIGH while the arm is away from the end stop, and the switch
  *      pulls it LOW when the axis reaches the limit.
  *      Broken wire  -> pin stays HIGH -> the limit is never detected.
  *
  *  0 = ACTIVE HIGH
  *      Normally CLOSED switch between the pin and GND.  At rest the switch
  *      holds the pin LOW; reaching the limit OPENS it and the pull up takes
  *      the pin HIGH.
  *      Broken wire  -> pin goes HIGH -> reported as a limit hit.  This is the
  *      fail safe arrangement used on CNC machines.
  *
  *  Only the electrical meaning of the level changes; the EXTI edge handling,
  *  the 5 ms debounce and the soft limits are identical either way.
  *  The emergency stop on PE6 is NOT affected: it is always a momentary button
  *  to GND and therefore always active low.
  */
#define ARM_LIMIT_ACTIVE_LOW             1

/** Emergency stop (PE6) behaviour: stop the pulses only, or cut the drivers too?
  *
  *  1 = KEEP TORQUE (this bench setup)
  *      PE6 stops every axis within one pulse tick and aborts the running
  *      segment, but the TB6600 enable lines stay active, so the arm holds its
  *      pose instead of dropping under its own weight.  This matters here
  *      because the machine has no brake and the power-on pose has the lower
  *      arm horizontal: releasing the drivers drops it onto the table.
  *      Recovery: release the button, send M17.  Send M18 to release the
  *      torque once the arm is supported.
  *      Price: while the stop is latched the joints cannot be pushed aside by
  *      hand, and the drivers keep drawing holding current.
  *
  *  0 = CLASSIC E-STOP
  *      The enable lines are dropped as well, so the arm is free - and a raised
  *      arm falls.  Choose this only if PE6 is a real safety stop and you would
  *      rather have no torque than a held pose.
  *
  * Either way the stop itself is identical: pulse generation stops in the same
  * tick, the segment is abandoned, the fault latches and M17 is required before
  * anything moves again.
  */
#define ARM_ESTOP_KEEP_TORQUE            1

/** End stops can be omitted entirely on a bench setup.  With nothing wired the
  * internal pull up makes every limit read HIGH, which is the idle level for
  * ARM_LIMIT_ACTIVE_LOW == 1, so no spurious fault is produced.  The soft
  * angle limits below still protect the mechanism.                            */

/*============================================================================*/
/* 5. I/O and alarm timing  (section 4.2.5)                                   */
/*============================================================================*/

#define ARM_LIMIT_DEBOUNCE_MS            5u      /* section 4.9              */
#define ARM_SAFETY_PERIOD_MS             10u     /* SafetyTask period        */
#define ARM_HEARTBEAT_PERIOD_MS          250u    /* PE0 toggles every 250 ms */

/** Period of the diagnostic line on USART3.
  *
  * It used to be 1 s while idle and 100 ms while moving.  That looked clever
  * until a 0.7 s move fell completely between two 1 s prints (the delay is
  * chosen from the state at the END of the print, so one print landing in the
  * gap between two segments skips a whole move) - which is exactly what
  * happened during the 2026-11-05 stress test: 60 moves, and not a single
  * sample from the middle of a base rotation.
  *
  * A fixed 200 ms (5 lines/s, about 600 bytes/s on a 115200 link) is cheap
  * enough to leave on and gives 3..5 samples inside every segment.          */
#define ARM_DIAG_PERIOD_MS               200u
#define ARM_FAULT_FLASH_PERIOD_MS        100u    /* 5 Hz alarm blink         */
#define ARM_WARN_FLASH_MS                1000u   /* 1 s blink on WARN        */

/*============================================================================*/
/* 6. Serial protocol  (sections 3.3, 3.4, 4.7)                               */
/*============================================================================*/

/** G-code link, USART2 PD5/PD6.  9600 8N1  [KEEP]                          */
#define ARM_GCODE_UART_BAUD              9600u

/** Debug link, USART3 PB10/PB11.                                            */
#define ARM_DEBUG_UART_BAUD              115200u

/** Circular DMA receive buffer for the G-code link.                          */
#define ARM_UART_RX_RING                 128u

/** Longest accepted command line (original used a heap String).              */
#define ARM_LINE_MAX                     64u

/** Blocking transmit timeout for both links.                                 */
#define ARM_UART_TX_TIMEOUT_MS           100u

/** Command queue depth, matches Queue<Cmd> queue(15) in robotArm.ino:20.     */
#define ARM_CMD_QUEUE_DEPTH              15u

/** Max parameters parsed from one line, original loop was "while (i < 5)".   */
#define ARM_CMD_MAX_VALUES               5u

/** Line terminator accepted on the G-code link.
  *
  *  0 = original semantics: only CR ends a line, a bare LF is discarded
  *      (robotArm.ino:18-20, and what the Delphi host relies on because its
  *      com.writeLn() sends CR LF).
  *
  *  1 = also accept a bare LF as the terminator.
  *
  *  Both settings behave identically for the Delphi host: it sends "\r\n", the
  *  CR terminates the line and the LF then forms an empty line, which is
  *  dropped by the "line length > 0" check before parsing.  Setting this to 1
  *  only matters for generic serial terminals whose send box appends just LF,
  *  which otherwise produce no reaction at all.
  */
#define ARM_ACCEPT_LF_TERMINATOR         0

/*============================================================================*/
/* 7. FreeRTOS task geometry  (section 4.10)                                  */
/*============================================================================*/

/**
  * @brief  [学习标注] 优先级真值：CMSIS-RTOS V2 的枚举是 32/24/16/8（不是这里注释写的 4/3/2/1，
  *         那是“第几级”）——FreeRTOS 直接把这个枚举值当任务优先级用（configMAX_PRIORITIES=56
  *         ）。
  */
#define ARM_PRIO_MOTION                  osPriorityAboveNormal   /* 4 */
#define ARM_PRIO_COMMAND                 osPriorityNormal        /* 3 */
#define ARM_PRIO_SAFETY                  osPriorityBelowNormal   /* 2 */
#define ARM_PRIO_LED                     osPriorityLow           /* 1 */
#define ARM_PRIO_DEFAULT                 osPriorityLow           /* 1 */

/* Stack sizes in 32-bit words (arm_app.c multiplies by 4).
 *
 * These are measured, not guessed.  The Debug build runs at -O0, and any task
 * that reaches vsnprintf() needs a lot of room:
 *
 *   CommandTask   : arm_uart_print_comment() formats "// ..." with snprintf
 *   defaultTask   : arm_uart_debugf() formats the diag: line
 *
 * defaultTask used to get 128 words (512 bytes) from the CubeMX generated
 * attributes in main.c.  That was enough to overflow, and because
 * vApplicationStackOverflowHook() was an empty function the overflow was
 * totally silent: the stack ran off the bottom of ucHeap, overwrote xTickCount
 * and friends with return addresses, a configASSERT then fired and the CPU sat
 * in "cpsid i; b ." with the tick dead and PE0 stuck high.                */
#define ARM_STACK_MOTION                 384u
#define ARM_STACK_COMMAND                512u
#define ARM_STACK_SAFETY                 384u
#define ARM_STACK_LED                    192u
#define ARM_STACK_DEFAULT                512u

/*============================================================================*/
/* 8. Unit helpers                                                            */
/*============================================================================*/

#define ARM_DEG2RAD(d)                   ((d) * 3.14159265358979323846f / 180.0f)
#define ARM_RAD2DEG(r)                   ((r) * 180.0f / 3.14159265358979323846f)
#define ARM_PI                           3.14159265358979323846f

#endif /* ARM_CONFIG_H */
