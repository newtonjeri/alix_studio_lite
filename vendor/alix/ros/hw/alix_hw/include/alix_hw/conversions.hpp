// Pure unit-conversion functions and constant tables for the Butterfli
// real-hardware component. Everything here is side-effect free and
// unit-tested without the SDK (test/test_conversions.cpp, test/test_mimic.cpp).
//
// Ground truth sources:
//   - deg/rad: replaces rm_driver's low-precision constants
//     (DEGREE_RAD = 0.01745, RAD_DEGREE = 57.295791433) with M_PI-exact math.
//   - Lift units: HARMONIZED 1:1 with MoveIt — HW units are TRUE
//     millimetres in both directions (cmd: metres x 1000 -> HW mm; fb:
//     HW mm / 1000 -> metres). The former 2/3 command / 3/2 feedback gearing
//     (Documentation/hardware_integration_changes.md §4.3,
//     rm_driver.cpp:3946-3954) is DEPRECATED — see the lift block below.
//   - Hand scale/order: hardware_integration_changes.md §5.2/§5.4 and
//     rm_control.cpp joint_name_to_hw_index() — SDK DOF order is
//     [little, ring, middle, index, thumb_flex(thumb_2), thumb_rot(thumb_1)],
//     command value = 1000 - round(angle/max_rad * 1000) (1000 = open).
//   - Mimic multipliers/offsets: inspire_*_hand.ros2_control.xacro (URDF).

#ifndef ALIX_HW__CONVERSIONS_HPP_
#define ALIX_HW__CONVERSIONS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace alix_hw
{

// ---------------------------------------------------------------------------
// Angle units (arm joints). SDK commands/reports degrees (0.001 deg).
// ---------------------------------------------------------------------------
inline constexpr double kDegPerRad = 180.0 / M_PI;
inline constexpr double kRadPerDeg = M_PI / 180.0;

inline double deg_to_rad(double deg) { return deg * kRadPerDeg; }
inline double rad_to_deg(double rad) { return rad * kDegPerRad; }

// UDP joint_speed field is reported in units of deg/s (already scaled by the
// SDK from its 0.01 deg/s wire precision).
inline double deg_s_to_rad_s(double deg_s) { return deg_s * kRadPerDeg; }

// ---------------------------------------------------------------------------
// Lift pole. Units are 1:1 with MoveIt: HW "mm" ARE millimetres, so the
// full 0-0.315 m stroke maps to HW 0-315 — the same number the controller
// travel is configured to (docs/TEST_PROCEDURE.md) and the same ceiling the
// URDF declares (pole.xacro: upper="0.315"). Both poles run true-mm firmware
// (V1.7.4+, right upgraded to match left 2026-08-07).
//
// DEPRECATED: the old 2/3 command / 3/2 feedback gearing (HW 0-200 x 1.5 =
// physical 0-0.3 m, bench item D.0-4) — harmonization also recovered the top
// 15 mm of stroke. The gearing is gone from this interface, not switchable:
// a rollback to a 2:3 controller would need this constant pair restored, and
// would move the pole 1.5x its commanded distance until then.
//
// MEASURED speed profile (bench_pole_profile full 7-stroke x 5-speed matrix,
// both poles, 2026-07-14 — doc/BENCHMARKS.md §3.5, raw logs bench/results/).
// The onboard profile is trapezoidal; global least-squares fit over all 70
// cells (rms 6.8 mm/s):
//   cruise velocity  v = kLiftCruiseMpsPerPct * speed%   (linear in speed%)
//   acceleration     kLiftAccelMps2 (both ramps)
//   start latency    DIRECTION-DEPENDENT (2026-07-15 directional matrix):
//                    down 194-266 ms, up 305-378 ms (+~100 ms, grows with
//                    speed%); kLiftStartLatencyS = observed max (up).
//                    Up-moves also settle ~1.5 mm short of target (all 70
//                    up-moves, both poles — approach-from-below deadband;
//                    that was 1 count under the pre-harmonization 2:3 units,
//                    and is inside the 5 mm goal tolerance either way).
//                    Velocity itself is direction-SYMMETRIC (k identical).
// The vendor-nominal "100% == 0.1 m/s" understates the hardware ~1.8x and
// was replaced by these measured constants as the source of truth.
// ---------------------------------------------------------------------------
inline constexpr double kLiftCmdUnitsPerMetre = 1000.0;              // m -> HW mm
inline constexpr double kLiftMetresPerFbUnit = 1.0 / 1000.0;         // HW mm -> m
inline constexpr double kLiftStrokeM = 0.315;                        // physical stroke
// 0.00185 = the CONVERGED matrix rows (k 1.84-1.86 at 20-50% on long
// strokes); the earlier 0.0018 rounding made commanded speeds run ~3% fast
// and pole goals finish systematically early (field sessions 2026-07-21).
//
// LOW-% VALIDITY (bench_sync 3-way ladder, 2026-07-24/25, both poles —
// BENCHMARKS §3.9): inverting the measured travel times through
// profile_travel_time gives an implied k = v/pct of 0.182 at 4%, 0.178-0.182
// at 5%, 0.182 at 9% and 0.183-0.188 at 10-19% — i.e. this constant holds to
// ~2% all the way down to 4%, NOT only above 10% as previously assumed. Only
// 2% sags (k 0.159, -14%). The cells that look off at higher percentages
// (29-32% over 50 HW mm -> 0.158-0.169, 59-100% over 80 HW mm -> 0.126-0.165)
// are the §3.5 SHORT-STROKE velocity ceiling, not a speed-mapping error.
// Consequence: the controllers' min_match_speed floor is 0.0074 (4%), and the
// residual low-% error is quantization, handled in lift_speed_percent below.
inline constexpr double kLiftCruiseMpsPerPct = 0.00185;              // measured
inline constexpr double kLiftAccelMps2 = 0.166;                      // measured
inline constexpr double kLiftStartLatencyS = 0.38;                   // measured max (up)
inline constexpr double kLiftMaxVelocity =
  kLiftCruiseMpsPerPct * 100.0;  // 0.185 m/s cruise at 100% (measured)

inline int lift_m_to_hw_mm(double metres)
{
  return static_cast<int>(std::lround(metres * kLiftCmdUnitsPerMetre));
}

inline double lift_hw_mm_to_m(int hw_mm) { return hw_mm * kLiftMetresPerFbUnit; }

// Map a desired cruise velocity (m/s) onto the SDK speed percentage [1, 100]
// through the MEASURED mapping, so a commanded velocity is what the hardware
// actually delivers at cruise (previously v/0.1*100, which under-reported the
// hardware and made 0.05 m/s commands move at 0.09 m/s).
//
// ROUNDS UP (bench_sync 3-way ladder, 2026-07-25). The SDK takes an INTEGER
// percentage, so the achievable speeds are quantized — and at the low
// percentages synchronization uses, one count is a large fraction of the
// command (at 4% it is 25%). Rounding to nearest rounded DOWN half the time,
// which makes the pole finish LATE, and late is the harmful direction: the
// pole holds up the sequence. Rounding UP finishes EARLY, which is benign —
// the device simply waits at target. Measured on the same T=4 s / 20 HW mm
// cell: pct 4 (rounded down from 4.3) -> +240 ms late; pct 5 -> -600 ms early.
// Cost elsewhere is at most one count of extra speed (<= 1% above ~40%).
inline int lift_speed_percent(double velocity_m_s)
{
  // The epsilon keeps an EXACT multiple of the cruise constant on its own
  // count: v/k for v = 5*k evaluates to 5.000000000000001 in binary floating
  // point, and a bare ceil would promote it to 6.
  const double pct = std::abs(velocity_m_s) / kLiftCruiseMpsPerPct;
  return static_cast<int>(
    std::clamp(static_cast<long>(std::ceil(pct - 1e-9)), 1L, 100L));
}

// Travel time of a symmetric trapezoidal profile: distance d at cruise v with
// acceleration a on both ramps (triangular when d is too short to cruise).
// This is the measured device model used for goal deadlines — pure math so it
// is unit-testable (accel <= 0 falls back to the naive d/v).
inline double profile_travel_time(double distance_m, double cruise_m_s, double accel_m_s2)
{
  const double d = std::abs(distance_m);
  const double v = std::abs(cruise_m_s);
  if (v < 1e-9) {return 0.0;}
  if (accel_m_s2 <= 1e-9) {return d / v;}
  const double d_ramps = v * v / accel_m_s2;  // accel + decel distance
  if (d >= d_ramps) {
    return d / v + v / accel_m_s2;            // trapezoid
  }
  return 2.0 * std::sqrt(d / accel_m_s2);     // triangular (never cruises)
}

// Inverse of profile_travel_time in its trapezoid branch: the cruise speed
// whose profile covers `distance_m` in `duration_s` given `accel_m_s2`
// (T = d/v + v/a -> v^2 - aTv + ad = 0 -> the SMALLER root is the valid
// trapezoid solution). Used for SYNCHRONIZATION: a pole goal commanded at
// this speed finishes on its trajectory's planned timeline, matching a
// concurrently executing arm trajectory of the same duration.
//   - infeasible duration (T < the 2*sqrt(d/a) triangular minimum) or zero
//     distance -> kLiftMaxVelocity (finish as fast as the hardware can)
//   - accel <= 0 -> naive d/T
// `max_speed` defaults to the lift ceiling for backward compatibility, but is
// a parameter because the same controller now drives the HAND, whose ceiling
// is a different quantity in different units (rad/s, per channel). Clamping a
// hand goal at the lift's 0.185 would silently cap every grasp.
inline double profile_speed_for_duration(
  double distance_m, double duration_s, double accel_m_s2,
  double max_speed = kLiftMaxVelocity)
{
  const double d = std::abs(distance_m);
  if (d < 1e-9 || duration_s <= 1e-9) {
    return max_speed;
  }
  if (accel_m_s2 <= 1e-9) {
    return std::min(d / duration_s, max_speed);
  }
  const double at = accel_m_s2 * duration_s;
  const double disc = at * at - 4.0 * accel_m_s2 * d;
  if (disc <= 0.0) {
    return max_speed;  // requested duration unreachable — best effort
  }
  return std::clamp((at - std::sqrt(disc)) / 2.0, 0.0, max_speed);
}

// ---------------------------------------------------------------------------
// Inspire RH56DFX-2L hand (RH56 series). SDK DOF order (rm_interface.h:2607,
// identical to the RH56 series manual register order §2.4):
//   0 little, 1 ring, 2 middle, 3 index, 4 thumb_flex (thumb_2 joint),
//   5 thumb_rot (thumb_1 joint).
// Scale: 0 = fully flexed (max_rad), hand_scale_max (default 1000) = open.
// ---------------------------------------------------------------------------
inline constexpr std::size_t kHandDof = 6;

// Per-SDK-channel joint travel in radians (URDF limits; §5.4 table).
inline constexpr std::array<double, kHandDof> kHandMaxRad = {
  1.344,     // 0 little_1
  1.344,     // 1 ring_1
  1.344,     // 2 middle_1
  1.344,     // 3 index_1
  0.5236,    // 4 thumb_2 (flex)
  1.246165,  // 5 thumb_1 (rotate)
};

// URDF joint-name suffix for each SDK channel (prefixing is per-side).
inline constexpr std::array<const char *, kHandDof> kHandChannelSuffix = {
  "little_1_joint", "ring_1_joint", "middle_1_joint",
  "index_1_joint",  "thumb_2_joint", "thumb_1_joint",
};

// The two thumb channels (4 flex, 5 rotate). exclude_thumb removes their
// joints from the URDF; the physical hand still has all six registers.
inline constexpr bool hand_channel_is_thumb(std::size_t channel)
{
  return channel == 4 || channel == 5;
}

// Suffix-based joint-name -> SDK channel lookup (mirrors
// rm_control.cpp::joint_name_to_hw_index; order of checks matters for
// thumb_2 vs thumb_1). Returns -1 when the name is not a hand joint.
inline int hand_channel_from_joint_name(const std::string & name)
{
  if (name.find("little_1") != std::string::npos) {return 0;}
  if (name.find("ring_1") != std::string::npos) {return 1;}
  if (name.find("middle_1") != std::string::npos) {return 2;}
  if (name.find("index_1") != std::string::npos) {return 3;}
  if (name.find("thumb_2") != std::string::npos) {return 4;}
  if (name.find("thumb_1") != std::string::npos) {return 5;}
  return -1;
}

// radians -> HW scale for one channel: scale_max = open (0 rad),
// 0 = fully flexed (max_rad). Clamped to the valid range.
inline int hand_rad_to_hw(double angle_rad, int channel, int scale_max = 1000)
{
  const double max_rad = kHandMaxRad[static_cast<std::size_t>(channel)];
  const double clamped = std::clamp(angle_rad, 0.0, max_rad);
  const long scaled = std::lround(clamped / max_rad * scale_max);
  return static_cast<int>(scale_max - scaled);
}

// HW scale -> radians (inverse of hand_rad_to_hw).
inline double hand_hw_to_rad(int hw_value, int channel, int scale_max = 1000)
{
  const double max_rad = kHandMaxRad[static_cast<std::size_t>(channel)];
  const int clamped = std::clamp(hw_value, 0, scale_max);
  return (1.0 - static_cast<double>(clamped) / scale_max) * max_rad;
}

// ---------------------------------------------------------------------------
// Hand SPEED_SET model (bench_sync --hand-char, BOTH hands 2026-07-27 —
// doc/BENCHMARKS.md §3.9 "HAND VERDICT").
//
//   stroke_ms = t_lat + k * span_counts / SPEED_SET       (least squares)
//
// measured over completed strokes only, with the stall of 2026-07-23 absent:
//   right  close 18 + 375*x   open 13 + 375*x
//   left   close 26 + 370*x   open  1 + 376*x
// k = 370-376 across BOTH hands and three sessions -> one hardware constant,
// not a per-hand fit. t_lat is 1-27 ms, indistinguishable from zero (the real
// command latency is the separate ~115 ms figure, sigma 15, which the
// dispatcher covers by NOT applying a lead: measured start skew already lands
// inside +/-100 ms with none).
//
// Rearranged, the law is simply "SPEED_SET is proportional to velocity":
//   v [counts/ms] = SPEED_SET / k    ->    SPEED_SET = k * v
// and with counts = rad * 1000/max_rad and ms = s/1000,
//   SPEED_SET = kHandStrokeK * v_rad_per_s / max_rad[channel].
// So, exactly as for the pole, a commanded velocity is the velocity the
// hardware delivers.
// ---------------------------------------------------------------------------
inline constexpr double kHandStrokeK = 373.0;      // measured, both hands
inline constexpr int kHandSpeedSetMin = 100;       // hardware floor
inline constexpr int kHandSpeedSetMax = 1000;      // hardware ceiling

// Velocity (rad/s) -> per-channel SPEED_SET, clamped to the hardware range.
// A request below the floor CANNOT be honoured — the hand cannot go slower —
// so the caller must treat the clamp as saturation, not as a match (see
// hand_speed_saturates_low below).
inline int hand_speed_set_from_rad_s(double rad_per_s, int channel,
  double k = kHandStrokeK)
{
  const double max_rad = kHandMaxRad[static_cast<std::size_t>(channel)];
  if (max_rad <= 0.0) {return kHandSpeedSetMax;}
  const double raw = k * std::abs(rad_per_s) / max_rad;
  return static_cast<int>(std::clamp(
      static_cast<long>(std::lround(raw)),
      static_cast<long>(kHandSpeedSetMin),
      static_cast<long>(kHandSpeedSetMax)));
}

// Inverse: the velocity a given SPEED_SET actually delivers on a channel.
inline double hand_rad_s_from_speed_set(int speed_set, int channel,
  double k = kHandStrokeK)
{
  const double max_rad = kHandMaxRad[static_cast<std::size_t>(channel)];
  return static_cast<double>(speed_set) * max_rad / k;
}

// Slowest velocity a channel can be commanded (SPEED_SET floor). Below this
// the hand saturates and finishes EARLY — benign, but it is not a match, and
// the controller reports it rather than assuming the duration held.
inline double hand_min_velocity(int channel, double k = kHandStrokeK)
{
  return hand_rad_s_from_speed_set(kHandSpeedSetMin, channel, k);
}

// Longest stroke the hand can stretch a `span_rad` motion to on `channel`
// (the SPEED_SET-floor ceiling: ~3.0 s for a full 800-count span). A goal
// whose duration exceeds this cannot be duration-matched.
inline double hand_max_stroke_time(double span_rad, int channel,
  double k = kHandStrokeK)
{
  const double v_min = hand_min_velocity(channel, k);
  return v_min > 0.0 ? std::abs(span_rad) / v_min : 0.0;
}

// ---------------------------------------------------------------------------
// RH56 Modbus register map (RH56 series manual §2.4; byte-addressed, one
// 2-byte register per DOF, SDK channel order). Accessed THROUGH the arm
// controller: end interface board RS485 = port 1, hand device id 1.
// Hardware-verified 2026-07-14 (bench_hand_modbus_rtu): reads/writes work
// after rm_set_modbus_mode(port=1); ANGLE_ACT is genuine measured feedback;
// the hand-control protocol (rm_set_hand_angle/speed/force) is EMPIRICALLY
// mutually exclusive with end-port modbus mode on controller fw 1.7.2 —
// all-modbus operation must never mix in protocol calls.
// ---------------------------------------------------------------------------
inline constexpr int kHandModbusPort = 1;      // end interface board RS485
inline constexpr int kHandRegAngleSet = 1486;  // W/R commanded angles
inline constexpr int kHandRegForceSet = 1498;  // W/R force thresholds (g)
inline constexpr int kHandRegSpeedSet = 1522;  // W/R per-DOF speeds
inline constexpr int kHandRegAngleAct = 1546;  // R measured angles
// RH56 manual PRJ-02-TS-U-001 v1.0.9 s2.4.16, confirmed by a read-only
// register scan on hardware 2026-07-31. UNIT IS GRAMS, and the value is
// SIGNED — an unloaded finger reads slightly negative.
inline constexpr int kHandRegForceAct = 1582;  // R actual force per finger (g)
inline constexpr int kHandRegCurrent  = 1594;  // R actuator current (mA)
// 6 BYTE fields, one per DOF — NOT 6 shorts. Reading them at the 2-byte
// stride the *_ACT blocks use silently interleaves ERROR with STATUS and
// returns the same value in every physical state (measured 2026-07-31).
inline constexpr int kHandRegError    = 1606;  // R per-DOF error bitfield
inline constexpr int kHandRegStatus   = 1612;  // R per-DOF state machine

// ---------------------------------------------------------------------------
// Hand mimic joints (states computed in read(); command interfaces stay
// actuated-only). Multipliers/offsets from inspire_*_hand.ros2_control.xacro.
//   thumb_3 = 1.1425 * thumb_2
//   thumb_4 = 0.7508 * thumb_3 + 0.1   (chained through thumb_3)
//   {index,middle,ring,little}_2 = 1.1169 * *_1 - 0.1865
// ---------------------------------------------------------------------------
inline constexpr double kMimicThumb3Mult = 1.1425;
inline constexpr double kMimicThumb4Mult = 0.7508;
inline constexpr double kMimicThumb4Offset = 0.1;
inline constexpr double kMimicFingerMult = 1.1169;
inline constexpr double kMimicFingerOffset = -0.1865;

inline double mimic_thumb3(double thumb2) { return kMimicThumb3Mult * thumb2; }

inline double mimic_thumb4(double thumb2)
{
  return kMimicThumb4Mult * mimic_thumb3(thumb2) + kMimicThumb4Offset;
}

inline double mimic_finger2(double finger1)
{
  return kMimicFingerMult * finger1 + kMimicFingerOffset;
}

// Velocity propagation (offsets vanish under differentiation).
inline double mimic_thumb3_vel(double thumb2_vel) { return kMimicThumb3Mult * thumb2_vel; }
inline double mimic_thumb4_vel(double thumb2_vel)
{
  return kMimicThumb4Mult * kMimicThumb3Mult * thumb2_vel;
}
inline double mimic_finger2_vel(double finger1_vel) { return kMimicFingerMult * finger1_vel; }

// ---------------------------------------------------------------------------
// Arm streaming guard rail: documented hard limit — streamed joint motion
// must not exceed 180 deg/s "or the joint will not respond"
// (develop.realman-robotics.com JSON motion config).
// ---------------------------------------------------------------------------
inline constexpr double kArmStreamMaxDegPerS = 180.0;

// Clamp a per-cycle joint delta (deg) to the streaming velocity limit.
inline double clamp_stream_delta_deg(double delta_deg, double period_s)
{
  const double max_delta = kArmStreamMaxDegPerS * period_s;
  return std::clamp(delta_deg, -max_delta, max_delta);
}

}  // namespace alix_hw

#endif  // ALIX_HW__CONVERSIONS_HPP_
