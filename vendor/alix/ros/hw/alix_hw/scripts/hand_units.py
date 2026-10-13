#!/usr/bin/env python3
"""
hand_units.py — Inspire RH56DFX-2L hand unit converter.

Converts between the hand's 0-1000 command scale (ANGLE_SET / rm_set_hand_angle,
1000 = fully open, 0 = fully flexed, -1 = hold) and the radian joint values
used by MoveIt / the URDF. The math mirrors the system interface exactly
(alix_hw/include/alix_hw/conversions.hpp: hand_hw_to_rad /
hand_rad_to_hw and the mimic chain), so a value converted here matches what
RmSideSystem commands or echoes.

Input order = SDK/register DOF order (RH56 series manual §2.4):
    little  ring  middle  index  thumb_bend  thumb_rot

Examples
--------
  # hardware 0-1000 -> radians (default direction), right hand joint names:
  hand_units.py 1000 1000 1000 500 1000 1000 --side right

  # radians -> hardware 0-1000:
  hand_units.py 0.0 0.0 0.0 0.672 0.0 0.0 --to-hw

  # YAML snippet ready for a MoveIt named target / RViz joint publisher:
  hand_units.py 500 500 500 500 500 500 --side left --yaml

"""

import argparse
import sys

# --- Constants mirrored from alix_hw/include/alix_hw/conversions.hpp
# (source of truth; keep in sync). max_rad = URDF joint upper limits.
DOF_NAMES = ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rot')
MAX_RAD = (1.344, 1.344, 1.344, 1.344, 0.5236, 1.246165)

# SDK channel -> URDF joint-name suffix (thumb_bend = thumb_2, thumb_rot = thumb_1)
JOINT_SUFFIX = ('little_1', 'ring_1', 'middle_1', 'index_1', 'thumb_2', 'thumb_1')

# Mimic chain (matches the hand URDF / RmSideSystem read()):
#   thumb_3 = 1.1425 * thumb_2
#   thumb_4 = 0.7508 * thumb_3 + 0.1
#   {index,middle,ring,little}_2 = 1.1169 * *_1 - 0.1865
MIMIC_THUMB3_MULT = 1.1425
MIMIC_THUMB4_MULT = 0.7508
MIMIC_THUMB4_OFFSET = 0.1
MIMIC_FINGER_MULT = 1.1169
MIMIC_FINGER_OFFSET = -0.1865


def hw_to_rad(hw: float, channel: int, scale_max: int = 1000) -> float:
    """0-1000 hand scale -> radians (1000 = open = 0 rad)."""
    clamped = min(max(hw, 0), scale_max)
    return (1.0 - clamped / scale_max) * MAX_RAD[channel]


def rad_to_hw(rad: float, channel: int, scale_max: int = 1000) -> int:
    """Convert radians -> 0-1000 hand scale (0 rad = open = 1000)."""
    clamped = min(max(rad, 0.0), MAX_RAD[channel])
    return scale_max - round(clamped / MAX_RAD[channel] * scale_max)


def joint_name(side: str, channel: int) -> str:
    prefix = 'ILH_left' if side == 'left' else 'IRH_right'
    return f'{prefix}_{JOINT_SUFFIX[channel]}_joint'


def mimic_values(side: str, rads: list) -> dict:
    """
    Mimic joint values implied by the 6 actuated radians.

    For display / full joint states — MoveIt goals only need the actuated six.
    """
    prefix = 'ILH_left' if side == 'left' else 'IRH_right'
    thumb2 = rads[4]
    thumb3 = MIMIC_THUMB3_MULT * thumb2
    out = {
        f'{prefix}_thumb_3_joint': thumb3,
        f'{prefix}_thumb_4_joint': MIMIC_THUMB4_MULT * thumb3 + MIMIC_THUMB4_OFFSET,
    }
    for channel, finger in ((0, 'little'), (1, 'ring'), (2, 'middle'), (3, 'index')):
        out[f'{prefix}_{finger}_2_joint'] = (
            MIMIC_FINGER_MULT * rads[channel] + MIMIC_FINGER_OFFSET)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        'values', nargs=6, type=float, metavar='V',
        help='six values in SDK order: little ring middle index thumb_bend '
             'thumb_rot (0-1000 hand scale, or radians with --to-hw; '
             '-1 = hold, passed through)')
    parser.add_argument('--to-hw', action='store_true',
                        help='convert radians -> 0-1000 (default: 0-1000 -> radians)')
    parser.add_argument('--side', choices=('left', 'right'), default='right',
                        help='joint-name prefix for the output (default: right)')
    parser.add_argument('--scale-max', type=int, default=1000,
                        help='hand scale ceiling (default 1000, per RH56 ANGLE registers)')
    parser.add_argument('--yaml', action='store_true',
                        help='emit a YAML joint map (MoveIt named-target style)')
    parser.add_argument('--mimics', action='store_true',
                        help='also print the implied mimic joint values')
    args = parser.parse_args()

    results = []  # (channel, input, output or None-for-hold)
    for channel, value in enumerate(args.values):
        if value == -1:
            results.append((channel, value, None))
        elif args.to_hw:
            results.append((channel, value, rad_to_hw(value, channel, args.scale_max)))
        else:
            results.append((channel, value, hw_to_rad(value, channel, args.scale_max)))

    if args.yaml:
        if args.to_hw:
            print('# 0-%d hand scale, SDK order (little ring middle index '
                  'thumb_bend thumb_rot)' % args.scale_max)
            print('hand_angle: [%s]' % ', '.join(
                '-1' if out is None else str(out) for _, _, out in results))
        else:
            for channel, _, out in results:
                if out is not None:
                    print(f'{joint_name(args.side, channel)}: {out:.6f}')
            if args.mimics:
                rads = [0.0 if out is None else out for _, _, out in results]
                for name, val in mimic_values(args.side, rads).items():
                    print(f'{name}: {val:.6f}  # mimic')
        return 0

    src_unit, dst_unit = (('rad', 'hw 0-%d' % args.scale_max) if args.to_hw
                          else ('hw 0-%d' % args.scale_max, 'rad'))
    print(f"{'channel':>2s}  {'dof':10s}  {'joint (' + args.side + ')':28s} "
          f"{'in [' + src_unit + ']':>16s}  {'out [' + dst_unit + ']':>16s}")
    print('-' * 80)
    for channel, value, out in results:
        out_str = 'hold (-1)' if out is None else (
            f'{out:d}' if args.to_hw else f'{out:.6f}')
        print(f'{channel:2d}  {DOF_NAMES[channel]:10s}  '
              f'{joint_name(args.side, channel):28s} {value:16.4f}  {out_str:>16s}')

    if args.mimics and not args.to_hw:
        rads = [0.0 if out is None else out for _, _, out in results]
        print('\nimplied mimic joints (follow automatically; not commanded):')
        for name, val in mimic_values(args.side, rads).items():
            print(f'  {name:30s} {val:.6f} rad')
    return 0


if __name__ == '__main__':
    sys.exit(main())
