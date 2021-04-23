#!/usr/bin/env python3
"""Plot selected data fields of a TriFinger robot log.

Specify one or more fields that are to be plotted.  The names correspond to the
data entries of the ``RobotLogEntry`` class.  For fields that contain vectors
(e.g. joint positions), add a index to the name.  Example:

    status.action_repetitions
    observation.position[0]    <- plot observed positions of joint 0
"""
import argparse
import re
import sys
import typing

import matplotlib.pyplot as plt

import robot_interfaces


def get_data(
    log: robot_interfaces.trifinger.BinaryLogReader, field_name: str
) -> typing.List[typing.Any]:
    """Extract data of the given field from the robot log.

    Args:
        log: Robot log.
        field_name: Name of the data field.  Corresponds to the name of the
            attribute in the ``RobotLogEntry``, e.g.
            "status.action_repetitions" or "observation.position[0]" ([0]
            specifies to take the values of joint 0).

    Returns:
        The extracted data.
    """
    # extract index if one is given
    index = None
    m = re.match(r"(.*)\[(\d+)\]$", field_name)
    if m:
        field_name = m.group(1)
        index = int(m.group(2))

    data = log.data
    for field_component in field_name.split("."):
        data = [getattr(entry, field_component) for entry in data]

    if index is not None:
        data = [entry[index] for entry in data]

    return data


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("filename", type=str, help="The trifinger robot log file.")
    parser.add_argument(
        "fields",
        type=str,
        nargs="+",
        help="One or more data fields that are added to the plot.",
    )
    parser.add_argument(
        "--base",
        type=str,
        default="timeindex",
        help="Data field that is used for the x-axis. Default: %(default)s",
    )
    args = parser.parse_args()

    log = robot_interfaces.trifinger.BinaryLogReader(args.filename)

    base_data = get_data(log, args.base)
    for field in args.fields:
        field_data = get_data(log, field)
        plt.plot(base_data, field_data, label=field)

    plt.legend()
    plt.xlabel(args.base)
    plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
