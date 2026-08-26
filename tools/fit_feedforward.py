#!/usr/bin/env python3
"""Fit V = kS*sign(v) + kV*v + kA*a from a characterization CSV."""

import argparse
import csv
import math
import sys
from pathlib import Path


def solve_3x3(matrix, vector):
    augmented = [list(row) + [value] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("data is singular; collect varying speeds and accelerations")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            scale = augmented[row][column]
            augmented[row] = [
                value - scale * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return [augmented[row][3] for row in range(3)]


def fit(rows):
    features = []
    voltages = []
    for voltage, velocity, acceleration in rows:
        if not all(math.isfinite(value) for value in (voltage, velocity, acceleration)):
            continue
        if abs(velocity) < 1e-6:
            continue
        features.append([math.copysign(1.0, velocity), velocity, acceleration])
        voltages.append(voltage)
    if len(features) < 3:
        raise ValueError("need at least three finite, non-zero-velocity samples")

    normal = [[0.0] * 3 for _ in range(3)]
    target = [0.0] * 3
    for x, voltage in zip(features, voltages):
        for i in range(3):
            target[i] += x[i] * voltage
            for j in range(3):
                normal[i][j] += x[i] * x[j]
    gains = solve_3x3(normal, target)

    mean = sum(voltages) / len(voltages)
    residual = sum(
        (voltage - sum(gain * value for gain, value in zip(gains, x))) ** 2
        for x, voltage in zip(features, voltages)
    )
    total = sum((voltage - mean) ** 2 for voltage in voltages)
    r_squared = 1.0 - residual / total if total > 1e-12 else 1.0
    return gains, r_squared, len(features)


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = {"voltage", "velocity", "acceleration"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError("CSV header must contain voltage,velocity,acceleration")
        rows = []
        for line_number, row in enumerate(reader, start=2):
            try:
                rows.append(tuple(float(row[name]) for name in
                                  ("voltage", "velocity", "acceleration")))
            except (TypeError, ValueError) as error:
                raise ValueError(f"invalid numeric value on line {line_number}") from error
        return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="characterization CSV file")
    args = parser.parse_args()
    try:
        gains, r_squared, count = fit(read_csv(args.csv))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(f"samples: {count}")
    print(f"kS: {gains[0]:.9g}")
    print(f"kV: {gains[1]:.9g}")
    print(f"kA: {gains[2]:.9g}")
    print(f"R^2: {r_squared:.6f}")


if __name__ == "__main__":
    sys.exit(main())
