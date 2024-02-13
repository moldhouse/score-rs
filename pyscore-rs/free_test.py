from typing import NamedTuple
import datetime as dt

import score_rs
import numpy as np
from numpy.testing import assert_almost_equal


class Fix(NamedTuple):
    lon: float
    lat: float
    pressure_alt: int
    time: int


def parse_line(line: str) -> Fix:
    time = ((int(line[1:3]) * 60) + int(line[3:5])) * 60 + int(line[5:7])
    lat = float(line[7:9]) + (float(line[9:11]) + float(line[11:14]) / 1000.0) / 60.0
    if line[14] == "S":
        lat = -lat

    lon = float(line[15:18]) + (float(line[18:20]) + float(line[20:23]) / 1000.0) / 60.0
    if line[23] == "W":
        lon = -lon

    pressure_alt = int(float(line[25:30]))
    return Fix(lon, lat, pressure_alt, time)


def read_igc(
    file_path: str, release: dt.time
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lon, lat, pressure_alt = [], [], []
    release_seconds_since_midnight = (
        release.hour * 60 + release.minute
    ) * 60 + release.second
    with open(file_path, "r") as file:
        for line in file.read().split("\n"):
            if line.startswith("B"):
                fix = parse_line(line)
                if fix.time < release_seconds_since_midnight:
                    continue
                lon.append(fix.lon)
                lat.append(fix.lat)
                pressure_alt.append(fix.pressure_alt)

    return np.array(lon), np.array(lat), np.array(pressure_alt)


def test_free():
    release = dt.time(8, 12, 29)
    data = read_igc("fixtures/2023-06-17_288167.igc", release)
    res = score_rs.optimize(data[0], data[1], data[2], 6)
    assert_almost_equal(res[1], 1018.54, 2)
    assert res[0] == [0, 936, 2847, 3879, 5048, 7050, 8128]

    res = score_rs.optimize(data[0], data[1], data[2], 2)
    assert_almost_equal(res[1], 804.95, 2)
    assert res[0] == [887, 3886, 7801]
