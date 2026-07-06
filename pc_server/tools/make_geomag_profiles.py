#!/usr/bin/env python3
"""Generate pc_server/geomagnetic_profiles.json for all 47 Japanese prefectures.

Geomagnetic model
-----------------
Geospatial Information Authority of Japan (GSI, 国土地理院) "磁気図2020.0年値"
approximate formulas (quadratic polynomials in latitude/longitude):

    https://vldb.gsi.go.jp/sokuchi/geomag/menu_04/index.html
    (section 「近似式から求める（偏角、伏角、全磁力、水平分力、鉛直分力）」)

    Δφ = φ - 37°N,  Δλ = λ - 138°E   (φ latitude, λ longitude, degrees)

    D2020.0 = 8°15.822′ + 18.462′Δφ - 7.726′Δλ
              + 0.007′Δφ² - 0.007′ΔφΔλ - 0.655′Δλ²            [west positive]
    I2020.0 = 51°26.559′ + 72.683′Δφ - 8.642′Δλ
              - 0.943′Δφ² - 0.142′ΔφΔλ + 0.585′Δλ²            [downward positive]
    F2020.0 = 47881.463 + 547.650Δφ - 256.043Δλ
              - 2.388Δφ² - 2.750ΔφΔλ + 5.199Δλ²               [nT]
    H2020.0 = 29855.926 - 439.613Δφ - 74.293Δλ
              - 6.703Δφ² + 7.987ΔφΔλ - 5.094Δλ²               [nT]
    Z2020.0 = 37441.791 + 1058.480Δφ - 274.371Δλ
              - 10.853Δφ² - 6.454ΔφΔλ + 9.899Δλ²              [nT]

As of 2026-06 the newest published chart on the GSI site is the 2020.0 epoch
(磁気図2025.0年値 is not yet released; https://www.gsi.go.jp/buturisokuchi/
menu03_magnetic_chart.html states 「最新版は『磁気図2020.0年値』です」), so the
2020.0 formulas are used here.

Unit conversion: D/I minute terms are summed in arcminutes and divided by 60
to get degrees; F/H/Z are computed in nT and divided by 1000 to get µT.
Declination is west-positive in Japan; declination_east_deg = -declination_west_deg.

Verification (performed 2026-06-11)
-----------------------------------
1. Reproduction of the previous geomagnetic_profiles.json reference entries
   (same coordinates, same formulas) — maximum absolute error ~5e-7, i.e.
   agreement to 6 decimal places (requirement was 3):
     kyoto (35.0116, 135.7681): D 7.884827 / I 49.331329 / H 30.879436 /
                                Z 35.927237 / F 47.368231  -> all reproduced
     osaka (34.6937, 135.5023): D 7.807517 / I 48.972184 / H 31.033943 /
                                Z 35.652765 / F 47.261827  -> all reproduced
2. GSI worked example on the formula page: Tokyo (35.68N, 139.70E) ->
   D = 7°36.5′ west; this script reproduces 7°36.5′ exactly.
3. Spot checks against the GSI grid-based calculation service
   (https://vldb.gsi.go.jp/sokuchi/geomag/menu_04/bilinear.cgi, 2020.0 grid,
   includes local magnetic anomalies that the smooth quadratic fit cannot):
     Tokyo   (35.6895,139.6917): D 7.61 vs 7.62°, I 49.62 vs 49.63°,
                                 F 46747 vs 46708 nT, H 30263 vs 30253 nT
     Sapporo (43.0642,141.3469): D 9.58 vs 9.69°, I 57.79 vs 57.53°,
                                 F 50260 vs 50192 nT, H 26800 vs 26949 nT
     Naha    (26.2124,127.6809): D 5.11 vs 5.46°, I 38.81 vs 38.09°,
                                 F 44585 vs 44783 nT, H 34932 vs 35243 nT
   Differences (<=0.72° angle, <=320 nT) are consistent with the GSI note
   that the approximation excludes local anomalies and degrades at remote
   islands, and are far inside this project's rejection tolerances
   (total ±35 %, horizontal ±45 %, inclination ±20°).

Capital coordinates
-------------------
Prefectural-capital coordinates (4 decimal places, world geodetic system) are
the prefectural-office positions derived from GSI data, cross-checked between
two independent listings that agree to within ~2 arcseconds:
  - https://www.benricho.org/chimei/latlng_data.html (decimal, WGS84)
  - https://uub.jp/pdr/s/cap_4.html (DMS, 国土地理院 sourced)
Exception: kyoto and osaka keep the exact coordinates of the previous
geomagnetic_profiles.json (city-hall positions) so that the previously
verified values of the existing/selected profiles are preserved bit-for-bit.

Usage:  python3 make_geomag_profiles.py
Writes: ../geomagnetic_profiles.json (relative to this script).
Standard library only.
"""

from __future__ import annotations

import json
from pathlib import Path

EPOCH = 2020.0
OUTPUT = Path(__file__).resolve().parent.parent / "geomagnetic_profiles.json"

SOURCE = (
    "Geospatial Information Authority of Japan geomagnetic 2020.0 approximate "
    "formulas (magnetic chart 2020.0 epoch, "
    "https://vldb.gsi.go.jp/sokuchi/geomag/menu_04/index.html). "
    "Declination in this file includes both west-positive and east-positive values."
)

# 47 prefectures in JIS X 0401 order:
# key (romaji), English name, Japanese name, capital latitude, capital longitude.
PREFECTURES: list[tuple[str, str, str, float, float]] = [
    ("hokkaido",  "Hokkaido",  "北海道", 43.0643, 141.3469),
    ("aomori",    "Aomori",    "青森",   40.8246, 140.7405),
    ("iwate",     "Iwate",     "岩手",   39.7035, 141.1527),
    ("miyagi",    "Miyagi",    "宮城",   38.2686, 140.8721),
    ("akita",     "Akita",     "秋田",   39.7186, 140.1024),
    ("yamagata",  "Yamagata",  "山形",   38.2404, 140.3637),
    ("fukushima", "Fukushima", "福島",   37.7500, 140.4678),
    ("ibaraki",   "Ibaraki",   "茨城",   36.3417, 140.4468),
    ("tochigi",   "Tochigi",   "栃木",   36.5659, 139.8836),
    ("gunma",     "Gunma",     "群馬",   36.3907, 139.0605),
    ("saitama",   "Saitama",   "埼玉",   35.8570, 139.6490),
    ("chiba",     "Chiba",     "千葉",   35.6046, 140.1232),
    ("tokyo",     "Tokyo",     "東京",   35.6895, 139.6917),
    ("kanagawa",  "Kanagawa",  "神奈川", 35.4477, 139.6425),
    ("niigata",   "Niigata",   "新潟",   37.9025, 139.0232),
    ("toyama",    "Toyama",    "富山",   36.6953, 137.2113),
    ("ishikawa",  "Ishikawa",  "石川",   36.5946, 136.6257),
    ("fukui",     "Fukui",     "福井",   36.0652, 136.2217),
    ("yamanashi", "Yamanashi", "山梨",   35.6641, 138.5685),
    ("nagano",    "Nagano",    "長野",   36.6513, 138.1809),
    ("gifu",      "Gifu",      "岐阜",   35.3912, 136.7237),
    ("shizuoka",  "Shizuoka",  "静岡",   34.9769, 138.3831),
    ("aichi",     "Aichi",     "愛知",   35.1802, 136.9066),
    ("mie",       "Mie",       "三重",   34.7303, 136.5086),
    ("shiga",     "Shiga",     "滋賀",   35.0045, 135.8686),
    # kyoto/osaka: coordinates carried over unchanged from the previous
    # geomagnetic_profiles.json (city-hall positions) for continuity.
    ("kyoto",     "Kyoto",     "京都",   35.0116, 135.7681),
    ("osaka",     "Osaka",     "大阪",   34.6937, 135.5023),
    ("hyogo",     "Hyogo",     "兵庫",   34.6913, 135.1831),
    ("nara",      "Nara",      "奈良",   34.6853, 135.8329),
    ("wakayama",  "Wakayama",  "和歌山", 34.2261, 135.1675),
    ("tottori",   "Tottori",   "鳥取",   35.5034, 134.2383),
    ("shimane",   "Shimane",   "島根",   35.4723, 133.0505),
    ("okayama",   "Okayama",   "岡山",   34.6617, 133.9350),
    ("hiroshima", "Hiroshima", "広島",   34.3966, 132.4596),
    ("yamaguchi", "Yamaguchi", "山口",   34.1860, 131.4707),
    ("tokushima", "Tokushima", "徳島",   34.0658, 134.5593),
    ("kagawa",    "Kagawa",    "香川",   34.3401, 134.0433),
    ("ehime",     "Ehime",     "愛媛",   33.8416, 132.7657),
    ("kochi",     "Kochi",     "高知",   33.5597, 133.5311),
    ("fukuoka",   "Fukuoka",   "福岡",   33.6064, 130.4180),
    ("saga",      "Saga",      "佐賀",   33.2494, 130.2988),
    ("nagasaki",  "Nagasaki",  "長崎",   32.7500, 129.8673),
    ("kumamoto",  "Kumamoto",  "熊本",   32.7898, 130.7416),
    ("oita",      "Oita",      "大分",   33.2381, 131.6126),
    ("miyazaki",  "Miyazaki",  "宮崎",   31.9110, 131.4239),
    ("kagoshima", "Kagoshima", "鹿児島", 31.5602, 130.5580),
    ("okinawa",   "Okinawa",   "沖縄",   26.2124, 127.6809),
]


def gsi_2020_geomag(lat_deg: float, lon_deg: float) -> dict[str, float]:
    """Evaluate the GSI 2020.0 approximate formulas.

    Returns declination (deg, west positive), inclination (deg, down positive)
    and horizontal/vertical/total intensity (µT).
    """
    dp = lat_deg - 37.0   # Δφ
    dl = lon_deg - 138.0  # Δλ

    # D and I: constant term degrees+arcminutes, polynomial terms in arcminutes.
    d_min = (8 * 60 + 15.822
             + 18.462 * dp - 7.726 * dl
             + 0.007 * dp * dp - 0.007 * dp * dl - 0.655 * dl * dl)
    i_min = (51 * 60 + 26.559
             + 72.683 * dp - 8.642 * dl
             - 0.943 * dp * dp - 0.142 * dp * dl + 0.585 * dl * dl)
    # F, H, Z in nT.
    f_nt = (47881.463 + 547.650 * dp - 256.043 * dl
            - 2.388 * dp * dp - 2.750 * dp * dl + 5.199 * dl * dl)
    h_nt = (29855.926 - 439.613 * dp - 74.293 * dl
            - 6.703 * dp * dp + 7.987 * dp * dl - 5.094 * dl * dl)
    z_nt = (37441.791 + 1058.480 * dp - 274.371 * dl
            - 10.853 * dp * dp - 6.454 * dp * dl + 9.899 * dl * dl)

    return {
        "declination_west_deg": d_min / 60.0,
        "inclination_deg": i_min / 60.0,
        "horizontal_uT": h_nt / 1000.0,
        "vertical_uT": z_nt / 1000.0,
        "total_uT": f_nt / 1000.0,
    }


def make_profile(label_en: str, label_ja: str, lat: float, lon: float) -> dict:
    g = gsi_2020_geomag(lat, lon)
    return {
        "label": f"{label_en} ({label_ja})",
        "latitude_deg": lat,
        "longitude_deg": lon,
        "epoch": EPOCH,
        "declination_west_deg": round(g["declination_west_deg"], 6),
        "declination_east_deg": round(-g["declination_west_deg"], 6),
        "inclination_deg": round(g["inclination_deg"], 6),
        "horizontal_uT": round(g["horizontal_uT"], 6),
        "vertical_uT": round(g["vertical_uT"], 6),
        "total_uT": round(g["total_uT"], 6),
        "total_tolerance_ratio": 0.35,
        "horizontal_tolerance_ratio": 0.45,
        "inclination_tolerance_deg": 20.0,
        "inclination_z_sign": -1.0,
        "use_for_rejection": True,
    }


def verify() -> None:
    """Assert reproduction of the previously verified kyoto/osaka values."""
    references = {
        "kyoto": (35.0116, 135.7681,
                  {"declination_west_deg": 7.884827, "inclination_deg": 49.331329,
                   "horizontal_uT": 30.879436, "vertical_uT": 35.927237,
                   "total_uT": 47.368231}),
        "osaka": (34.6937, 135.5023,
                  {"declination_west_deg": 7.807517, "inclination_deg": 48.972184,
                   "horizontal_uT": 31.033943, "vertical_uT": 35.652765,
                   "total_uT": 47.261827}),
    }
    for name, (lat, lon, expected) in references.items():
        got = gsi_2020_geomag(lat, lon)
        for key, exp in expected.items():
            err = abs(got[key] - exp)
            assert err < 5e-4, f"{name} {key}: got {got[key]:.6f}, expected {exp} (err {err:.2e})"


def main() -> None:
    verify()
    profiles = {
        key: make_profile(label_en, label_ja, lat, lon)
        for key, label_en, label_ja, lat, lon in PREFECTURES
    }
    data = {
        "selected": "osaka",
        "auto_apply": True,
        "source": SOURCE,
        "profiles": profiles,
    }
    OUTPUT.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                      encoding="utf-8")
    print(f"wrote {len(profiles)} profiles to {OUTPUT}")


if __name__ == "__main__":
    main()
