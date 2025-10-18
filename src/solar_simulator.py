"""Solar irradiance simulator utilities."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import math
import random
from typing import Optional


G_SC = 1367.0  # Solar constant (W/m²)


@dataclass
class SolarConfig:
    """Configuration parameters for the solar irradiance simulator."""

    latitude_degrees: float = -33.4489
    longitude_degrees: float = -70.6693
    clearness_factor: float = 0.7
    noise_std_dev: float = 0.1


@dataclass
class SolarReading:
    """Result of a solar irradiance calculation."""

    irradiance_w_m2: float
    day_of_year: int
    solar_time_hours: float
    clearness_factor: float


class SolarIrradianceSimulator:
    """Computes horizontal plane solar irradiance for the current instant."""

    def __init__(self, config: SolarConfig):
        self.config = config
        self._last_reading: Optional[SolarReading] = None

    def compute(self, now: Optional[datetime] = None) -> SolarReading:
        """Compute irradiance for a given timestamp (defaults to current time)."""
        if now is None:
            now = datetime.now(timezone.utc).astimezone()
        elif now.tzinfo is None:
            # Assume local time if naive.
            now = now.astimezone()

        day_of_year = now.timetuple().tm_yday
        clearness_factor = self._clamp_clearness(self.config.clearness_factor)

        declination_rad = self._solar_declination(day_of_year)
        latitude_rad = math.radians(self.config.latitude_degrees)

        solar_time_hours = self._solar_time_hours(now, day_of_year)
        hour_angle_rad = math.radians(15.0 * (solar_time_hours - 12.0))

        cos_incidence = (
            math.sin(latitude_rad) * math.sin(declination_rad)
            + math.cos(latitude_rad) * math.cos(declination_rad) * math.cos(hour_angle_rad)
        )
        cos_incidence = max(0.0, cos_incidence)

        eccentricity_correction = 1.0 + 0.033 * math.cos(math.radians(360.0 * day_of_year / 365.0))
        base_irradiance = clearness_factor * G_SC * eccentricity_correction * cos_incidence
        noise_std_dev = max(0.0, self.config.noise_std_dev)
        noise_factor = random.gauss(1.0, noise_std_dev)
        irradiance = base_irradiance * noise_factor

        reading = SolarReading(
            irradiance_w_m2=max(0.0, irradiance),
            day_of_year=day_of_year,
            solar_time_hours=solar_time_hours,
            clearness_factor=clearness_factor,
        )
        self._last_reading = reading
        return reading

    def _solar_declination(self, day_of_year: int) -> float:
        """Return solar declination angle in radians for given day of year."""
        declination_deg = 23.45 * math.sin(math.radians((360.0 / 365.0) * (284 + day_of_year)))
        return math.radians(declination_deg)

    def _solar_time_hours(self, now: datetime, day_of_year: int) -> float:
        """Convert local clock time into solar time hours."""
        offset = now.utcoffset()
        tz_offset_hours = offset.total_seconds() / 3600 if offset else 0.0
        standard_meridian = 15.0 * tz_offset_hours

        b_rad = math.radians((360.0 / 365.0) * (day_of_year - 81))
        equation_of_time = 9.87 * math.sin(2 * b_rad) - 7.53 * math.cos(b_rad) - 1.5 * math.sin(b_rad)

        time_correction = 4.0 * (self.config.longitude_degrees - standard_meridian) + equation_of_time
        local_minutes = now.hour * 60.0 + now.minute + now.second / 60.0
        solar_minutes = local_minutes + time_correction
        return solar_minutes / 60.0

    @staticmethod
    def _clamp_clearness(value: float) -> float:
        """Keep clearness factor within a reasonable physical range."""
        return max(0.0, min(1.0, value))

    @property
    def last_reading(self) -> Optional[SolarReading]:
        """Return the most recent computed reading (if any)."""
        return self._last_reading
