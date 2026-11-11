"""DBC loading and signal-value generation for cyclic CAN transmission."""

import math
import random
from dataclasses import dataclass
from typing import Callable, Dict, Mapping, MutableMapping, Optional, Tuple

import cantools
from cantools.database.can import Message, Signal


@dataclass
class SignalTxSettings:
    mode: str = "raw"
    value: float = 0.0
    minimum: float = 0.0
    maximum: float = 100.0
    step: float = 1.0


def signal_value_bounds(signal: Signal) -> Tuple[float, float]:
    if (
        signal.minimum is not None
        and signal.maximum is not None
        and signal.maximum >= signal.minimum
    ):
        return float(signal.minimum), float(signal.maximum)

    if signal.is_float:
        return 0.0, 100.0

    if signal.is_signed:
        raw_minimum = -(1 << (signal.length - 1))
        raw_maximum = (1 << (signal.length - 1)) - 1
    else:
        raw_minimum = 0
        raw_maximum = (1 << signal.length) - 1
    physical_values = (
        raw_minimum * signal.scale + signal.offset,
        raw_maximum * signal.scale + signal.offset,
    )
    return float(min(physical_values)), float(max(physical_values))


def default_signal_settings(signal: Signal) -> SignalTxSettings:
    minimum, maximum = signal_value_bounds(signal)
    step = abs(float(signal.scale)) or 1.0
    initial = signal.initial
    value = float(initial) if initial is not None else 0.0
    if not minimum <= value <= maximum:
        value = minimum
    return SignalTxSettings(
        value=value,
        minimum=minimum,
        maximum=maximum,
        step=step,
    )


def encode_message(
    message: Message,
    settings: Mapping[str, SignalTxSettings],
    sweep_values: MutableMapping[str, float],
    random_value: Callable[[float, float], float] = random.uniform,
    sweep_directions: Optional[MutableMapping[str, int]] = None,
) -> bytes:
    """Encode configured physical signal values and advance generator state."""
    values: Dict[str, float] = {}
    next_sweep_values: Dict[str, float] = {}
    next_sweep_directions: Dict[str, int] = {}
    directions = sweep_directions if sweep_directions is not None else {}
    for signal in message.signals:
        config = settings[signal.name]
        minimum, maximum = signal_value_bounds(signal)
        if config.mode == "raw":
            if not math.isfinite(config.value):
                raise ValueError(f"{signal.name}: static value must be finite.")
            if not minimum <= config.value <= maximum:
                raise ValueError(
                    f"{signal.name}: value must be between {minimum:g} and {maximum:g}."
                )
            values[signal.name] = config.value
        elif config.mode == "random":
            if not math.isfinite(config.minimum) or not math.isfinite(config.maximum):
                raise ValueError(f"{signal.name}: random bounds must be finite.")
            if config.minimum > config.maximum:
                raise ValueError(f"{signal.name}: minimum must not exceed maximum.")
            if config.minimum < minimum or config.maximum > maximum:
                raise ValueError(
                    f"{signal.name}: random bounds must be between "
                    f"{minimum:g} and {maximum:g}."
                )
            values[signal.name] = random_value(config.minimum, config.maximum)
        elif config.mode == "sweep":
            if not all(
                math.isfinite(value)
                for value in (config.minimum, config.maximum, config.step)
            ):
                raise ValueError(f"{signal.name}: sweep values must be finite.")
            span = config.maximum - config.minimum
            if span <= 0 or config.step <= 0 or config.step > span:
                raise ValueError(
                    f"{signal.name}: sweep requires a positive range and a step "
                    "no larger than that range."
                )
            if config.minimum < minimum or config.maximum > maximum:
                raise ValueError(
                    f"{signal.name}: sweep bounds must be between "
                    f"{minimum:g} and {maximum:g}."
                )
            current = sweep_values.get(signal.name, config.minimum)
            if not config.minimum <= current <= config.maximum:
                current = config.minimum
            values[signal.name] = current
            direction = directions.get(signal.name, 1)
            next_value = current + direction * config.step
            next_direction = direction
            if next_value >= config.maximum:
                next_value = config.maximum
                next_direction = -1
            elif next_value <= config.minimum:
                next_value = config.minimum
                next_direction = 1
            next_sweep_values[signal.name] = next_value
            next_sweep_directions[signal.name] = next_direction
        else:
            raise ValueError(f"{signal.name}: unsupported update mode {config.mode!r}.")

    payload = message.encode(values, strict=False, padding=True)
    sweep_values.update(next_sweep_values)
    directions.update(next_sweep_directions)
    return payload


def load_dbc(path: str) -> cantools.database.Database:
    return cantools.database.load_file(path)
