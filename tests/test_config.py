"""Tests for configuration, profiles and their JSON round trip."""

from __future__ import annotations

import json

from ffbbridge.core.config import (
    BridgeConfig,
    ModuleSettings,
    ProfileSet,
    RoutingConfig,
    SafetyConfig,
)


def test_profile_set_round_trips_through_json(tmp_path):
    profiles = ProfileSet(
        default=BridgeConfig(name="Default GA"),
        profiles=[
            BridgeConfig(
                name="Spitfire",
                match=["*spitfire*"],
                safety=SafetyConfig(master_gain=0.9),
                modules={"ground_roll": ModuleSettings(gain=1.4, params={"max_hz": 40.0})},
            )
        ],
    )
    path = tmp_path / "profiles.json"
    profiles.save(path)
    loaded = ProfileSet.load(path)

    assert loaded.default.name == "Default GA"
    assert loaded.profiles[0].safety.master_gain == 0.9
    assert loaded.profiles[0].modules["ground_roll"].params["max_hz"] == 40.0


def test_profile_matching_is_case_insensitive_and_checks_both_names():
    profile = BridgeConfig(name="Cessna", match=["*C172*"])
    assert profile.matches("Cessna Skyhawk c172 Asobo", "")
    assert profile.matches("", "C172")
    assert not profile.matches("Boeing 747", "B748")


def test_profile_without_match_patterns_never_matches():
    """The default profile is chosen by fallback, not by matching everything."""
    assert not BridgeConfig(name="Default").matches("anything", "at all")


def test_select_prefers_first_matching_profile():
    profiles = ProfileSet(
        default=BridgeConfig(name="Default"),
        profiles=[
            BridgeConfig(name="Specific", match=["*c172*"]),
            BridgeConfig(name="Generic", match=["*cessna*"]),
        ],
    )
    assert profiles.select("Cessna C172", "").name == "Specific"
    assert profiles.select("Cessna 208", "").name == "Generic"
    assert profiles.select("Airbus A320", "").name == "Default"


def test_module_defaults_are_filled_in_without_losing_overrides():
    defaults = {
        "ground_roll": ModuleSettings(enabled=True, gain=1.0, params={"max_hz": 32.0}),
        "touchdown": ModuleSettings(enabled=True, gain=1.0),
    }
    config = BridgeConfig(modules={"ground_roll": ModuleSettings(gain=1.5, params={})})
    merged = config.with_module_defaults(defaults)

    assert merged.modules["ground_roll"].gain == 1.5
    assert merged.modules["ground_roll"].params["max_hz"] == 32.0  # default preserved
    assert "touchdown" in merged.modules


def test_unknown_modules_survive_a_round_trip():
    """A profile written by a newer build must not lose settings in an older one."""
    config = BridgeConfig(modules={"from_the_future": ModuleSettings(gain=0.5)})
    merged = config.with_module_defaults({"ground_roll": ModuleSettings()})
    assert "from_the_future" in merged.modules


def test_load_or_default_survives_a_corrupt_file(tmp_path):
    path = tmp_path / "broken.json"
    path.write_text("{not json at all", encoding="utf-8")
    assert ProfileSet.load_or_default(path).default.name == "Default GA"


def test_load_or_default_survives_a_missing_file(tmp_path):
    assert ProfileSet.load_or_default(tmp_path / "nope.json").default.name == "Default GA"


def test_safety_values_are_clamped_on_load():
    safety = SafetyConfig.from_dict({"master_gain": 5.0, "max_force": -1.0, "decay_ms": 0.0})
    assert safety.master_gain == 1.0
    assert safety.max_force == 0.0
    assert safety.decay_ms >= 1.0


def test_routing_rejects_an_unknown_mode():
    assert RoutingConfig.from_dict({"mode": "nonsense"}).mode == "auto"
    assert RoutingConfig.from_dict({"mode": "rudder_only"}).mode == "rudder_only"


def test_config_dict_is_json_serialisable():
    payload = ProfileSet().to_dict()
    assert json.loads(json.dumps(payload)) == payload


def test_module_accessor_creates_settings_on_demand():
    config = BridgeConfig()
    settings = config.module("brand_new")
    settings.gain = 0.25
    assert config.modules["brand_new"].gain == 0.25


def test_soft_lock_survives_the_round_trip_and_reads_as_a_fraction():
    from ffbbridge.core.config import WheelConfig

    wheel = WheelConfig(rotation_deg=540.0, soft_lock_deg=180.0)
    assert wheel.soft_lock_fraction == 1 / 3
    assert WheelConfig.from_dict(wheel.to_dict()) == wheel
    assert WheelConfig().soft_lock_fraction == 0.0


def test_aileron_curve_survives_the_round_trip_and_is_clamped():
    from ffbbridge.core.config import AILERON_CURVE_MAX, WheelConfig

    wheel = WheelConfig(aileron_curve=0.6)
    assert WheelConfig.from_dict(wheel.to_dict()) == wheel
    assert WheelConfig().aileron_curve == 0.0
    assert WheelConfig.from_dict({"aileron_curve": AILERON_CURVE_MAX}).aileron_curve == (
        AILERON_CURVE_MAX
    )
    assert WheelConfig.from_dict({"aileron_curve": 50.0}).aileron_curve == AILERON_CURVE_MAX
    assert WheelConfig.from_dict({"aileron_curve": -5.0}).aileron_curve == 0.0

def test_saving_for_an_aircraft_beats_the_family_profile_covering_it():
    profiles = ProfileSet()
    profiles.profiles.append(BridgeConfig(name="Airliners", match=["*a320*"]))

    tuned = BridgeConfig(name="live")
    tuned.wheel.rotation_deg = 1080.0
    stored = profiles.set_for_aircraft(tuned, "Airbus A320neo Cabin", "A20N")

    assert profiles.select("Airbus A320neo Cabin", "A20N") is stored
    assert stored.wheel.rotation_deg == 1080.0
    # Another A320 that is not this one keeps the family settings.
    assert profiles.select("Airbus A320 Sharklets", "A320").name == "Airliners"


def test_saving_the_same_aircraft_twice_updates_it_in_place():
    profiles = ProfileSet()
    config = BridgeConfig()
    profiles.set_for_aircraft(config, "Cessna 172", "C172")
    config.safety.master_gain = 0.4
    profiles.set_for_aircraft(config, "Cessna 172", "C172")

    assert len(profiles.profiles) == 1
    assert profiles.profiles[0].safety.master_gain == 0.4


def test_a_stored_profile_is_a_snapshot_not_a_live_view():
    """Otherwise every slider moved afterwards edits a profile nobody chose."""
    profiles = ProfileSet()
    live = BridgeConfig()
    stored = profiles.set_for_aircraft(live, "Cessna 172", "C172")

    live.safety.master_gain = 0.1
    live.module("skid").gain = 2.0

    assert stored.safety.master_gain != 0.1
    assert stored.module("skid").gain != 2.0


def test_an_aircraft_title_with_glob_characters_still_matches_itself():
    """MSFS titles contain brackets, which fnmatch reads as a character class."""
    profiles = ProfileSet()
    title = "Cessna 152 [G1000]"
    stored = profiles.set_for_aircraft(BridgeConfig(), title, "C152")

    assert profiles.select(title, "C152") is stored
    assert profiles.select("Cessna 152 G", "C152").name != stored.name


def test_a_saved_aircraft_profile_survives_a_round_trip(tmp_path):
    profiles = ProfileSet()
    tuned = BridgeConfig()
    tuned.wheel.soft_lock_deg = 180.0
    profiles.set_for_aircraft(tuned, "Cessna 152 [G1000]", "C152")

    path = tmp_path / "profiles.json"
    profiles.save(path)
    reloaded = ProfileSet.load(path)

    assert reloaded.select("Cessna 152 [G1000]", "C152").wheel.soft_lock_deg == 180.0
