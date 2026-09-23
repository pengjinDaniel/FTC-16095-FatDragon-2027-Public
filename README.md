# FTC 16095 FatDragon — 2026–2027 BIOBUZZ

Official code repository for FTC Team 16095 FatDragon during the 2026–2027 BIOBUZZ season.

## Current Setup

- FTC SDK 12.0
- Java
- Android Studio
- FTCLib Command-Based framework

## Project Structure

- `subsystems/` — robot subsystems
- `commands/` — command-based robot actions
- `tests/` — test OpModes

## Current Progress

- [x] Basic mecanum drivetrain
- [x] Command-based drivetrain control
- [x] Pinpoint integration (offsets, pod directions, and position hold need robot tuning)
- [x] Limelight HIGH CELL targeting (pipeline and geometry need robot tuning)
- [x] Intake, transit, shooter, and flower controls (sensor and mechanism tuning needed)

## Notes

This repository is under active development for the 2026–2027 FTC season.

Shooter velocity control and automatic firing remain disabled until the 2027-specific
PIDF, readiness thresholds, distance tables, vision geometry, position hold, and servo
positions are measured and calibrated. Autonomous routines are not implemented yet.
