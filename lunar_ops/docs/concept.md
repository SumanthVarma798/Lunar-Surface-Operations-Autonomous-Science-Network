# Lunar Rover – Base Concept

## What this is

A simulated lunar rover and a simulated Earth ground station.

## What the rover does

- Maintains internal state (IDLE, MOVING, ERROR)
- Publishes telemetry every few seconds
- Accepts simple commands

## What Earth does

- Sends commands
- Listens to telemetry
- Does not assume the rover is always reachable

## Assumptions

- Communication may drop
- Rover must continue safely on its own