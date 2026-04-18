# G10 — Entelect Grand Prix 2026

Race strategy solver for the Entelect University Cup hackathon. Generates optimal race plans (tyre selection, target speeds, braking points, pit stops) for an F1-inspired simulation across 4 levels of increasing complexity.

## Build

Requires **g++ with C++17** (MinGW or similar).

```bash
# Build all levels
make all

# Build a specific level
make level1
make level2
make level3
make level4
```

## Run

```bash
# Run and generate submission files
make run1   # → submission_level1.txt
make run2   # → submission_level2.txt
make run3   # → submission_level3.txt
make run4   # → submission_level4.txt

# Or run directly
./level1 inputFiles/level1input.txt > submission_level1.txt
```

Diagnostic info (scores, pit stops, tyre choices) is printed to stderr.

## Levels

| Level | Track | Laps | Key Mechanics |
|-------|-------|------|---------------|
| 1 | Neo Kyalami | 50 | Speed optimization, tyre selection |
| 2 | Silverstone | 60 | + Fuel management, pit refueling |
| 3 | Spa-Francorchamps | 70 | + Dynamic weather, tyre swaps |
| 4 | Circuit de Monaco | 80 | + Tyre degradation, multi-set pit strategy |

## Project Structure

```
src/
  types.h       Shared data types and constants
  parser.h      JSON input parsing
  physics.h     Kinematics, fuel, degradation, weather, scoring
  output.h      Submission JSON generation
  level1.cpp    Level 1 solver
  level2.cpp    Level 2 solver
  level3.cpp    Level 3 solver
  level4.cpp    Level 4 solver
inputFiles/     Level input JSON files
```

Each level is a standalone executable that includes the shared headers. Higher levels build on lower-level physics but add level-specific strategy logic.
