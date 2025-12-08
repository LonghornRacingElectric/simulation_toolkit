# Component Testing Requirements for C++ Suspension Model

## Overview
This document identifies the components that need to be tested in the C++ implementation to recreate the Python suspension model (`suspension.py` and `suspension_data.py`).

## Component Mapping: Python → C++

### Already Tested Components ✅
1. **Node** (`test_node.cpp`) - Basic 3D point element
2. **Beam** (`test_beam.cpp`) - Equivalent to Python `Link` (rigid link)
3. **Bellcrank** (`test_bellcrank.cpp`) - 2-element component
4. **Wishbone** (`test_wishbone.cpp`) - 2-element component  
5. **Tie** (`test_tie.cpp`) - Steering tie rod
6. **Pushrod** (`test_pushrod.cpp`) - Basic push/pull rod element
7. **Damper** (`test_damper.cpp`) - Velocity-dependent damping element
8. **CG** (`test_cg.cpp`) - Center of gravity
9. **KinPC** (`test_kin_pc.cpp`) - Kinematic pitch center
10. **KinRC** (`test_kin_rc.cpp`) - Kinematic roll center
11. **Kingpin** (`test_kingpin.cpp`) - Kingpin geometry

### Missing Test Components ❌

#### 1. **Spring** (High Priority)
- **Python**: `src/vehicle_model/suspension_model/suspension_elements/_2_elements/spring.py`
- **C++ Location**: Not found (may need to be implemented)
- **Purpose**: Force-displacement relationship (F = k * compression)
- **Key Properties**:
  - `free_length`: Unloaded spring length
  - `rate`: Spring stiffness (force per unit compression)
  - `compression`: Current compression from free length
  - `force`: Current spring force
- **Note**: C++ has `Damper` but not `Spring`. Spring is a static force element, while Damper is velocity-dependent.

#### 2. **Tire** (High Priority)
- **Python**: `src/vehicle_model/suspension_model/suspension_elements/_2_elements/tire.py`
- **C++ Location**: `kin_cpp/tertiary_elements/tire.h`
- **Purpose**: Tire geometry and orientation
- **Key Properties**:
  - Contact patch position
  - Tire center position
  - Outer/inner diameter
  - Width
  - Static toe and camber angles
  - Induced steer angle (delta)
  - Inclination angle (gamma)
- **Dependencies**: Kingpin (already tested)

#### 3. **PushPullRod** (High Priority)
- **Python**: `src/vehicle_model/suspension_model/suspension_elements/_4_elements/push_pull_rod.py`
- **C++ Location**: `kin_cpp/tertiary_elements/push_pull_rod.h`
- **Purpose**: Complete push/pull rod assembly with optional bellcrank
- **Key Components**:
  - Outboard rod (connects to suspension)
  - Spring (force element)
  - Optional inboard rod
  - Optional bellcrank
- **Key Methods**:
  - `update()`: Updates geometry based on suspension motion
  - `rod_length()`: Current rod length
  - `spring_damper_length()`: Current spring length
  - `rotate_bellcrank()`: Rotates bellcrank if present
- **Dependencies**: Bellcrank (tested), Spring (needs test), Beam/Link (tested)

#### 4. **DoubleWishbone** (High Priority)
- **Python Equivalent**: `QuarterCar` (`src/vehicle_model/suspension_model/suspension_elements/_5_elements/quarter_car.py`)
- **C++ Location**: `kin_cpp/tertiary_elements/double_wishbone.h`
- **Purpose**: Complete corner assembly (equivalent to Python QuarterCar)
- **Key Components**:
  - Upper and lower wishbones
  - Tire
  - Tie rod (steering)
  - PushPullRod
  - Kingpin
- **Key Methods**:
  - `jounce()`: Vertical wheel travel
  - `steer()`: Steering input
  - `motion_ratio()`: Spring motion ratio
  - `wheelrate()`: Effective wheel rate
  - `caster()`, `kpi()`, `toe()`, `inclination_angle()`: Geometry properties
  - `FVIC_position()`, `SVIC_position()`: Instant center positions
- **Dependencies**: Wishbone (tested), Tire (needs test), PushPullRod (needs test), Tie (tested), Kingpin (tested)

#### 5. **Stabar** (Anti-roll Bar) (Medium Priority)
- **Python**: `src/vehicle_model/suspension_model/suspension_elements/_2_elements/stabar.py`
- **C++ Location**: Not found (may need to be implemented)
- **Purpose**: Anti-roll bar connecting left and right corners
- **Key Components**:
  - Torsion bar (central element)
  - Left and right arms
  - Left and right droplinks
- **Key Properties**:
  - `torsional_stiffness`: Bar stiffness
  - `left_rotation`, `right_rotation`: Arm rotation angles
- **Key Methods**:
  - `update()`: Updates geometry based on droplink positions
- **Note**: Used in `suspension.py` for roll stiffness calculations

#### 6. **Axle** (Medium Priority)
- **Python Equivalent**: Part of `Suspension` class (front/rear axle logic)
- **C++ Location**: `kin_cpp/quaternary_elements/axle.h`
- **Purpose**: Left and right corner assembly on same axle
- **Key Components**:
  - Left DoubleWishbone
  - Right DoubleWishbone
  - CG reference
- **Key Methods**:
  - `roll()`: Axle roll motion
  - `steer()`: Steering input
  - `axle_heave()`: Axle heave motion
  - `track_width()`: Track width calculation
  - `roll_stiffness()`: Roll stiffness
- **Dependencies**: DoubleWishbone (needs test), CG (tested)

#### 7. **FullSuspension** (Low Priority - Integration Test)
- **Python Equivalent**: `Suspension` class (`src/vehicle_model/suspension_model/suspension.py`)
- **C++ Location**: `kin_cpp/quinary_elements/full_suspension.h`
- **Purpose**: Complete vehicle suspension system
- **Key Components**:
  - Front axle (Axle)
  - Rear axle (Axle)
  - CG
  - Kinematic pitch centers
- **Key Methods**:
  - `steer()`: Handwheel angle input
  - `heave()`: Vehicle heave
  - `pitch()`: Vehicle pitch
  - `roll()`: Vehicle roll
  - `heave_stiffness()`, `roll_stiffness()`, `pitch_stiffness()`: Vehicle-level stiffnesses
- **Dependencies**: Axle (needs test), CG (tested), KinPC (tested)

## Testing Priority

### Critical Path (Must Test First)
1. **Spring** - Required by PushPullRod
2. **Tire** - Required by DoubleWishbone
3. **PushPullRod** - Required by DoubleWishbone
4. **DoubleWishbone** - Core suspension corner model

### Secondary Priority
5. **Stabar** - Required for roll stiffness calculations
6. **Axle** - Required for full suspension model

### Integration Testing
7. **FullSuspension** - End-to-end integration test

## Component Dependencies Graph

```
Node (✅)
  └─> Beam/Link (✅)
      ├─> Spring (❌) ──┐
      ├─> Wishbone (✅) │
      ├─> Bellcrank (✅)│
      └─> Tie (✅)       │
                         ├─> PushPullRod (❌) ──┐
Kingpin (✅) ────────────┴─> Tire (❌)         │
                                                 ├─> DoubleWishbone (❌) ──┐
CG (✅) ─────────────────────────────────────────┘                      │
                                                                          ├─> Axle (❌) ──┐
KinPC (✅) ───────────────────────────────────────────────────────────────┘              │
                                                                                          ├─> FullSuspension (❌)
KinRC (✅) ───────────────────────────────────────────────────────────────────────────────┘

Stabar (❌) ──> (connects to DoubleWishbone via droplinks)
```

## Additional Considerations

### Missing C++ Implementations
- **Spring**: May need to be implemented if not present (check if Damper can be extended)
- **Stabar**: May need to be implemented

### Python-Specific Features Not in C++
- **SuspensionData**: YAML-based configuration loader (C++ may use different initialization)
- **MF52 Tire Model**: External tire model library (may need C++ equivalent or interface)

### Test Coverage Gaps
- Motion ratio calculations
- Wheel rate calculations
- Instant center (FVIC/SVIC) calculations
- Roll center calculations
- Pitch center calculations
- Caster, KPI, scrub radius, mechanical trail calculations
- Spring motion ratio calculations
- Stabar motion ratio calculations

## Recommended Test Structure

For each component, tests should cover:
1. **Construction/Initialization**: Verify correct setup from parameters
2. **Geometry Updates**: Test position/orientation updates
3. **Property Calculations**: Test derived properties (angles, lengths, etc.)
4. **Motion**: Test jounce, steer, roll, pitch operations
5. **Force Calculations**: Test spring forces, motion ratios (where applicable)
6. **Edge Cases**: Boundary conditions, singular configurations
7. **Integration**: Interaction with dependent components

