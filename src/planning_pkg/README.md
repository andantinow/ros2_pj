# planning_pkg (C++ only, English comments)

Owned by planning lead:
- Offline (C++ CLI):
  - Generate raceline: centerline.csv → raceline.csv with v_ref
- Online (C++ node):
  - Publish /global_raceline (nav_msgs/Path, latched) and /global_vref (Float32MultiArray)

Notes on NMPC solver codegen:
- acados official generator is Python-based (acados_template).
- To keep this package C++-only, we do NOT ship codegen here.
- Control package or a separate tooling repo should run codegen and provide C artifacts.

## Build
colcon build --packages-select planning_pkg

## Generate raceline (C++ CLI)
ros2 run planning_pkg generate_raceline --centerline_csv tracks/centerline.csv --out_csv data/raceline.csv --mu 1.0 --ax_max 4.0 --ax_min -6.0 --v_max 20.0

Input centerline CSV:
- header: x,y
- rows: closed-loop polyline in meters

Output raceline.csv columns:
- s,x,y,psi,kappa,v_ref

## Run server
ros2 run planning_pkg raceline_server --ros-args -p raceline_file:=data/raceline.csv -p frame_id:=map -p publish_vref:=true

QoS: Reliable + TransientLocal (latched), depth=1
