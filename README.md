# UR-prosjekt

Denne koden er brukt til å styre en UR-robot og bruke maskinsyn til å til å detektere farger på kuber. 
UR-roboten skal utifra kameraresultat, flytte seg over de ulike kubene i rekkefølgen rød, gul, blå.

<img width="300" alt="20260528_140845000_iOS" src="https://github.com/user-attachments/assets/903f29f8-d544-4162-b51e-7f9c9a85bd6f" />

---

# Installasjon

Pakker som må være installert er:
- ROS2 Jazzy
- MoveIt2
- ur_robot_driver

---
# Bruk

## Surface terminal 1

Start UR-driver:

```Bash
ros2 launch ur_robot_driver ur_control.launch.py
```

## Surface terminal 2

Start MoveIt og RViz:

```Bash
ros2 launch ur_moveit_config ur_moveit.launch.py
```

## PC terminal 1

Starter kamera og kubedeteksjon:

```bash
ros2 launch cube_vision vision.launch.py
```

## PC terminal 2

Starter robotstyring:

```bash
ros2 launch ur_project_bringup project.launch.py
```

---
# Struktur

Workspacet består av tre ROS2-pakker:

- `cube_vision`
  - Kamera og bildebehandling
  - HSV-segmentering
  - Konturdeteksjon
  - Piksel -> robot-transformasjon
 
- `robot_control`
  - Tilstandsmaskin
  - Bevegelseskontroll
  - MoveIt-integrasjon
 
- `ur_project_bringup`
  - Launch-filer
  - Konfigurasjon
  - Oppstart av hele systemet

---
