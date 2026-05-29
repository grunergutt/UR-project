# UR-prosjekt
Dette prosjektet bruker ROS2, MoveIt og maskinsyn til å styre en UR-robot. 
Systemet detekterer røde, gule og blå kuber ved hjelp av kamera og bildebehandling,
og beregner deretter robotens målposisjoner basert på kameradata. 
Roboten beveger seg så til kubene i rekkefølgen rød, gul og blå.

<img width="300" alt="20260528_140845000_iOS" src="https://github.com/user-attachments/assets/903f29f8-d544-4162-b51e-7f9c9a85bd6f" />

---

# Innhold
| Mappe/fil | Beskrivelse |
|---|---|
| `src/cube_vision/` | Bildebehandling og koordinattransformasjon – se egen README |
| `src/robot_control/` | Robotstyring og MoveIt-integrasjon – se egen README |
| `src/ur_project_bringup/` | Launch-filer og konfigurasjon – se egen README |
