# TP 2

Para correr en un mundo vacio y obtener las mediciones de odometria utilizar:

```
ros2 launch TP2/tb3_empty_world/launch/tb3_simulation_launch.py headless:=False
```

Y para guardar las mediciones de odometria:

```
python3 dump_odom.py > log_square.txt
```

Para poder generar los graficos correr:

```
python3 tp2/dump_parser.py ARCHIVO_DUMP_ODOM
```

El script toma los siguientes parametros opcionales:

- `--path` grafica el camino recorrido
- `--pose` grafica la pose del robot
  - `--step n` cada cuantos puntos grafica una pose
- `--linear` grafica la velocidad lineal a lo largo del tiempo
- `--angular` grafica la velocidad angular a lo largo del tiempo
- `-x` grafica la posicion en x a lo largo del tiempo
- `-y` grafica la posicion en y a lo largo del tiempo
- `--orientation` grafica la orientacion a lo largo del tiempo
- `-p [--points] f f f...` toma una serie de porcentajes y los marca en todos los graficos. Ejemplo: `-p 0.1 0.3 0.5 0.75 0.8` este marcara los puntos de interes al 10%, 30%, 50%, 75% y 80% del camino en todos los graficos

## Detecciones de cilindros

Para correr el mundo con los cilindros y la deteccion de los mismos correr:

```
ros2 launch TP2/launch.cylinders.py headless:=False
```
