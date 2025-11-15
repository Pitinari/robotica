# Proyecto GTSAM Graph-SLAM

Implementación de SLAM de Grafos de Poses en 2D y 3D usando la biblioteca GTSAM (Georgia Tech Smoothing and Mapping). Este proyecto implementa tanto métodos de **optimización batch** (Gauss-Newton) como **optimización incremental** (ISAM2).

## Estructura del Proyecto

```
TP4/
├── src/
│   ├── main.cpp                  # Punto de entrada principal
│   ├── g2o_reader.cpp            # Parseador de archivos G2O
│   ├── graph_slam_2d.cpp         # Implementación SLAM 2D
│   ├── graph_slam_3d.cpp         # Implementación SLAM 3D
│   └── utils/
│       └── file_utils.cpp        # Utilidades de E/S de archivos
├── include/
│   ├── g2o_reader.h
│   ├── graph_slam_2d.h
│   ├── graph_slam_3d.h
│   └── utils/
│       └── file_utils.h
├── data/
│   ├── input_INTEL_g2o.g2o      # Dataset 2D (Intel Research Lab)
│   └── parking-garage.g2o        # Dataset 3D (Parking Garage)
├── results/                      # Directorio de salida para resultados
├── plot_2d_results.py           # Script Python para visualización 2D
├── plot_3d_results.py           # Script Python para visualización 3D
└── CMakeLists.txt               # Configuración de compilación CMake
```

## Requisitos Previos

### Software Requerido

- **CMake** (>= 3.10)
- **Compilador C++** con soporte C++14 (GCC >= 4.7.3)
- **Biblioteca GTSAM** (ya compilada en `../gtsam`)
- **Python 3** con paquetes:
  - `matplotlib`
  - `pandas`
  - `numpy`

### Instalar Dependencias de Python

```bash
pip install matplotlib pandas numpy
```

## Compilar el Proyecto

La biblioteca GTSAM ya está compilada en `./gtsam/build`. Para compilar este proyecto:

```bash
cd ./TP4

# Crear directorio de compilación
mkdir -p build
cd build

# Configurar con CMake
cmake ..

# Compilar
make

```

## Ejecutar los Algoritmos

El programa soporta cuatro modos de operación:

### Uso

```bash
./build/graph_slam <modo> <archivo_g2o> <prefijo_salida>
```

**Modos:**

- `2d-batch` - Optimización batch 2D usando Gauss-Newton
- `2d-incremental` - Optimización incremental 2D usando ISAM2
- `3d-batch` - Optimización batch 3D usando Gauss-Newton
- `3d-incremental` - Optimización incremental 3D usando ISAM2

### Tarea 2.2.B - Optimización Batch 2D

Optimizar el dataset Intel usando optimización batch Gauss-Newton:

```bash
./build/graph_slam 2d-batch data/input_INTEL_g2o.g2o results/2d_batch
```

**Salida:**

- `results/2d_batch_initial.csv` - Trayectoria sin optimizar
- `results/2d_batch_optimized.csv` - Trayectoria optimizada

**Visualizar:**

```bash
python3 plot_2d_results.py results/2d_batch_initial.csv results/2d_batch_optimized.csv results/2d_batch.png "Optimización Batch 2D"
```

### Tarea 2.3.C - Optimización Incremental 2D

Optimizar el dataset Intel usando ISAM2 incremental:

```bash
./build/graph_slam 2d-incremental data/input_INTEL_g2o.g2o results/2d_incremental
```

**Salida:**

- `results/2d_incremental_initial.csv` - Trayectoria sin optimizar
- `results/2d_incremental_optimized.csv` - Trayectoria optimizada

**Visualizar:**

```bash
python3 plot_2d_results.py results/2d_incremental_initial.csv results/2d_incremental_optimized.csv results/2d_incremental.png "Optimización Incremental 2D (ISAM2)"
```

### Tarea 3.2.B - Optimización Batch 3D

Optimizar el dataset Parking Garage usando batch Gauss-Newton:

```bash
./build/graph_slam 3d-batch data/parking-garage.g2o results/3d_batch
```

**Salida:**

- `results/3d_batch_initial.csv` - Trayectoria sin optimizar
- `results/3d_batch_optimized.csv` - Trayectoria optimizada

**Visualizar:**

```bash
python3 plot_3d_results.py results/3d_batch_initial.csv results/3d_batch_optimized.csv results/3d_batch.png "Optimización Batch 3D"
```

### Tarea 3.3.C - Optimización Incremental 3D

Optimizar el dataset Parking Garage usando ISAM2 incremental:

```bash
./build/graph_slam 3d-incremental data/parking-garage.g2o results/3d_incremental
```

**Salida:**

- `results/3d_incremental_initial.csv` - Trayectoria sin optimizar
- `results/3d_incremental_optimized.csv` - Trayectoria optimizada

**Visualizar:**

```bash
python3 plot_3d_results.py results/3d_incremental_initial.csv results/3d_incremental_optimized.csv results/3d_incremental.png "Optimización Incremental 3D (ISAM2)"
```

## Archivos de Salida

### Formato CSV

**CSV de Trayectoria 2D:**

```csv
id,x,y,theta
0,0.0,0.0,0.0
1,0.5,0.1,0.05
...
```

**CSV de Trayectoria 3D:**

```csv
id,x,y,z,qx,qy,qz,qw
0,0.0,0.0,0.0,0.0,0.0,0.0,1.0
1,1.0,0.1,0.0,0.0,0.0,0.05,0.998
...
```

### Visualización

Los scripts de Python generan gráficos de calidad publicable que muestran:

- **Línea roja**: Trayectoria sin optimizar (de las estimaciones iniciales del archivo G2O)
- **Línea azul**: Trayectoria optimizada (después de la optimización SLAM)
- **Círculo verde**: Posición inicial
- **Cuadrado violeta**: Posición final
