# practica_1_robotica

Práctica 1 de Robótica y Automatización Inteligente (Máster en Computación y Sistemas Inteligentes, Universidad de Deusto): extracción y análisis de datos cinemáticos del robot UR3e con ROS 2 Jazzy.

El robot simulado ejecuta varias trayectorias (Joint, Lineal y Circular). Mientras tanto, un nodo suscrito al estado de las articulaciones registra la posición, la velocidad y la aceleración de cada articulación junto con su marca de tiempo, y guarda los datos de cada trayectoria por separado.

## Estado

| Parte | Estado |
|---|---|
| Registrador cinemático (`kinematics_recorder`) | Hecho |
| Contrato de avisos de trayectoria (`trajectory_contract.py`) | Hecho |
| Ejecutor de trayectorias Joint y Lineal | En desarrollo |
| Trayectoria Circular y fichero launch | En desarrollo |
| Opcional: guardar solo cuando el robot se mueve | En desarrollo |
| Opcional: mensaje ROS 2 propio | En desarrollo |

## Estructura del repositorio

```
practica_1_robotica/
├── README.md
├── .gitignore                    # Ignora __pycache__/, *.pyc y data/
├── data/                         # CSV generados (no se suben al repositorio)
└── practica1/                    # Paquete ROS 2 de tipo ament_python
    ├── package.xml               # Nombre, versión y dependencias del paquete
    ├── setup.py                  # Instalación; en entry_points se registran los ejecutables
    ├── setup.cfg                 # Dónde se instalan los ejecutables para que ros2 run los encuentre
    ├── resource/practica1        # Marcador vacío para que ROS 2 encuentre el paquete
    ├── test/                     # Tests de estilo (flake8, pep257) y de copyright
    └── practica1/                # Módulo Python con el código
        ├── __init__.py
        ├── kinematics_recorder.py   # Nodo registrador
        └── trajectory_contract.py   # Topic, QoS y valores de los avisos de trayectoria
```

## Requisitos

- El entorno Docker de la asignatura ([ros2_mucsi_26_27](https://github.com/ignacioDeusto/ros2_mucsi_26_27)), con ROS 2 Jazzy, MoveIt 2 y la simulación del UR3e.
- Este repositorio clonado dentro de su espacio de trabajo, en `ros_workspace/src/`.

## Compilación

Dentro del contenedor:

```bash
cd ~/ros_workspace
colcon build --symlink-install --packages-select practica1
source install/setup.bash
```

Con `--symlink-install`, los cambios en ficheros `.py` se aplican sin recompilar. Sí hay que recompilar al modificar `setup.py` o `package.xml`.

## Ejecución

1. Lanzar la simulación del robot (desde el host, en la carpeta del entorno):

   ```bash
   ./lab.sh sim 203        # Linux / macOS
   .\lab.bat sim 203       # Windows
   ```

   o, dentro del contenedor, `ros2 launch launcher_robots_lab_robotica sim_203.launch.py`.

2. Lanzar el registrador desde la raíz de este repositorio, para que los CSV se guarden en su carpeta `data/`:

   ```bash
   cd ~/ros_workspace/src/practica_1_robotica
   ros2 run practica1 kinematics_recorder
   ```

3. Ejecutar las trayectorias. Pendiente: el ejecutor está en desarrollo.

Mientras tanto, se pueden simular los avisos del ejecutor desde otro terminal y mover el robot desde RViz:

```bash
# Empieza una trayectoria
ros2 topic pub --once -w 1 --qos-durability transient_local --qos-reliability reliable \
  /current_trajectory std_msgs/msg/String "{data: 'joint'}"

# Termina la trayectoria
ros2 topic pub --once -w 1 --qos-durability transient_local --qos-reliability reliable \
  /current_trajectory std_msgs/msg/String "{data: ''}"
```

## Nodo `kinematics_recorder`

| Topic | Tipo | Uso |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Estado de las articulaciones (suscripción) |
| `/current_trajectory` | `std_msgs/msg/String` | Avisos de inicio y fin de trayectoria (suscripción) |

Funcionamiento:

- `/joint_states` tiene dos publicadores: `joint_state_broadcaster` (el brazo, unos 500 Hz) y `rg2_control_node` (la pinza). Los mensajes de la pinza se descartan.
- Los nombres de las articulaciones llegan en orden alfabético, así que cada valor se busca por el nombre de su articulación y se reordena de la base a la muñeca.
- La marca de tiempo es `header.stamp`, el instante en que se tomó la medida, y no el momento en que se recibe el mensaje.
- La posición y la velocidad vienen en el mensaje. La aceleración se calcula por diferencias finitas entre dos mensajes consecutivos: `a = (v_actual - v_anterior) / (t_actual - t_anterior)`.
- Cada segundo muestra en el log el último estado calculado.
- Mientras hay una trayectoria en curso, acumula sus muestras. Cuando termina, las guarda en un CSV propio.

### Contrato de avisos (`trajectory_contract.py`)

El ejecutor de trayectorias y el registrador se comunican por el topic `/current_trajectory`:

- **Valores**: `'joint'`, `'linear'` o `'circular'` mientras se ejecuta esa trayectoria, y `''` cuando no se ejecuta ninguna.
- **QoS**: `RELIABLE` y `TRANSIENT_LOCAL`, con profundidad 1. Así el último aviso llega también a un nodo que se suscriba más tarde. Los dos nodos importan el mismo perfil del contrato, porque con perfiles distintos podrían no conectarse.
- **Secuencia del ejecutor para cada trayectoria**: llevar el robot al punto de inicio sin avisar, publicar el nombre, ejecutar la trayectoria esperando a que termine y publicar `''`.

### Formato de los CSV

Un fichero por trayectoria en `data/`, con el nombre `<trayectoria>_<AAAA-MM-DD_hh-mm-ss>.csv`, por ejemplo `joint_2026-11-14_19-45-09.csv`.

| Columna | Unidad | Contenido |
|---|---|---|
| `time_sec` | s | Marca de tiempo de la medida (`header.stamp`, tiempo de reloj) |
| `pos_<articulación>` | rad | Posición de cada articulación |
| `vel_<articulación>` | rad/s | Velocidad de cada articulación |
| `acc_<articulación>` | rad/s² | Aceleración de cada articulación |

Las articulaciones aparecen en este orden: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`. En total son 19 columnas y una fila por mensaje recibido (unas 500 por segundo).

## Calidad del código

```bash
cd ~/ros_workspace
colcon test --packages-select practica1
colcon test-result --verbose
```

Los tests comprueban PEP 8 con `flake8` (comillas simples, líneas de 99 caracteres como máximo y orden de imports) y el formato de los docstrings con `pep257`.

## Limitaciones conocidas

La aceleración calculada entre dos mensajes consecutivos (unos 2 ms) es sensible al ruido. El hardware simulado no se ejecuta en tiempo real: cuando un ciclo de control se retrasa, la velocidad publicada da un pequeño salto, y al dividirlo entre 2 ms aparecen picos de aceleración que no son reales. Calcularla sobre una ventana de 20 a 50 ms reduce mucho esos picos.
