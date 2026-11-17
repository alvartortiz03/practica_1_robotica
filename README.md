# practica_1_robotica

Práctica 1 de Robótica y Automatización Inteligente (Máster en Computación y Sistemas Inteligentes, Universidad de Deusto): extracción y análisis de datos cinemáticos del robot UR3e con ROS 2 Jazzy.

El robot simulado ejecuta varias trayectorias (Joint, Lineal y Circular). Mientras tanto, un nodo suscrito al estado de las articulaciones registra la posición, la velocidad y la aceleración de cada articulación junto con su marca de tiempo, y guarda los datos de cada trayectoria por separado. Además publica cada estado calculado con un mensaje ROS 2 propio.

## Estado

| Parte | Estado |
|---|---|
| Registrador cinemático (`kinematics_recorder`) | Hecho |
| Contrato de avisos de trayectoria (`trajectory_contract.py`) | Hecho |
| Ejecutor de trayectorias Joint y Lineal (`trajectory_runner`) | Hecho |
| Trayectoria Circular y fichero launch | Hecho |
| Opcional: guardar solo cuando el robot se mueve | Hecho |
| Opcional: mensaje ROS 2 propio (`KinematicState`) | Hecho |

## Estructura del repositorio

```
practica_1_robotica/
├── README.md
├── .gitignore                    # Ignora __pycache__/, *.pyc y data/
├── data/                         # CSV generados (no se suben al repositorio)
├── practica1_interfaces/         # Paquete ROS 2 de tipo ament_cmake con los mensajes propios
│   ├── msg/KinematicState.msg    # Definición del mensaje
│   ├── CMakeLists.txt            # Genera el código del mensaje (rosidl_generate_interfaces)
│   ├── package.xml               # Dependencias; lo declara paquete de interfaces
│   └── LICENSE
└── practica1/                    # Paquete ROS 2 de tipo ament_python
    ├── package.xml               # Nombre, versión y dependencias del paquete
    ├── setup.py                  # Instalación; en entry_points se registran los ejecutables
    ├── setup.cfg                 # Dónde se instalan los ejecutables para que ros2 run los encuentre
    ├── resource/practica1        # Marcador vacío para que ROS 2 encuentre el paquete
    ├── launch/                   # practica1.launch.py: arranca el registrador y el ejecutor
    ├── test/                     # Tests de estilo (flake8, pep257) y de copyright
    └── practica1/                # Módulo Python con el código
        ├── __init__.py
        ├── kinematics_recorder.py   # Nodo registrador
        ├── robot_controller.py      # Control del robot con MoveIt 2 (clase ControlRobot)
        ├── trajectory_contract.py   # Topic, QoS y valores de los avisos de trayectoria
        └── trajectory_runner.py     # Nodo ejecutor de trayectorias
```

## Requisitos

- El entorno Docker de la asignatura ([ros2_mucsi_26_27](https://github.com/ignacioDeusto/ros2_mucsi_26_27)), con ROS 2 Jazzy, MoveIt 2 y la simulación del UR3e.
- Este repositorio clonado dentro de su espacio de trabajo, en `ros_workspace/src/`.

## Compilación

Dentro del contenedor:

```bash
cd ~/ros_workspace
colcon build --symlink-install --packages-up-to practica1
source install/setup.bash
```

`--packages-up-to` compila `practica1` y los paquetes de los que depende, incluido `practica1_interfaces`. Con `--packages-select practica1` no se compilaría el mensaje y el nodo no arrancaría.

Con `--symlink-install`, los cambios en ficheros `.py` se aplican sin recompilar. Sí hay que recompilar al modificar `setup.py`, `package.xml` o el `.msg`.

## Ejecución

1. Lanzar la simulación del robot (desde el host, en la carpeta del entorno):

   ```bash
   ./lab.sh sim 203        # Linux / macOS
   .\lab.bat sim 203       # Windows
   ```

   o, dentro del contenedor, `ros2 launch launcher_robots_lab_robotica sim_203.launch.py`.

2. Lanzar el registrador y el ejecutor con el fichero launch, desde la raíz de este repositorio para que los CSV se guarden en su carpeta `data/`:

   ```bash
   cd ~/ros_workspace/src/practica_1_robotica
   ros2 launch practica1 practica1.launch.py
   ```

   El robot ejecuta las trayectorias Joint, Lineal y Circular, y el registrador guarda un CSV por cada una. El ejecutor termina solo al acabar la última y el registrador sigue activo hasta que se pulse Ctrl+C.

También se pueden lanzar los dos nodos por separado, cada uno en un terminal y el registrador primero:

```bash
cd ~/ros_workspace/src/practica_1_robotica
ros2 run practica1 kinematics_recorder     # Terminal 1
ros2 run practica1 trajectory_runner       # Terminal 2
```

Para probar el registrador sin el ejecutor, se pueden simular sus avisos desde otro terminal y mover el robot desde RViz:

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
| `/kinematic_state` | `practica1_interfaces/msg/KinematicState` | Estado cinemático calculado, unos 500 por segundo (publicación) |

Funcionamiento:

- `/joint_states` tiene dos publicadores: `joint_state_broadcaster` (el brazo, unos 500 Hz) y `rg2_control_node` (la pinza). Los mensajes de la pinza se descartan.
- Los nombres de las articulaciones llegan en orden alfabético, así que cada valor se busca por el nombre de su articulación y se reordena de la base a la muñeca.
- La marca de tiempo es `header.stamp`, el instante en que se tomó la medida, y no el momento en que se recibe el mensaje.
- La posición y la velocidad vienen en el mensaje. La aceleración se calcula por diferencias finitas entre dos mensajes consecutivos: `a = (v_actual - v_anterior) / (t_actual - t_anterior)`.
- Cada segundo muestra en el log el último estado calculado.
- Mientras hay una trayectoria en curso, acumula sus muestras. Cuando termina, las guarda en un CSV propio.
- Solo guarda las muestras tomadas **con el robot en movimiento**, es decir, cuando la mayor velocidad absoluta de sus articulaciones supera `MOTION_VELOCITY_THRESHOLD` (0,001 rad/s). En simulación, el robot quieto da velocidad 0 exacta y en movimiento alcanza décimas de rad/s, así que el umbral separa ambos casos con margen sin depender de un 0 exacto, que un robot real no daría por el ruido. Los periodos en reposo se descartan: antes de arrancar, al terminar y en las pausas entre movimientos. La aceleración se sigue calculando con todos los mensajes; solo se filtra lo que se guarda.
- En el log solo se avisa cuando el robot empieza a moverse o se detiene, no con cada mensaje: con unos 500 mensajes por segundo, escribir siempre ralentiza el nodo y hace que se pierdan mensajes.
- La suscripción a `/joint_states` tiene una cola de 100 mensajes (unos 200 ms a 500 Hz). Así, si el nodo está ocupado un momento, los mensajes esperan en la cola en lugar de perderse.

### Mensaje propio `KinematicState`

El nodo publica en `/kinematic_state`, con el mensaje `practica1_interfaces/msg/KinematicState`, cada estado para el que ya tiene aceleración: unos 500 mensajes por segundo. Publica siempre, también en reposo y sin trayectoria; el campo `moving` indica si el robot se mueve. El filtro de movimiento solo afecta a lo que se guarda en los CSV.

| Campo | Tipo | Contenido |
|---|---|---|
| `header` | `std_msgs/Header` | `stamp` es el instante de la medida, copiado sin modificar de `/joint_states` |
| `joint_names` | `string[]` | Nombres de las articulaciones, de la base a la muñeca |
| `position` | `float64[]` | Posición de cada articulación (rad) |
| `velocity` | `float64[]` | Velocidad de cada articulación (rad/s) |
| `acceleration` | `float64[]` | Aceleración de cada articulación (rad/s²), por diferencias finitas |
| `trajectory` | `string` | Trayectoria en curso, o `''` si no hay ninguna |
| `moving` | `bool` | `true` si la mayor velocidad absoluta supera `MOTION_VELOCITY_THRESHOLD` |

Las listas `position`, `velocity` y `acceleration` van en paralelo con `joint_names`. Para verlo:

```bash
ros2 interface show practica1_interfaces/msg/KinematicState
ros2 topic echo /kinematic_state --once
ros2 topic hz /kinematic_state
```

### Contrato de avisos (`trajectory_contract.py`)

El ejecutor de trayectorias y el registrador se comunican por el topic `/current_trajectory`:

- **Valores**: `'joint'`, `'linear'` o `'circular'` mientras se ejecuta esa trayectoria, y `''` cuando no se ejecuta ninguna.
- **QoS**: `RELIABLE` y `TRANSIENT_LOCAL`, con profundidad 1. Así el último aviso llega también a un nodo que se suscriba más tarde. Los dos nodos importan el mismo perfil del contrato, porque con perfiles distintos podrían no conectarse.
- **Secuencia del ejecutor para cada trayectoria**: llevar el robot al punto de inicio sin avisar, publicar el nombre, ejecutar la trayectoria esperando a que termine y publicar `''`.

### Formato de los CSV

Un fichero por trayectoria en `data/`, con el nombre `<trayectoria>_<AAAA-MM-DD_hh-mm-ss>.csv`, por ejemplo `joint_2026-11-14_19-45-09.csv`. La fecha y la hora son las de España (`Europe/Madrid`), aunque el contenedor funcione en UTC.

| Columna | Unidad | Contenido |
|---|---|---|
| `time_sec` | s | Marca de tiempo de la medida (`header.stamp`, tiempo de reloj) |
| `pos_<articulación>` | rad | Posición de cada articulación |
| `vel_<articulación>` | rad/s | Velocidad de cada articulación |
| `acc_<articulación>` | rad/s² | Aceleración de cada articulación |

Las articulaciones aparecen en este orden: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`. En total son 19 columnas y una fila por cada mensaje recibido con el robot en movimiento (unas 500 por segundo). Como las muestras en reposo no se guardan, si la trayectoria tiene una pausa la columna `time_sec` da un salto en ese punto.

## Nodo `trajectory_runner`

Mueve el robot con MoveIt 2 y avisa al registrador de qué trayectoria se está ejecutando, siguiendo el contrato de avisos.

| Topic | Tipo | Uso |
|---|---|---|
| `/current_trajectory` | `std_msgs/msg/String` | Avisos de inicio y fin de trayectoria (publicación) |
| `/joint_states` | `sensor_msgs/msg/JointState` | Postura actual del robot (suscripción) |

Trayectorias que ejecuta, en este orden:

| Trayectoria | Aviso | Movimiento | Cómo se ejecuta |
|---|---|---|---|
| Joint | `'joint'` | De `POSTURA_INICIO` a `POSTURA_DESTINO_JOINT`: giran la base (0 → 1 rad), el hombro (−π/2 → −1,2 rad) y la muñeca 3 (0 → 0,5 rad) | `mover_articulaciones`: MoveIt planifica hasta la postura de destino |
| Lineal | `'linear'` | El extremo baja 10 cm en vertical (`DISTANCIA_LINEAL_M`) sin cambiar su orientación | `mover_trayectoria` con una sola pose de destino |
| Circular | `'circular'` | El extremo recorre una circunferencia horizontal de 5 cm de radio (`RADIO_CIRCULO_M`) y vuelve al punto de salida | `mover_trayectoria` con 36 poses (`PUNTOS_CIRCULO`) |

Funcionamiento:

- El nodo que mueve el robot es `ControlRobot` (`robot_controller.py`). La clase `EjecutorTrayectorias` lo usa y crea sobre él el publicador de avisos, así que todo funciona con un solo nodo.
- Antes de la primera trayectoria espera hasta 5 s (`ESPERA_REGISTRADOR_SEG`) a que haya algún nodo suscrito a `/current_trajectory`, para que el registrador no pierda el principio. Si no aparece ninguno, lo avisa en el log y continúa.
- Las tres trayectorias parten de `POSTURA_INICIO`. El robot va hasta ella sin avisar, de modo que esa aproximación no se guarda en ningún CSV.
- Cada trayectoria sigue la secuencia del contrato: publica su nombre, ejecuta el movimiento esperando a que termine y publica `''`. El aviso de fin va en un bloque `finally`, así que se publica también si el movimiento falla; de este modo el registrador cierra siempre la trayectoria.
- Las trayectorias Lineal y Circular usan el servicio `compute_cartesian_path` de MoveIt, que calcula un camino en línea recta entre poses con pasos de 1 cm como máximo (`PASO_CARTESIANO_M`). Si MoveIt no puede calcular el camino completo, el robot no se mueve y la trayectoria se da por fallida.
- El MoveIt del entorno solo tiene el planificador OMPL, que no ofrece un movimiento circular. Por eso la circunferencia se recorre como 36 tramos rectos, uno cada 10°, de menos de 1 cm cada uno. El centro se coloca a un radio de distancia del extremo, así el círculo empieza y termina donde ya está el robot.
- Cuando acaba la última trayectoria, el nodo se cierra solo.

### Fichero launch

`practica1.launch.py` arranca `kinematics_recorder` y `trajectory_runner` con una sola orden. La simulación no forma parte del launch y se lanza antes, por separado. El launch no garantiza que el registrador arranque antes que el ejecutor; de eso se encarga la espera inicial del ejecutor.

## Calidad del código

```bash
cd ~/ros_workspace
colcon test --packages-select practica1 practica1_interfaces
colcon test-result --verbose
```

En `practica1`, los tests comprueban PEP 8 con `flake8` (comillas simples, líneas de 99 caracteres como máximo y orden de imports) y el formato de los docstrings con `pep257`. En `practica1_interfaces` comprueban el formato de `CMakeLists.txt` y la validez de `package.xml`.

## Limitaciones conocidas

- **Picos de aceleración.** La aceleración calculada entre dos mensajes consecutivos (unos 2 ms) es sensible al ruido. El hardware simulado no se ejecuta en tiempo real: cuando un ciclo de control se retrasa, la velocidad publicada da un pequeño salto, y al dividirlo entre 2 ms aparecen picos de aceleración que no son reales. Calcularla sobre una ventana de 20 a 50 ms reduce mucho esos picos.
- **Paradas breves.** El filtro de movimiento decide muestra a muestra. Si una trayectoria frena casi a 0 durante un instante, por ejemplo en una esquina de un camino cartesiano, esas pocas muestras no se guardan.
- **Círculo aproximado.** La trayectoria Circular es en realidad un polígono de 36 lados. Con 5 cm de radio, se separa de la circunferencia menos de 0,2 mm.
- **El registrador no termina solo.** Al lanzar con el fichero launch, el ejecutor acaba tras la última trayectoria, pero el registrador sigue activo hasta que se pulse Ctrl+C.
