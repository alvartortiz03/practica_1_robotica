# Estado cinemático del brazo UR3e en un instante, calculado por kinematics_recorder.
#
# Las listas position, velocity y acceleration van en paralelo: el elemento i
# corresponde a la articulación joint_names[i], ordenadas de la base a la muñeca.

# Instante de la medida, copiado de la cabecera del mensaje de /joint_states.
std_msgs/Header header

# Nombres de las articulaciones, en el mismo orden que las listas de valores.
string[] joint_names

# Posición de cada articulación [rad].
float64[] position

# Velocidad de cada articulación [rad/s].
float64[] velocity

# Aceleración de cada articulación [rad/s^2], por diferencias finitas.
float64[] acceleration

# Trayectoria en curso ('joint', 'linear' o 'circular'), o cadena vacía si no hay ninguna.
string trajectory

# True si la mayor velocidad absoluta de las articulaciones supera el umbral de movimiento.
bool moving
