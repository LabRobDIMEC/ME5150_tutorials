import numpy as np
from queue import PriorityQueue
import pybullet as p

def is_in_obstacle(node, obstacle_height, grid_size, lista_de_obstaculos, sub_grid_divisions=10):
    """Determina si un nodo está dentro de un obstáculo."""
    sub_grid_size = grid_size / sub_grid_divisions
    for obstacle_id in lista_de_obstaculos: #recorre los obstaculos dentro de la lista entregada
        #TODO: encontrar la posición del obstáculo en el mundo.
        raise NotImplementedError("Falta implementar is_in_obstacle")
        for i in range(sub_grid_divisions):
            for j in range(sub_grid_divisions):
                sub_node_x = node[0] + i * sub_grid_size
                sub_node_y = node[1] + j * sub_grid_size
                #TODO: agregar la condicion para verificar si el nodo está dentro del obstáculo y retornar True
    return False #si no está dentro de un obstáculo, retorna False

def grid_and_obstacles(grid_size, world_size, obstacles_height, lista_de_obstaculos):
    """Genera una grilla y una lista de obstáculos en el mundo.
    Esta función ya está implementada y no es necesario modificarla. """
    grid = []
    obstacles= []
    for x in np.arange(0, world_size*2, grid_size):
        for y in np.arange(-world_size, world_size, grid_size):
            if not is_in_obstacle([x, y], obstacles_height, grid_size, lista_de_obstaculos):
                grid.append((x,y))
            else:
                obstacles.append((x,y, obstacles_height))
    return grid, obstacles


def reconstruct_path(came_from, current):
    """ Reconstruye el camino desde el nodo inicial hasta el nodo actual.
    Esta función ya está implementada y no es necesario modificarla."""
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse() #invertir el camino para que vaya desde el inicio hasta el final
    return path


def get_neighbors(node, grid, obstacles, grid_size, height_z):
    """" Obtiene los vecinos de un nodo en la grilla.
    Esta función ya está implementada y no es necesario modificarla."""
    directions = [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)]  # Movimientos en 2D 
    scaled_directions = [(d[0] * grid_size, d[1] * grid_size, d[2] * grid_size) for d in directions]
    
    neighbors = []
    
    for direction in scaled_directions:
        neighbor = (node[0] + direction[0], node[1] + direction[1], height_z + direction[2])
        
        is_obstacle = False
        for obs in obstacles:
            if abs(neighbor[0] - obs[0]) < 0.09 and abs(neighbor[1] - obs[1]) < 0.09 and abs(neighbor[2] - obs[2]) < 0.09:#print("Obstáculo:", obs)
                is_obstacle = True
                break
        
        if not is_obstacle:
            for cell in grid:
                if abs(neighbor[0] - cell[0]) < 0.09 and abs(neighbor[1] - cell[1]) < 0.09 and abs(neighbor[2] - height_z) < 0.09:
                    neighbors.append(neighbor)
                    break

    return neighbors

def a_star_with_obstacles(start, goal, grid, obstacles, grid_size, obstacles_height):
    """ aplica el algoritmo de A* para encontrar el camino más 
    corto entre dos puntos en un entorno con obstáculos."""
    """" los inputs de esta función podrian modificarse, si se estima necesario"""
    set_abierto = PriorityQueue()
    set_abierto.put((start, 0)) #se pone el elemento en la cola de prioridad
    came_from = {}
    g_score = {start: 0}
    raise NotImplementedError("Falta implementar a_star_with_obstacles")
    f_score = {start: np.linalg.norm(np.array(start) - np.array(goal))}
    while not set_abierto.empty():
        current,_ = set_abierto.get()
        if abs(current[0] - goal[0]) < 0.09 and abs(current[1] - goal[1]) < 0.09 and abs(current[2] - goal[2]) < 0.09:
         """ Si el nodo actual es el nodo objetivo, reconstruir el camino y retornarlo.
         se recomienta sumar un delta a la coordenada z para que el efecto no choque con la cancha"""
            
        
        """ Recorrer la lista de vecinos previamente calculada 
        aplicar la formula del algoritmo, (calcular un costo tentativo) con el metodo matematico 
        de preferencia para la heuristica
        aplicar una condición para comprobar si el nuevo costo tentativo 
        es menor que el costo previo registrado
        en caso de ser menor, se actualiza el costo y se agrega el vecino a la cola de prioridad
        """
        set_abierto.put((neighbor, f_score[neighbor]))
    raise ValueError("No se encontró un camino")

def world_to_grid(world_point, grid_size, obstacles_height):
    """
    convierte un punto del mundo de la simulación a coordenadas de la grilla.
    De ser necesario, se puede modificar esta función, para obtener mejores resultados en la simulacion
    """
    x_world, y_world, z_world = world_point
    x_grid = x_world  / 0.1  
    x_grid = round(x_grid) * grid_size  
    
    y_grid = y_world / 0.1   
    y_grid = round(y_grid) * grid_size  
    
    z_grid = z_world
    
    return (x_grid, y_grid, z_grid)


