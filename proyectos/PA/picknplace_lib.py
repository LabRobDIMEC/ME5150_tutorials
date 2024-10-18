import numpy as np
from queue import PriorityQueue
import pybullet as p

def is_in_obstacle(node,obstacle_height, grid_size, lista_de_obstaculos, sub_grid_divisions=10):
    sub_grid_size = grid_size / sub_grid_divisions
    for obstacle_id in lista_de_obstaculos:
        aabb_min, aabb_max = p.getAABB(obstacle_id)
        for i in range(sub_grid_divisions):
            for j in range(sub_grid_divisions):
                sub_node_x = node[0] + i * sub_grid_size
                sub_node_y = node[1] + j * sub_grid_size
                if (aabb_min[0] <= sub_node_x <= aabb_max[0] and
                    aabb_min[1] <= sub_node_y <= aabb_max[1] and
                    aabb_min[2] <= obstacle_height <= aabb_max[2]):
                    return True
    return False

def grid_and_obstacles(grid_size, world_size, obstacles_height, lista_de_obstaculos):
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
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def get_neighbors(node, grid, obstacles, grid_size, height_z):
    # Definir las direcciones en términos unitarios, luego escalarlas por el tamaño de la grilla
    directions = [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)]  # Movimientos en 2D y z
    scaled_directions = [(d[0] * grid_size, d[1] * grid_size, d[2] * grid_size) for d in directions]
    
    neighbors = []
    
    for direction in scaled_directions:
        neighbor = (node[0] + direction[0], node[1] + direction[1], height_z + direction[2])
        #print("Vecino:", neighbor)
        
        is_obstacle = False
        for obs in obstacles:
            if abs(neighbor[0] - obs[0]) < 0.09 and abs(neighbor[1] - obs[1]) < 0.09 and abs(neighbor[2] - obs[2]) < 0.09:#print("Obstáculo:", obs)
                is_obstacle = True
                break
        
        if not is_obstacle:
            for cell in grid:
                if abs(neighbor[0] - cell[0]) < 0.09 and abs(neighbor[1] - cell[1]) < 0.09 and abs(neighbor[2] - height_z) < 0.09:
                    neighbors.append(neighbor)
                    #print("Celda:", cell)
                    break

    return neighbors

def a_star_with_obstacles(start, goal, grid, obstacles, grid_size, obstacles_height):
    set_abierto = PriorityQueue()
    set_abierto.put((start, 0)) #se pone el elemento en la cola de prioridad
    came_from = {}
    g_score = {start: 0}
    f_score = {start: np.linalg.norm(np.array(start) - np.array(goal))}
    while not set_abierto.empty():
        current,_ = set_abierto.get()
        #print("current", current)
        if abs(current[0] - goal[0]) < 0.09 and abs(current[1] - goal[1]) < 0.09 and abs(current[2] - goal[2]) < 0.09:
            print("A* llegó")
            path= reconstruct_path(came_from, current)
            path_with_delta= [(x,y,z+0.08) for x,y,z in path]
            return path_with_delta
        for neighbor in get_neighbors(current, grid, obstacles, grid_size, obstacles_height):
            tentative_g_score = g_score[current] + np.linalg.norm(np.array(current) - np.array(neighbor))
            #print("tentative_g_score", tentative_g_score)
            if tentative_g_score < g_score.get(neighbor, np.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + np.linalg.norm(np.array(neighbor) - np.array(goal))
                set_abierto.put((neighbor, f_score[neighbor]))
    raise ValueError("No se encontró un camino")

def world_to_grid(world_point, grid_size, obstacles_height):
    """
    Convierte un punto del mundo de la simulación a coordenadas de la grilla.
    Returns:
        tuple: Coordenadas en la grilla (x, y).
    """
    x_world, y_world, z_world = world_point
    x_grid = x_world  / 0.1  
    x_grid = round(x_grid) * grid_size  
    
    y_grid = y_world / 0.1   
    y_grid = round(y_grid) * grid_size  
    
    z_grid = z_world
    
    return (x_grid, y_grid, z_grid)


