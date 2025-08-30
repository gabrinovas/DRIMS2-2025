import numpy as np
# import daice_centers
import yaml
goal_number = 5

top_die = 4

def read_coordinates():
    file = "daice_centers.yaml"

    try:
        with open(file, "r", encoding="utf-8") as f:
            datos = yaml.safe_load(f)   # Carga el YAML en un diccionario de Python

        [x0, x1, y0, y1, z0, z1] = [datos["x0"], datos["x1"], datos["y0"], datos["y1"], datos["z0"], datos["z1"]]
        

        # return datos    
    #     # Acceso a los valores
    #     print("Valores cargados:")
    #     for clave, valor in datos.items():
    #         print(f"{clave}: {valor}")

    #     # Ejemplo: acceder al color "top"
    #     top = datos["top"]
    #     # print(f"\nColor 'top': {top}")

    except FileNotFoundError:
        print(f"File {file} doesn't exists.")

def die_top_or_bottom(die, goal_number):
    return die == goal_number, die == 7 - goal_number

def rotate_180_degrees():
    rotation_matrix = np.array([[1,0,0], [0,-1,0], [0,0,-1]])
    return rotation_matrix

def rotate_x_90_degrees():
    rotation_matrix = np.array([[1,0,0], [0,0,-1], [0,1,0]])
    return rotation_matrix

def rotate_y_90_degrees():
    rotation_matrix = np.array([[0,0,1], [0, 1,0], [-1,0,0]])
    return rotation_matrix

def daice_obtention(top_die, goal_number, dies_discarded, rotate_180_matrix):
    top, bottom = die_top_or_bottom(top_die, goal_number)

    if top:
        dies_discarded = [i+1 for i in range(6)]
        return dies_discarded
        # return print("COMPLETED: We already have the goal face on top")
    elif bottom:
        dies_discarded = [i+1 for i in range(6)]
        return dies_discarded
        # return print("COMPLETED: We already have the goal face on bottom")

    else:
        dies_discarded.append(top)
        dies_discarded.append(bottom)
        return dies_discarded



def main():
    dice = [1,2,3,4,5,6]
    rotations_matrix = [rotate_180_degrees(), rotate_x_90_degrees(), rotate_y_90_degrees()]
    # [x0, x1, y0, y1, z0, z1] = read_coordinates()
    dies_discarded = []
    dies = daice_obtention(top_die, goal_number, dies_discarded, rotations_matrix[0])

    if dies_discarded == dice:
        return print("COMPLETED: We already have the goal number")
    else:
        for rotation in rotations_matrix[1:]:

            dies = daice_obtention(top_die, goal_number, dies_discarded, rotations_matrix[0])
            if dies_discarded == dice:
                return print("COMPLETED: We already have the goal number")