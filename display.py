import serial
import numpy as np
import pygame

ser = serial.Serial('COM3', 115200)

pygame.init()

GRID_SIZE = 20
SCREEN_SIZE = 1200
GRID_WIDTH = SCREEN_SIZE // GRID_SIZE
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


def receive_data():
    data = ser.readline().decode().strip()
    if data and not data[0].isdigit(): 
        print(data)

    if data:
        values = [int(val) for val in data.split(',') if val.isdigit()]
        if len(values) == GRID_SIZE * GRID_SIZE:
            array2D = np.array(values).reshape((GRID_SIZE, GRID_SIZE))
            return array2D
    return np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)

def draw_grid(array2D):
    screen.fill(BLACK)
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if array2D[y][x] > 0:
                color = WHITE
                distance = 10 - array2D[y][x]
                pygame.draw.rect(screen, color, (x*GRID_WIDTH, distance*GRID_WIDTH, GRID_WIDTH, GRID_WIDTH))
    pygame.display.flip()

screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE))
clock = pygame.time.Clock()
running = True
array2D = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            x, y = pygame.mouse.get_pos()
            x_index = x // GRID_WIDTH
            y_index = y // GRID_WIDTH

            for i in range(20):
                if array2D[i][x_index] == 10 - y_index:
                    ser.write(bytes(f"{i},{x_index}\n", 'utf-8'))

                else:
                    pass

    array2D = receive_data()
    draw_grid(array2D)
    clock.tick(30)

pygame.quit()
