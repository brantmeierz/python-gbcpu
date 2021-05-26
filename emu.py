import gb
import random
import time
import pygame as pg
pg.init()

screen = pg.display.set_mode([160, 144])

FPS = 10

running = True
while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
    pg.display.flip()

    for i in range(0, 40):
        for j in range(0, 36):
            #color = gb.palette[random.randint(0, len(gb.palette) - 1)]

            val = random.randint(0, 255)
            color = (val, val, val)
            pg.draw.rect(screen, color, (i * 4, j * 4, 4, 4))

    pg.time.Clock().tick(FPS)

pg.quit()
#print(gb.palette)