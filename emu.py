import gb
import random
import time
import pygame as pg
pg.init()

PIXEL_SIZE = 5

screen = pg.display.set_mode([160 * PIXEL_SIZE, 144 * PIXEL_SIZE])

FPS = 5

running = True
while running:

	for event in pg.event.get():
		if event.type == pg.QUIT:
			running = False

		if event.type == pg.KEYDOWN:
			if event.key == pg.K_ESCAPE:
				running = False

	for i in range(0, int(160 / 4)):
		for j in range(0, int(144 / 4)):
			#color = gb.palette[random.randint(0, len(gb.palette) - 1)]

			#val = random.randint(0, 255)
			#color = (val, val, val)
			color = random.choice(gb.palette)
			pg.draw.rect(screen, color, (i * PIXEL_SIZE * 4, j * PIXEL_SIZE * 4, PIXEL_SIZE * 4, PIXEL_SIZE * 4))

	pg.display.flip()

	pg.time.Clock().tick(FPS)

pg.quit()
#print(gb.palette)