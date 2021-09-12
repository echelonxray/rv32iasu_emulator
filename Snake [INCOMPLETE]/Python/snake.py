#!/usr/bin/python3

import os
import time
import threading
import random
import math

# Start Getch Impl
class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()

class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()
# End Getch Impl

class Food:
  x = 0
  y = 0

class SnakeCell:
  def __init__(self, a, b):
    self.x = a
    self.y = b

class Snake:
  new_direction = 0
  direction = 0
  cells = [ SnakeCell(0, 0), 
            SnakeCell(0, 1), 
            SnakeCell(0, 2), 
            SnakeCell(0, 3), 
            SnakeCell(0, 4) ]

global getch
global thread_gameloop
global Game_Running
global snk
global food
global term_size
global term_size_x
global term_size_y
global grid_size_x
global grid_size_y
global len_inc_amount

getch = _Getch()
thread_gameloop = None
Game_Running = False
snk = None
food = None
term_size = None
term_size_x = 0
term_size_y = 0
grid_size_x = 0
grid_size_y = 0
len_inc_amount = 0

def Game_Loop():
  while (Game_Running):
    time.sleep(0.25)
    Crawl_Snake()
    Refresh_Display()

def Refresh_Display():
  output = []
  li = 0
  while (li < term_size_y):
    output.append(" " * term_size_x)
    li += 1
  for x in snk.cells:
    output[x.y] = output[x.y][:x.x] + "+" + output[x.y][(x.x + 1):]
  output[food.y] = output[food.y][:food.x] + "x" + output[food.y][(food.x + 1):]
  print("\x1b[" + str(1) + ";" + str(1), end = "H", flush = True)
  print(output[0], end = "\x0d", flush = True)
  li = 1
  while (li < len(output)):
    print("\n" + output[li], end = "\x0d", flush = True)
    li += 1

def Place_Food(snk, food):
  free_grid_cells = (grid_size_x * grid_size_y) - len(snk.cells)
  food_index = random.randrange(1, free_grid_cells, 1) - 1
  y = 0
  while (y < grid_size_y):
    x = 0
    while (x < grid_size_x):
      for z in snk.cells:
        if (z.x == x and z.y == y):
          food_index += 1
      if (y * grid_size_x + x == food_index):
        x = grid_size_x
        y = grid_size_y
      x += 1
    y += 1
  food.y = math.floor(food_index / grid_size_x)
  food.x = math.floor(food_index % grid_size_x)

def Move_Snake(snk, x_offset, y_offset):
  for x in snk.cells:
    x.x += x_offset
    x.y += y_offset

def Crawl_Snake():
  global len_inc_amount
  snk.direction = snk.new_direction
  x_pos = snk.cells[0].x
  y_pos = snk.cells[0].y
  # Move Head Position
  if   (snk.direction == 0):
    snk.cells[0].x += 0
    snk.cells[0].y += -1
  elif (snk.direction == 1):
    snk.cells[0].x += 0
    snk.cells[0].y += 1
  elif (snk.direction == 2):
    snk.cells[0].x += -1
    snk.cells[0].y += 0
  else:
    snk.cells[0].x += 1
    snk.cells[0].y += 0
  # Wrap the head to the other side of the grid if it goes out of bounds
  if (snk.cells[0].x < 0):
    snk.cells[0].x += grid_size_x
  if (snk.cells[0].x >= grid_size_x):
    snk.cells[0].x -= grid_size_x
  if (snk.cells[0].y < 0):
    snk.cells[0].y += grid_size_y
  if (snk.cells[0].y >= grid_size_y):
    snk.cells[0].y -= grid_size_y
  # Iterate through rest of body to update other cells position
  li = 1
  while (li < len(snk.cells)):
    if (snk.cells[0].x == snk.cells[li].x and snk.cells[0].y == snk.cells[li].y):
      pass # TODO: You Lose - Snake head collided with another cell of it's body
    if (snk.cells[0].x == food.x and snk.cells[0].y == food.y):
      # Food Consume
      # Append cells to end of snake
      last_cell_x = snk.cells[len(snk.cells) - 1].x
      last_cell_y = snk.cells[len(snk.cells) - 1].y
      x = 0
      while (x < len_inc_amount):
        snk.cells.append(SnakeCell(last_cell_x, last_cell_y))
        x += 1
      len_inc_amount += 1
      # Place new food on grid
      Place_Food(snk, food)
    old_pos_x = snk.cells[li].x
    old_pos_y = snk.cells[li].y
    snk.cells[li].x = x_pos
    snk.cells[li].y = y_pos
    x_pos = old_pos_x
    y_pos = old_pos_y
    li += 1

thread_gameloop = threading.Thread(target=Game_Loop)
Game_Running = True
len_inc_amount = 3
snk = Snake()
food = Food()
term_size = os.get_terminal_size()
term_size_x = term_size.columns
term_size_y = term_size.lines
grid_size_x = term_size_x
grid_size_y = term_size_y
print(term_size)
time.sleep(1)

print("\x1b[" + "?25l", end="", flush=True)

random.seed()
Move_Snake(snk, int(term_size_x / 2), int(term_size_y / 2))
Place_Food(snk, food)

thread_gameloop.start()
while (True):
  Refresh_Display()
  inp = getch()
  if (inp == "q" or inp == "Q"):
    Game_Running = False
    thread_gameloop.join()
    print("\x1b[" + "?25h", end="", flush=True)
    quit(0)
  elif (inp == "a" or inp == "A"):
    if (snk.direction != 3):
      snk.new_direction = 2
  elif (inp == "s" or inp == "S"):
    if (snk.direction != 0):
      snk.new_direction = 1
  elif (inp == "d" or inp == "D"):
    if (snk.direction != 2):
      snk.new_direction = 3
  elif (inp == "w" or inp == "W"):
    if (snk.direction != 1):
      snk.new_direction = 0
  elif (inp == "e" or inp == "E"):
    pass
