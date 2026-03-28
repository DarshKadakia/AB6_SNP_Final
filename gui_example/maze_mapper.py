# """
# COORDINATE MAP
# ──────────────
# Real-time maze mapping visualizer for differential robot.

# Data source: robot_core.snp_model.RobotModel (SNP serial protocol)

# Pose:   Localization class — encoder dead reckoning (CPR=460, wheel=8cm, base=15cm)
# Walls:  ir_right / ir_left bool flags from sensors
#         Wall segments are drawn at the robot's side while IR=1
#         and stopped when IR=0. The encoder-measured travel distance
#         determines the length of each wall segment.

# Usage:
#     python maze_mapper.py --port COM7
#     python maze_mapper.py --port /dev/ttyUSB0

# Controls:  C = clear map   S = save map   P = pause   Q = quit

# Dependencies:
#     pip install pygame pyserial
# """

# import pygame
# import math
# import time
# import sys
# import threading
# import argparse
# from dataclasses import dataclass
# from typing import List, Tuple, Optional

# from snp_model import RobotModel, Localization

# # ─── Robot physical constants ────────────────────────────────────
# WHEEL_DIAMETER = 8.0    # cm
# WHEEL_BASE     = 15.0   # cm
# IR_SIDE_OFFSET = 7.5    # cm — half of 150mm wheel base

# # ─── Display config ──────────────────────────────────────────────
# WINDOW_W       = 900
# WINDOW_H       = 700
# MAP_W          = 680
# MAP_H          = 660
# SIDEBAR_W      = WINDOW_W - MAP_W
# WORLD_CM       = 300
# CM_TO_PX       = MAP_W / WORLD_CM
# WALL_THICKNESS = 5
# ROBOT_RADIUS   = 10
# PATH_DOT_R     = 2

# # ─── Colors ──────────────────────────────────────────────────────
# MAP_BG        = (26,  26,  36)
# GRID_COLOR    = (40,  40,  55)
# WALL_L_COLOR  = (255, 100, 100)
# WALL_R_COLOR  = (100, 255, 150)
# PATH_COLOR    = (60,  0, 120)
# ROBOT_COLOR   = (255, 200,  60)
# ROBOT_DIR_COL = (255, 100,  60)
# SIDEBAR_BG    = (22,  22,  30)
# TEXT_PRIMARY  = (220, 220, 230)
# TEXT_MUTED    = (120, 120, 140)
# ACCENT        = ( 80, 160, 255)
# IR_L_ON       = (255, 100, 100)
# IR_R_ON       = (100, 255, 150)

# # ─── Data structures ─────────────────────────────────────────────
# @dataclass
# class RobotState:
#     x:        float = 150.0
#     y:        float = 150.0
#     heading:  float = 0.0     # degrees
#     ir_left:  bool  = False
#     ir_right: bool  = False

# @dataclass
# class WallSegment:
#     x1:        float
#     y1:        float
#     x2:        float
#     y2:        float
#     side:      str           # 'left' or 'right'
#     length_cm: float = 0.0

# # ─── Helpers ─────────────────────────────────────────────────────
# def world_to_screen(x_cm: float, y_cm: float) -> Tuple[int, int]:
#     return int(x_cm * CM_TO_PX), int(MAP_H - y_cm * CM_TO_PX)

# def side_point(rx, ry, heading_rad, offset_cm) -> Tuple[float, float]:
#     """Project laterally from robot centre. +offset = right, -offset = left."""
#     wx = rx + math.cos(heading_rad - math.pi / 2) * offset_cm
#     wy = ry + math.sin(heading_rad - math.pi / 2) * offset_cm
#     return wx, wy

# # ─── SNP Reader ──────────────────────────────────────────────────
# class SNPReader:
#     """
#     Background thread: reads RobotModel sensors, runs Localization,
#     and converts IR boolean flags into wall segments.

#     Logic:
#       IR rising  edge (0→1) : record wall start at current side position
#       IR high    (stays 1)  : extend active wall to current side position
#       IR falling edge (1→0) : finalise and queue completed wall segment
#     """

#     def __init__(self, port: str, baud: int = 115200):
#         self._model = RobotModel("COM7")
#         self._loc   = Localization()

#         self._state      = RobotState()
#         self._state_lock = threading.Lock()

#         self._walls:     List[WallSegment] = []
#         self._walls_lock = threading.Lock()

#         self._left_start:  Optional[Tuple[float, float]] = None
#         self._right_start: Optional[Tuple[float, float]] = None
#         self._active_left:  Optional[WallSegment] = None
#         self._active_right: Optional[WallSegment] = None

#         self.running = False
#         self._thread = threading.Thread(target=self._loop, daemon=True)

#     def start(self):
#         self.running = True
#         self._thread.start()

#     def stop(self):
#         self.running = False
#         self._model.close()

#     def _loop(self):
#         prev_ir_left  = False
#         prev_ir_right = False

#         while self.running:
#             sensors = self._model.get_sensors()
#             x, y, theta = self._loc.compute_odometry(sensors)

#             # Offset to map centre
#             map_x = 150.0 + x
#             map_y = 150.0 + y

#             ir_left  = bool(sensors.ir_left)
#             ir_right = bool(sensors.ir_right)

#             # ── LEFT IR wall tracking ────────────────────────────
#             lx, ly = side_point(map_x, map_y, theta, -IR_SIDE_OFFSET)

#             if ir_left and not prev_ir_left:           # rising edge
#                 self._left_start = (lx, ly)
#                 self._active_left = WallSegment(lx, ly, lx, ly, 'left', 0.0)

#             if ir_left and self._left_start:           # extending
#                 sx, sy = self._left_start
#                 length = math.hypot(lx - sx, ly - sy)
#                 self._active_left = WallSegment(sx, sy, lx, ly, 'left', length)

#             if not ir_left and prev_ir_left and self._left_start:  # falling edge
#                 sx, sy = self._left_start
#                 length = math.hypot(lx - sx, ly - sy)
#                 with self._walls_lock:
#                     self._walls.append(WallSegment(sx, sy, lx, ly, 'left', length))
#                 print(f"[IR LEFT ] wall: {length:.1f} cm")
#                 self._left_start  = None
#                 self._active_left = None

#             # ── RIGHT IR wall tracking ───────────────────────────
#             rx, ry = side_point(map_x, map_y, theta, +IR_SIDE_OFFSET)

#             if ir_right and not prev_ir_right:
#                 self._right_start = (rx, ry)
#                 self._active_right = WallSegment(rx, ry, rx, ry, 'right', 0.0)

#             if ir_right and self._right_start:
#                 sx, sy = self._right_start
#                 length = math.hypot(rx - sx, ry - sy)
#                 self._active_right = WallSegment(sx, sy, rx, ry, 'right', length)

#             if not ir_right and prev_ir_right and self._right_start:
#                 sx, sy = self._right_start
#                 length = math.hypot(rx - sx, ry - sy)
#                 with self._walls_lock:
#                     self._walls.append(WallSegment(sx, sy, rx, ry, 'right', length))
#                 print(f"[IR RIGHT] wall: {length:.1f} cm")
#                 self._right_start  = None
#                 self._active_right = None

#             with self._state_lock:
#                 self._state = RobotState(map_x, map_y,
#                                          math.degrees(theta),
#                                          ir_left, ir_right)

#             prev_ir_left  = ir_left
#             prev_ir_right = ir_right
#             time.sleep(0.02)

#     def get_state(self) -> RobotState:
#         with self._state_lock:
#             s = self._state
#         return RobotState(s.x, s.y, s.heading, s.ir_left, s.ir_right)

#     def pop_walls(self) -> List[WallSegment]:
#         with self._walls_lock:
#             out = list(self._walls)
#             self._walls.clear()
#         return out

#     @property
#     def active_left(self):
#         return self._active_left

#     @property
#     def active_right(self):
#         return self._active_right

# # ─── Map ─────────────────────────────────────────────────────────
# class MazeMap:
#     def __init__(self):
#         self.walls: List[WallSegment]        = []
#         self.path:  List[Tuple[float,float]] = []

#     def add_path_point(self, x, y):
#         if not self.path or math.hypot(x - self.path[-1][0],
#                                        y - self.path[-1][1]) > 1.5:
#             self.path.append((x, y))

#     def add_wall(self, seg: WallSegment):
#         self.walls.append(seg)

#     def clear(self):
#         self.walls.clear()
#         self.path.clear()

# # ─── Renderer ────────────────────────────────────────────────────
# class Renderer:
#     def __init__(self, screen: pygame.Surface):
#         self.screen   = screen
#         self.map_surf = pygame.Surface((MAP_W, MAP_H))
#         self.font_lg  = pygame.font.SysFont('monospace', 15, bold=True)
#         self.font_sm  = pygame.font.SysFont('monospace', 12)
#         self.font_xs  = pygame.font.SysFont('monospace', 11)

#     def draw_grid(self):
#         self.map_surf.fill(MAP_BG)
#         step = int(20 * CM_TO_PX)
#         for x in range(0, MAP_W, step):
#             pygame.draw.line(self.map_surf, GRID_COLOR, (x, 0), (x, MAP_H), 1)
#         for y in range(0, MAP_H, step):
#             pygame.draw.line(self.map_surf, GRID_COLOR, (0, y), (MAP_W, y), 1)

#     def draw_path(self, path):
#         for x, y in path:
#             pygame.draw.circle(self.map_surf, PATH_COLOR,
#                                world_to_screen(x, y), PATH_DOT_R)
#         if len(path) > 1:
#             pts = [world_to_screen(x, y) for x, y in path]
#             pygame.draw.lines(self.map_surf, PATH_COLOR, False, pts, 1)

#     def _draw_seg(self, seg: WallSegment):
#         color = WALL_L_COLOR if seg.side == 'left' else WALL_R_COLOR
#         x1, y1 = world_to_screen(seg.x1, seg.y1)
#         x2, y2 = world_to_screen(seg.x2, seg.y2)
#         pygame.draw.line(self.map_surf, color, (x1, y1), (x2, y2), WALL_THICKNESS)
#         pygame.draw.circle(self.map_surf, color, (x1, y1), WALL_THICKNESS // 2)
#         pygame.draw.circle(self.map_surf, color, (x2, y2), WALL_THICKNESS // 2)
#         if seg.length_cm > 0.5:
#             mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
#             surf = self.font_xs.render(f"{seg.length_cm:.1f}cm", True, (255, 230, 120))
#             self.map_surf.blit(surf, (mid_x - surf.get_width() // 2,
#                                       mid_y - surf.get_height() // 2 - 8))

#     def draw_walls(self, walls, active_left, active_right):
#         for seg in walls:
#             self._draw_seg(seg)
#         if active_left:
#             self._draw_seg(active_left)
#         if active_right:
#             self._draw_seg(active_right)

#     def draw_robot(self, state: RobotState):
#         sx, sy = world_to_screen(state.x, state.y)
#         pygame.draw.circle(self.map_surf, ROBOT_COLOR, (sx, sy), ROBOT_RADIUS)
#         h = math.radians(state.heading)
#         ex = sx + int(math.cos(h) * ROBOT_RADIUS * 1.6)
#         ey = sy - int(math.sin(h) * ROBOT_RADIUS * 1.6)
#         pygame.draw.line(self.map_surf, ROBOT_DIR_COL, (sx, sy), (ex, ey), 3)
#         pygame.draw.circle(self.map_surf, ROBOT_DIR_COL, (ex, ey), 3)
#         # IR dots
#         lx, ly = world_to_screen(
#             *side_point(state.x, state.y, math.radians(state.heading), -IR_SIDE_OFFSET * 0.6))
#         pygame.draw.circle(self.map_surf,
#                            IR_L_ON if state.ir_left else (60, 60, 80), (lx, ly), 5)
#         rx, ry = world_to_screen(
#             *side_point(state.x, state.y, math.radians(state.heading), +IR_SIDE_OFFSET * 0.6))
#         pygame.draw.circle(self.map_surf,
#                            IR_R_ON if state.ir_right else (60, 60, 80), (rx, ry), 5)

#     def draw_sidebar(self, state, maze, fps, port, paused, active_left, active_right):
#         sx = MAP_W
#         self.screen.fill(SIDEBAR_BG, (sx, 0, SIDEBAR_W, WINDOW_H))

#         def label(text, y, color=TEXT_PRIMARY, font=None):
#             s = (font or self.font_sm).render(text, True, color)
#             self.screen.blit(s, (sx + 10, y))

#         def divider(y):
#             pygame.draw.line(self.screen, GRID_COLOR,
#                              (sx + 8, y), (sx + SIDEBAR_W - 8, y), 1)
#             return y + 10

#         y = 14
#         label("MAZE MAPPER",               y, ACCENT, self.font_lg); y += 28
#         label(f"Port: {port}",             y, TEXT_MUTED);           y += 18
#         label(f"FPS:  {fps:.1f}",          y, TEXT_MUTED);           y += 24
#         y = divider(y)

#         label("ROBOT POSE",                y, ACCENT);               y += 18
#         label(f"X:   {state.x:6.1f} cm",  y);                       y += 16
#         label(f"Y:   {state.y:6.1f} cm",  y);                       y += 16
#         label(f"HDG: {state.heading:6.1f} deg", y);                  y += 24
#         y = divider(y)

#         label("IR SENSORS",                y, ACCENT);               y += 18
#         col = IR_L_ON if state.ir_left  else TEXT_MUTED
#         label(f"{'[ON]' if state.ir_left  else '[--]'} IR LEFT",  y, col); y += 16
#         col = IR_R_ON if state.ir_right else TEXT_MUTED
#         label(f"{'[ON]' if state.ir_right else '[--]'} IR RIGHT", y, col); y += 24
#         y = divider(y)

#         label("LIVE WALL",                 y, ACCENT);               y += 18
#         if active_left and active_left.length_cm > 0:
#             label(f"LEFT:  {active_left.length_cm:.1f} cm",  y, IR_L_ON);  y += 16
#         else:
#             label("LEFT:  ---",            y, TEXT_MUTED);           y += 16
#         if active_right and active_right.length_cm > 0:
#             label(f"RIGHT: {active_right.length_cm:.1f} cm", y, IR_R_ON); y += 16
#         else:
#             label("RIGHT: ---",            y, TEXT_MUTED);           y += 16
#         y += 8
#         y = divider(y)

#         label("MAP STATS",                 y, ACCENT);               y += 18
#         label(f"Walls:    {len(maze.walls)}",  y);                   y += 16
#         label(f"Path pts: {len(maze.path)}",   y);                   y += 24
#         y = divider(y)

#         label("CONTROLS",                  y, ACCENT);               y += 18
#         for txt in ["[C] Clear map", "[S] Save map", "[P] Pause", "[Q] Quit"]:
#             label(txt, y, TEXT_MUTED); y += 16

#         if paused:
#             label("** PAUSED **", y + 8, (255, 100, 100), self.font_lg)

#     def render(self, state, maze, fps, port, paused, active_left, active_right):
#         self.draw_grid()
#         self.draw_path(maze.path)
#         self.draw_walls(maze.walls, active_left, active_right)
#         self.draw_robot(state)
#         self.screen.blit(self.map_surf, (0, 0))
#         self.draw_sidebar(state, maze, fps, port, paused, active_left, active_right)
#         pygame.display.flip()

# # ─── Save ────────────────────────────────────────────────────────
# def save_map(maze: MazeMap, state: RobotState):
#     fname = f"maze_map_{int(time.time())}.txt"
#     with open(fname, 'w') as f:
#         f.write("# Maze map — encoder odometry + IR wall detection\n")
#         f.write(f"# Final pose: x={state.x:.1f} y={state.y:.1f} hdg={state.heading:.1f}deg\n")
#         f.write(f"# Walls: {len(maze.walls)}\n\n")
#         f.write("# x1,y1,x2,y2,side,length_cm\n")
#         for w in maze.walls:
#             f.write(f"{w.x1:.1f},{w.y1:.1f},{w.x2:.1f},{w.y2:.1f},{w.side},{w.length_cm:.2f}\n")
#         f.write("\n# PATH: x,y\n")
#         for x, y in maze.path:
#             f.write(f"{x:.1f},{y:.1f}\n")
#     print(f"[Save] {fname}")

# # ─── Main ────────────────────────────────────────────────────────
# def main():
#     parser = argparse.ArgumentParser(description='Maze Mapper')
#     parser.add_argument('--port', type=str, default=115200, required=False,
#                         help='Serial port e.g. COM7 or /dev/ttyUSB0')
#     parser.add_argument('--baud', type=int, default=115200)
#     args = parser.parse_args()

#     pygame.init()
#     screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
#     pygame.display.set_caption("Maze Mapper — Encoder + IR")
#     clock = pygame.time.Clock()

#     reader = SNPReader(args.port, args.baud)
#     reader.start()
#     print(f"[Start] Port: {args.port} @ {args.baud}")
#     print("[Controls] C=clear  S=save  P=pause  Q=quit")

#     maze     = MazeMap()
#     renderer = Renderer(screen)
#     paused   = False
#     state    = RobotState()

#     running = True
#     while running:
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 running = False
#             elif event.type == pygame.KEYDOWN:
#                 if   event.key == pygame.K_q: running = False
#                 elif event.key == pygame.K_c: maze.clear(); print("[Clear]")
#                 elif event.key == pygame.K_s: save_map(maze, state)
#                 elif event.key == pygame.K_p:
#                     paused = not paused
#                     print(f"[{'Paused' if paused else 'Resumed'}]")

#         if not paused:
#             state = reader.get_state()
#             maze.add_path_point(state.x, state.y)
#             for seg in reader.pop_walls():
#                 maze.add_wall(seg)

#         renderer.render(state, maze, clock.get_fps(), args.port, paused,
#                         reader.active_left, reader.active_right)
#         clock.tick(30)

#     reader.stop()
#     pygame.quit()
#     sys.exit()

# if __name__ == '__main__':
#     main()













"""
GRID BASED MAP
──────────────
Real-time maze mapping visualizer for differential robot.

Data source: robot_core.snp_model.RobotModel (SNP serial protocol)

Pose:   Localization class — encoder dead reckoning (CPR=460, wheel=8cm, base=15cm)
Walls:  ir_right / ir_left bool flags from sensors
        Wall segments are drawn at the robot's side while IR=1
        and stopped when IR=0. The encoder-measured travel distance
        determines the length of each wall segment.

Usage:
    python maze_mapper.py --port COM7
    python maze_mapper.py --port /dev/ttyUSB0

Controls:  C = clear map   S = save map   P = pause   Q = quit

Dependencies:
    pip install pygame pyserial
"""

import pygame
import math
import time
import sys
import threading
import argparse
from dataclasses import dataclass
from typing import List, Tuple, Optional

from robot_core.snp_model import RobotModel, Localization

# ─── Robot physical constants ────────────────────────────────────
WHEEL_DIAMETER = 8.0    # cm
WHEEL_BASE     = 15.0   # cm
IR_SIDE_OFFSET = 7.5    # cm — half of 150mm wheel base

# ─── Display config ──────────────────────────────────────────────
WINDOW_W       = 900
WINDOW_H       = 700
MAP_W          = 680
MAP_H          = 660
SIDEBAR_W      = WINDOW_W - MAP_W
WORLD_CM       = 300
CELL_CM        = 15           # one grid cell = 15 cm (≈ robot width)
GRID_COLS      = WORLD_CM // CELL_CM   # 20 columns
GRID_ROWS      = WORLD_CM // CELL_CM   # 20 rows
CELL_PX_W      = MAP_W // GRID_COLS    # pixels per cell (width)
CELL_PX_H      = MAP_H // GRID_ROWS    # pixels per cell (height)
CM_TO_PX       = MAP_W / WORLD_CM
WALL_THICKNESS = 4
ROBOT_RADIUS   = 10
PATH_DOT_R     = 2

# ─── Colors ──────────────────────────────────────────────────────
MAP_BG        = (26,  26,  36)
GRID_COLOR    = (50,  50,  70)
CELL_VISITED  = (34,  34,  50)   # visited cell fill
WALL_L_COLOR  = (255, 100, 100)
WALL_R_COLOR  = (100, 255, 150)
WALL_EDGE_COL = (200, 200, 220)  # confirmed wall edge colour
PATH_COLOR    = (60,  0, 120)
ROBOT_COLOR   = (255, 200,  60)
ROBOT_DIR_COL = (255, 100,  60)
SIDEBAR_BG    = (22,  22,  30)
TEXT_PRIMARY  = (220, 220, 230)
TEXT_MUTED    = (120, 120, 140)
ACCENT        = ( 80, 160, 255)
IR_L_ON       = (255, 100, 100)
IR_R_ON       = (100, 255, 150)

# ─── Data structures ─────────────────────────────────────────────
@dataclass
class RobotState:
    x:        float = 150.0
    y:        float = 150.0
    heading:  float = 0.0     # degrees
    ir_left:  bool  = False
    ir_right: bool  = False

@dataclass(frozen=True, eq=True)
class GridEdge:
    """
    A wall on the boundary between two grid cells.
    col, row  — the cell on the 'low' side of the edge (0-based)
    axis      — 'h' (horizontal edge, i.e. top/bottom of a cell)
             or 'v' (vertical edge, i.e. left/right of a cell)
    side      — 'left' or 'right' (which IR sensor detected it)
    """
    col:  int
    row:  int
    axis: str   # 'h' or 'v'
    side: str   # 'left' or 'right'

# ─── Helpers ─────────────────────────────────────────────────────
def world_to_screen(x_cm: float, y_cm: float) -> Tuple[int, int]:
    return int(x_cm * CM_TO_PX), int(MAP_H - y_cm * CM_TO_PX)

def cm_to_cell(x_cm: float, y_cm: float) -> Tuple[int, int]:
    """Return (col, row) grid cell for a world coordinate."""
    col = int(x_cm / CELL_CM)
    row = int(y_cm / CELL_CM)
    col = max(0, min(GRID_COLS - 1, col))
    row = max(0, min(GRID_ROWS - 1, row))
    return col, row

def side_point(rx, ry, heading_rad, offset_cm) -> Tuple[float, float]:
    """Project laterally from robot centre. +offset = right, -offset = left."""
    wx = rx + math.cos(heading_rad - math.pi / 2) * offset_cm
    wy = ry + math.sin(heading_rad - math.pi / 2) * offset_cm
    return wx, wy

def snap_wall_edge(x_cm: float, y_cm: float, heading_rad: float,
                   side: str) -> GridEdge:
    """
    Given the side-sensor position, decide which grid edge is being sensed.
    The edge axis is determined by the robot's heading:
      moving mostly horizontally → lateral walls are vertical edges ('v')
      moving mostly vertically   → lateral walls are horizontal edges ('h')
    """
    col, row = cm_to_cell(x_cm, y_cm)
    hx = abs(math.cos(heading_rad))
    hy = abs(math.sin(heading_rad))
    axis = 'h' if hx >= hy else 'v'
    return GridEdge(col, row, axis, side)


# ─── SNP Reader ──────────────────────────────────────────────────
class SNPReader:
    """
    Background thread: reads RobotModel sensors, runs Localization,
    and converts IR boolean flags into wall segments.

    Logic:
      IR rising  edge (0→1) : record wall start at current side position
      IR high    (stays 1)  : extend active wall to current side position
      IR falling edge (1→0) : finalise and queue completed wall segment
    """

    def __init__(self, port: str, baud: int = 115200):
        self._model = RobotModel("COM7")
        self._loc   = Localization()

        self._state      = RobotState()
        self._state_lock = threading.Lock()

        self._walls:     set = set()        # set of GridEdge (deduped)
        self._walls_lock = threading.Lock()

        self._active_left:  Optional[GridEdge] = None
        self._active_right: Optional[GridEdge] = None

        self.running = False
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.running = True
        self._thread.start()

    def stop(self):
        self.running = False
        self._model.close()

    def _loop(self):
        prev_ir_left  = False
        prev_ir_right = False

        while self.running:
            sensors = self._model.get_sensors()
            x, y, theta = self._loc.compute_odometry(sensors)

            # Offset to map centre
            map_x = 150.0 + x
            map_y = 150.0 + y

            ir_left  = bool(sensors.ir_left)
            ir_right = bool(sensors.ir_right)

            # ── LEFT IR wall tracking (grid snapping) ────────────
            lx, ly = side_point(map_x, map_y, theta, -IR_SIDE_OFFSET)

            if ir_left:
                edge = snap_wall_edge(lx, ly, theta, 'left')
                with self._walls_lock:
                    self._walls.add(edge)
                self._active_left = edge
            else:
                self._active_left = None

            if not ir_left and prev_ir_left:
                print(f"[IR LEFT ] wall edge logged at cell ({snap_wall_edge(lx, ly, theta, 'left').col}, {snap_wall_edge(lx, ly, theta, 'left').row})")

            # ── RIGHT IR wall tracking (grid snapping) ───────────
            rx, ry = side_point(map_x, map_y, theta, +IR_SIDE_OFFSET)

            if ir_right:
                edge = snap_wall_edge(rx, ry, theta, 'right')
                with self._walls_lock:
                    self._walls.add(edge)
                self._active_right = edge
            else:
                self._active_right = None

            if not ir_right and prev_ir_right:
                print(f"[IR RIGHT] wall edge logged at cell ({snap_wall_edge(rx, ry, theta, 'right').col}, {snap_wall_edge(rx, ry, theta, 'right').row})")

            with self._state_lock:
                self._state = RobotState(map_x, map_y,
                                         math.degrees(theta),
                                         ir_left, ir_right)

            prev_ir_left  = ir_left
            prev_ir_right = ir_right
            time.sleep(0.02)


    def get_state(self) -> RobotState:
        with self._state_lock:
            s = self._state
        return RobotState(s.x, s.y, s.heading, s.ir_left, s.ir_right)

    def pop_walls(self) -> set:
        with self._walls_lock:
            return set(self._walls)   # return full snapshot (set is deduped)

    @property
    def active_left(self):
        return self._active_left

    @property
    def active_right(self):
        return self._active_right

# ─── Map ─────────────────────────────────────────────────────────
class MazeMap:
    def __init__(self):
        self.walls:   set                    = set()   # GridEdge objects
        self.visited: set                    = set()   # (col, row) tuples
        self.path:    List[Tuple[float,float]] = []

    def add_path_point(self, x, y):
        if not self.path or math.hypot(x - self.path[-1][0],
                                       y - self.path[-1][1]) > 1.5:
            self.path.append((x, y))
        col, row = cm_to_cell(x, y)
        self.visited.add((col, row))

    def add_walls(self, edges: set):
        self.walls.update(edges)

    def clear(self):
        self.walls.clear()
        self.visited.clear()
        self.path.clear()

# ─── Renderer ────────────────────────────────────────────────────
class Renderer:
    def __init__(self, screen: pygame.Surface):
        self.screen   = screen
        self.map_surf = pygame.Surface((MAP_W, MAP_H))
        self.font_lg  = pygame.font.SysFont('monospace', 15, bold=True)
        self.font_sm  = pygame.font.SysFont('monospace', 12)
        self.font_xs  = pygame.font.SysFont('monospace', 11)

    def _cell_rect(self, col: int, row: int) -> pygame.Rect:
        """Return the pixel Rect for a grid cell (row 0 = bottom of world)."""
        px = col * CELL_PX_W
        py = MAP_H - (row + 1) * CELL_PX_H
        return pygame.Rect(px, py, CELL_PX_W, CELL_PX_H)

    def draw_grid(self, visited: set):
        self.map_surf.fill(MAP_BG)
        # Fill visited cells
        for (col, row) in visited:
            rect = self._cell_rect(col, row)
            pygame.draw.rect(self.map_surf, CELL_VISITED, rect)
        # Grid lines
        for c in range(GRID_COLS + 1):
            x = c * CELL_PX_W
            pygame.draw.line(self.map_surf, GRID_COLOR, (x, 0), (x, MAP_H), 1)
        for r in range(GRID_ROWS + 1):
            y = r * CELL_PX_H
            pygame.draw.line(self.map_surf, GRID_COLOR, (0, y), (MAP_W, y), 1)

    def draw_path(self, path):
        for x, y in path:
            pygame.draw.circle(self.map_surf, PATH_COLOR,
                               world_to_screen(x, y), PATH_DOT_R)
        if len(path) > 1:
            pts = [world_to_screen(x, y) for x, y in path]
            pygame.draw.lines(self.map_surf, PATH_COLOR, False, pts, 1)

    def draw_walls(self, walls: set, active_left, active_right):
        active_edges = set()
        if active_left:
            active_edges.add(active_left)
        if active_right:
            active_edges.add(active_right)

        for edge in walls | active_edges:
            color = WALL_L_COLOR if edge.side == 'left' else WALL_R_COLOR
            rect  = self._cell_rect(edge.col, edge.row)
            if edge.axis == 'v':
                # Vertical edge on the right side of this cell
                x = rect.right
                pygame.draw.line(self.map_surf, color,
                                 (x, rect.top), (x, rect.bottom), WALL_THICKNESS)
            else:
                # Horizontal edge on the top side of this cell
                y = rect.top
                pygame.draw.line(self.map_surf, color,
                                 (rect.left, y), (rect.right, y), WALL_THICKNESS)

    def draw_robot(self, state: RobotState):
        sx, sy = world_to_screen(state.x, state.y)
        # Highlight current cell
        col, row = cm_to_cell(state.x, state.y)
        rect = self._cell_rect(col, row)
        highlight = pygame.Surface((rect.w, rect.h), pygame.SRCALPHA)
        highlight.fill((255, 200, 60, 40))
        self.map_surf.blit(highlight, rect.topleft)

        pygame.draw.circle(self.map_surf, ROBOT_COLOR, (sx, sy), ROBOT_RADIUS)
        h = math.radians(state.heading)
        ex = sx + int(math.cos(h) * ROBOT_RADIUS * 1.6)
        ey = sy - int(math.sin(h) * ROBOT_RADIUS * 1.6)
        pygame.draw.line(self.map_surf, ROBOT_DIR_COL, (sx, sy), (ex, ey), 3)
        pygame.draw.circle(self.map_surf, ROBOT_DIR_COL, (ex, ey), 3)
        # IR dots
        lx, ly = world_to_screen(
            *side_point(state.x, state.y, math.radians(state.heading), -IR_SIDE_OFFSET * 0.6))
        pygame.draw.circle(self.map_surf,
                           IR_L_ON if state.ir_left else (60, 60, 80), (lx, ly), 5)
        rx, ry = world_to_screen(
            *side_point(state.x, state.y, math.radians(state.heading), +IR_SIDE_OFFSET * 0.6))
        pygame.draw.circle(self.map_surf,
                           IR_R_ON if state.ir_right else (60, 60, 80), (rx, ry), 5)

    def draw_sidebar(self, state, maze, fps, port, paused, active_left, active_right):
        sx = MAP_W
        self.screen.fill(SIDEBAR_BG, (sx, 0, SIDEBAR_W, WINDOW_H))

        def label(text, y, color=TEXT_PRIMARY, font=None):
            s = (font or self.font_sm).render(text, True, color)
            self.screen.blit(s, (sx + 10, y))

        def divider(y):
            pygame.draw.line(self.screen, GRID_COLOR,
                             (sx + 8, y), (sx + SIDEBAR_W - 8, y), 1)
            return y + 10

        y = 14
        label("MAZE MAPPER",               y, ACCENT, self.font_lg); y += 28
        label(f"Port: {port}",             y, TEXT_MUTED);           y += 18
        label(f"FPS:  {fps:.1f}",          y, TEXT_MUTED);           y += 24
        y = divider(y)

        label("ROBOT POSE",                y, ACCENT);               y += 18
        label(f"X:   {state.x:6.1f} cm",  y);                       y += 16
        label(f"Y:   {state.y:6.1f} cm",  y);                       y += 16
        label(f"HDG: {state.heading:6.1f} deg", y);                  y += 16
        col, row = cm_to_cell(state.x, state.y)
        label(f"Cell: ({col}, {row})",     y);                       y += 24
        y = divider(y)

        label("IR SENSORS",                y, ACCENT);               y += 18
        col_l = IR_L_ON if state.ir_left  else TEXT_MUTED
        label(f"{'[ON]' if state.ir_left  else '[--]'} IR LEFT",  y, col_l); y += 16
        col_r = IR_R_ON if state.ir_right else TEXT_MUTED
        label(f"{'[ON]' if state.ir_right else '[--]'} IR RIGHT", y, col_r); y += 24
        y = divider(y)

        label("LIVE WALL",                 y, ACCENT);               y += 18
        if active_left:
            label(f"LEFT:  ({active_left.col},{active_left.row}) {active_left.axis}",
                  y, IR_L_ON);  y += 16
        else:
            label("LEFT:  ---",            y, TEXT_MUTED);           y += 16
        if active_right:
            label(f"RIGHT: ({active_right.col},{active_right.row}) {active_right.axis}",
                  y, IR_R_ON); y += 16
        else:
            label("RIGHT: ---",            y, TEXT_MUTED);           y += 16
        y += 8
        y = divider(y)

        label("MAP STATS",                 y, ACCENT);               y += 18
        label(f"Wall edges: {len(maze.walls)}",    y);               y += 16
        label(f"Cells seen: {len(maze.visited)}",  y);               y += 16
        label(f"Path pts:   {len(maze.path)}",     y);               y += 24
        y = divider(y)

        label("CONTROLS",                  y, ACCENT);               y += 18
        for txt in ["[C] Clear map", "[S] Save map", "[P] Pause", "[Q] Quit"]:
            label(txt, y, TEXT_MUTED); y += 16

        if paused:
            label("** PAUSED **", y + 8, (255, 100, 100), self.font_lg)

    def render(self, state, maze, fps, port, paused, active_left, active_right):
        self.draw_grid(maze.visited)
        self.draw_path(maze.path)
        self.draw_walls(maze.walls, active_left, active_right)
        self.draw_robot(state)
        self.screen.blit(self.map_surf, (0, 0))
        self.draw_sidebar(state, maze, fps, port, paused, active_left, active_right)
        pygame.display.flip()

# ─── Save ────────────────────────────────────────────────────────
def save_map(maze: MazeMap, state: RobotState):
    fname = f"maze_map_{int(time.time())}.txt"
    with open(fname, 'w') as f:
        f.write("# Maze map — encoder odometry + IR wall detection (grid)\n")
        f.write(f"# Final pose: x={state.x:.1f} y={state.y:.1f} hdg={state.heading:.1f}deg\n")
        f.write(f"# Grid: {GRID_COLS}x{GRID_ROWS} cells, {CELL_CM}cm per cell\n")
        f.write(f"# Wall edges: {len(maze.walls)}\n\n")
        f.write("# col,row,axis,side\n")
        for e in sorted(maze.walls, key=lambda e: (e.col, e.row, e.axis)):
            f.write(f"{e.col},{e.row},{e.axis},{e.side}\n")
        f.write("\n# VISITED CELLS: col,row\n")
        for col, row in sorted(maze.visited):
            f.write(f"{col},{row}\n")
        f.write("\n# PATH: x,y\n")
        for x, y in maze.path:
            f.write(f"{x:.1f},{y:.1f}\n")
    print(f"[Save] {fname}")

# ─── Main ────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Maze Mapper')
    parser.add_argument('--port', type=str, default=115200, required=False,
                        help='Serial port e.g. COM7 or /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Maze Mapper — Encoder + IR")
    clock = pygame.time.Clock()

    reader = SNPReader(args.port, args.baud)
    reader.start()
    print(f"[Start] Port: {args.port} @ {args.baud}")
    print("[Controls] C=clear  S=save  P=pause  Q=quit")

    maze     = MazeMap()
    renderer = Renderer(screen)
    paused   = False
    state    = RobotState()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if   event.key == pygame.K_q: running = False
                elif event.key == pygame.K_c: maze.clear(); print("[Clear]")
                elif event.key == pygame.K_s: save_map(maze, state)
                elif event.key == pygame.K_p:
                    paused = not paused
                    print(f"[{'Paused' if paused else 'Resumed'}]")

        if not paused:
            state = reader.get_state()
            maze.add_path_point(state.x, state.y)
            maze.add_walls(reader.pop_walls())

        renderer.render(state, maze, clock.get_fps(), args.port, paused,
                        reader.active_left, reader.active_right)
        clock.tick(30)

    reader.stop()
    pygame.quit()
    sys.exit()

if __name__ == '__main__':
    main()