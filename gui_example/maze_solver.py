"""
maze_solver.py
──────────────
Loads a saved maze map (from maze_mapper.py) and solves it visually.

Usage:
    python maze_solver.py maze_map_<timestamp>.txt

Controls:
    Left-click  — set START cell (green)
    Right-click — set END cell   (red)
    SPACE       — run pathfinding
    C           — clear path
    Q           — quit

To swap the algorithm: edit the `find_path()` function only.

Dependencies:
    pip install pygame
"""

import pygame
import heapq
import sys
import math
import argparse
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set, Dict

# ─── Must match maze_mapper.py constants exactly ─────────────────
WINDOW_W   = 900
WINDOW_H   = 700
MAP_W      = 680
MAP_H      = 660
SIDEBAR_W  = WINDOW_W - MAP_W
WORLD_CM   = 300
CELL_CM    = 15
GRID_COLS  = WORLD_CM // CELL_CM   # 20
GRID_ROWS  = WORLD_CM // CELL_CM   # 20
CELL_PX_W  = MAP_W // GRID_COLS
CELL_PX_H  = MAP_H // GRID_ROWS

# ─── Colors ──────────────────────────────────────────────────────
MAP_BG         = (26,  26,  36)
GRID_COLOR     = (50,  50,  70)
CELL_VISITED   = (34,  34,  50)
WALL_L_COLOR   = (255, 100, 100)
WALL_R_COLOR   = (100, 255, 150)
SIDEBAR_BG     = (22,  22,  30)
TEXT_PRIMARY   = (220, 220, 230)
TEXT_MUTED     = (120, 120, 140)
ACCENT         = ( 80, 160, 255)
START_COLOR    = ( 60, 220,  80)
END_COLOR      = (220,  60,  60)
PATH_COLOR     = (255, 200,  60)
EXPLORED_COLOR = ( 50,  80, 120)
WALL_THICKNESS = 4

# ─── Data structures ─────────────────────────────────────────────
@dataclass(frozen=True, eq=True)
class GridEdge:
    col:  int
    row:  int
    axis: str   # 'h' or 'v'
    side: str   # 'left' or 'right'

Cell = Tuple[int, int]   # (col, row)

# ─── Map loader ──────────────────────────────────────────────────
def load_map(filepath: str):
    """
    Parse a maze_map_*.txt file saved by maze_mapper.py.
    Returns (walls: set[GridEdge], visited: set[Cell]).
    """
    walls:   Set[GridEdge] = set()
    visited: Set[Cell]     = set()
    section = None

    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                if 'VISITED' in line:
                    section = 'visited'
                elif 'PATH' in line:
                    section = 'path'
                elif 'col,row,axis,side' in line:
                    section = 'walls'
                continue

            parts = line.split(',')
            if section == 'walls' and len(parts) == 4:
                col, row, axis, side = int(parts[0]), int(parts[1]), parts[2], parts[3]
                walls.add(GridEdge(col, row, axis, side))
            elif section == 'visited' and len(parts) == 2:
                visited.add((int(parts[0]), int(parts[1])))

    return walls, visited

# ─── Wall lookup helper ──────────────────────────────────────────
def build_wall_lookup(walls: Set[GridEdge]) -> Set[Tuple[int,int,str]]:
    """
    Convert wall edges into a fast lookup set of (col, row, axis) — side is
    irrelevant for passability; what matters is that the edge exists.
    Also register the shared edge from the neighbour's perspective so both
    sides of a wall boundary are blocked.
    """
    lookup = set()
    for e in walls:
        lookup.add((e.col, e.row, e.axis))
        # Register the same physical edge from the adjacent cell's view
        if e.axis == 'h':
            # This is the top edge of (col, row) = bottom edge of (col, row+1)
            lookup.add((e.col, e.row + 1, 'h'))
        else:
            # This is the right edge of (col, row) = left edge of (col+1, row)
            lookup.add((e.col + 1, e.row, 'v'))
    return lookup

def is_blocked(wall_lookup: Set, col: int, row: int,
               ncol: int, nrow: int) -> bool:
    """
    Return True if moving from (col,row) to (ncol,nrow) is blocked by a wall.
    Only cardinal moves (Δ=1) are considered.
    """
    dc, dr = ncol - col, nrow - row
    if dc == 1:   # moving right → check right 'v' edge of current cell
        return (col, row, 'v') in wall_lookup
    if dc == -1:  # moving left  → check right 'v' edge of neighbour
        return (ncol, nrow, 'v') in wall_lookup
    if dr == 1:   # moving up    → check top 'h' edge of current cell
        return (col, row, 'h') in wall_lookup
    if dr == -1:  # moving down  → check top 'h' edge of neighbour
        return (ncol, nrow, 'h') in wall_lookup
    return True

def get_neighbours(col: int, row: int,
                   wall_lookup: Set, visited: Set[Cell]) -> List[Cell]:
    """Return passable cardinal neighbours within the grid."""
    result = []
    for dc, dr in [(1,0),(-1,0),(0,1),(0,-1)]:
        nc, nr = col + dc, row + dr
        if 0 <= nc < GRID_COLS and 0 <= nr < GRID_ROWS:
            if not is_blocked(wall_lookup, col, row, nc, nr):
                result.append((nc, nr))
    return result

# ═══════════════════════════════════════════════════════════════════
#  ★  PATHFINDING FUNCTION — swap this to change the algorithm  ★
#
#  Signature must stay the same:
#    start       : (col, row) tuple
#    end         : (col, row) tuple
#    wall_lookup : set built by build_wall_lookup()
#    visited     : set of (col, row) that exist in the map
#
#  Must return:
#    path      : list of (col, row) from start to end inclusive,
#                or [] if no path found
#    explored  : list of (col, row) in the order they were visited
#                (used only for visualisation)
# ═══════════════════════════════════════════════════════════════════
def find_path(start: Cell, end: Cell,
              wall_lookup: Set, visited: Set[Cell]):
    """
    A* search with Manhattan distance heuristic.
    Replace the body of this function to use any other algorithm.
    """
    def heuristic(a: Cell, b: Cell) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_heap = []          # (f, g, cell)
    heapq.heappush(open_heap, (0, 0, start))

    came_from: Dict[Cell, Optional[Cell]] = {start: None}
    g_score:   Dict[Cell, int]            = {start: 0}
    explored:  List[Cell]                 = []

    while open_heap:
        _, g, current = heapq.heappop(open_heap)

        if current in explored:
            continue
        explored.append(current)

        if current == end:
            # Reconstruct path
            path = []
            node = end
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path, explored

        for nb in get_neighbours(current[0], current[1], wall_lookup, visited):
            tentative_g = g + 1
            if tentative_g < g_score.get(nb, float('inf')):
                g_score[nb]  = tentative_g
                f             = tentative_g + heuristic(nb, end)
                came_from[nb] = current
                heapq.heappush(open_heap, (f, tentative_g, nb))

    return [], explored   # no path found

# ─── Renderer ────────────────────────────────────────────────────
class Renderer:
    def __init__(self, screen: pygame.Surface):
        self.screen   = screen
        self.map_surf = pygame.Surface((MAP_W, MAP_H))
        self.font_lg  = pygame.font.SysFont('monospace', 15, bold=True)
        self.font_sm  = pygame.font.SysFont('monospace', 12)

    def cell_rect(self, col: int, row: int) -> pygame.Rect:
        px = col * CELL_PX_W
        py = MAP_H - (row + 1) * CELL_PX_H
        return pygame.Rect(px, py, CELL_PX_W, CELL_PX_H)

    def draw_base(self, visited: Set[Cell]):
        self.map_surf.fill(MAP_BG)
        for (col, row) in visited:
            pygame.draw.rect(self.map_surf, CELL_VISITED, self.cell_rect(col, row))
        for c in range(GRID_COLS + 1):
            x = c * CELL_PX_W
            pygame.draw.line(self.map_surf, GRID_COLOR, (x, 0), (x, MAP_H), 1)
        for r in range(GRID_ROWS + 1):
            y = r * CELL_PX_H
            pygame.draw.line(self.map_surf, GRID_COLOR, (0, y), (MAP_W, y), 1)

    def draw_walls(self, walls: Set[GridEdge]):
        for edge in walls:
            color = WALL_L_COLOR if edge.side == 'left' else WALL_R_COLOR
            rect  = self.cell_rect(edge.col, edge.row)
            if edge.axis == 'h':
                y = rect.top if edge.side == 'left' else rect.bottom
                pygame.draw.line(self.map_surf, color,
                                 (rect.left, y), (rect.right, y), WALL_THICKNESS)
            else:
                x = rect.left if edge.side == 'left' else rect.right
                pygame.draw.line(self.map_surf, color,
                                 (x, rect.top), (x, rect.bottom), WALL_THICKNESS)

    def draw_overlay(self, explored: List[Cell], path: List[Cell],
                     start: Optional[Cell], end: Optional[Cell]):
        # Explored cells (faint blue)
        for cell in explored:
            if cell != start and cell != end:
                rect = self.cell_rect(*cell)
                surf = pygame.Surface((rect.w, rect.h), pygame.SRCALPHA)
                surf.fill((*EXPLORED_COLOR, 80))
                self.map_surf.blit(surf, rect.topleft)

        # Solution path (yellow highlight)
        for cell in path:
            if cell != start and cell != end:
                rect = self.cell_rect(*cell)
                surf = pygame.Surface((rect.w, rect.h), pygame.SRCALPHA)
                surf.fill((*PATH_COLOR, 140))
                self.map_surf.blit(surf, rect.topleft)
            # Draw path line through cell centres
        if len(path) > 1:
            pts = []
            for col, row in path:
                r = self.cell_rect(col, row)
                pts.append(r.center)
            pygame.draw.lines(self.map_surf, PATH_COLOR, False, pts, 3)

        # Start / end markers
        if start:
            rect = self.cell_rect(*start)
            pygame.draw.rect(self.map_surf, START_COLOR, rect.inflate(-4, -4), 0)
            pygame.draw.rect(self.map_surf, (255,255,255), rect.inflate(-4,-4), 2)
        if end:
            rect = self.cell_rect(*end)
            pygame.draw.rect(self.map_surf, END_COLOR, rect.inflate(-4, -4), 0)
            pygame.draw.rect(self.map_surf, (255,255,255), rect.inflate(-4,-4), 2)

    def draw_sidebar(self, filepath: str, walls: Set, visited: Set,
                     start: Optional[Cell], end: Optional[Cell],
                     path: List, explored: List, status: str):
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
        label("MAZE SOLVER",     y, ACCENT, self.font_lg); y += 28
        # truncate long filename
        fname = filepath.split('/')[-1]
        label(fname[:18],        y, TEXT_MUTED);           y += 24
        y = divider(y)

        label("MAP INFO",        y, ACCENT);               y += 18
        label(f"Walls:  {len(walls)}",    y);              y += 16
        label(f"Cells:  {len(visited)}", y);               y += 24
        y = divider(y)

        label("SELECTION",       y, ACCENT);               y += 18
        sc = START_COLOR if start else TEXT_MUTED
        label(f"Start: {start if start else '---'}", y, sc); y += 16
        ec = END_COLOR if end else TEXT_MUTED
        label(f"End:   {end if end else '---'}",     y, ec); y += 24
        y = divider(y)

        label("RESULT",          y, ACCENT);               y += 18
        label(f"Path len:  {len(path)}",     y);           y += 16
        label(f"Explored:  {len(explored)}", y);           y += 24
        y = divider(y)

        label("STATUS",          y, ACCENT);               y += 18
        col = (100, 255, 150) if 'found' in status.lower() else \
              (255, 100, 100) if 'no path' in status.lower() else TEXT_PRIMARY
        label(status,            y, col);                  y += 24
        y = divider(y)

        label("CONTROLS",        y, ACCENT);               y += 18
        for txt in ["[LClick] Set start",
                    "[RClick] Set end",
                    "[SPACE]  Solve",
                    "[C]      Clear",
                    "[Q]      Quit"]:
            label(txt, y, TEXT_MUTED); y += 16

    def render(self, walls, visited, explored, path, start, end,
               filepath, status):
        self.draw_base(visited)
        self.draw_walls(walls)
        self.draw_overlay(explored, path, start, end)
        self.screen.blit(self.map_surf, (0, 0))
        self.draw_sidebar(filepath, walls, visited, start, end,
                          path, explored, status)
        pygame.display.flip()

# ─── Pixel → cell ────────────────────────────────────────────────
def screen_to_cell(mx: int, my: int) -> Optional[Cell]:
    if mx < 0 or mx >= MAP_W or my < 0 or my >= MAP_H:
        return None
    col = mx // CELL_PX_W
    row = (MAP_H - my) // CELL_PX_H
    col = max(0, min(GRID_COLS - 1, col))
    row = max(0, min(GRID_ROWS - 1, row))
    return (col, row)

# ─── Main ────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Maze Solver')
    parser.add_argument('mapfile', type=str, help='Path to maze_map_*.txt')
    args = parser.parse_args()

    print(f"[Load] {args.mapfile}")
    walls, visited = load_map(args.mapfile)
    wall_lookup    = build_wall_lookup(walls)
    print(f"       {len(walls)} wall edges, {len(visited)} visited cells")

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Maze Solver — A*")
    clock  = pygame.time.Clock()

    renderer = Renderer(screen)

    start:    Optional[Cell] = None
    end:      Optional[Cell] = None
    path:     List[Cell]     = []
    explored: List[Cell]     = []
    status    = "Set start + end, then SPACE"

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_c:
                    path, explored, status = [], [], "Cleared."
                elif event.key == pygame.K_SPACE:
                    if start and end:
                        status = "Solving…"
                        renderer.render(walls, visited, explored, path,
                                        start, end, args.mapfile, status)
                        pygame.display.flip()
                        path, explored = find_path(start, end,
                                                   wall_lookup, visited)
                        if path:
                            status = f"Path found! {len(path)} cells, {len(explored)} explored"
                        else:
                            status = "No path found."
                        print(f"[Solve] {status}")
                    else:
                        status = "Pick start (L) and end (R) first."

            elif event.type == pygame.MOUSEBUTTONDOWN:
                cell = screen_to_cell(*event.pos)
                if cell and cell in visited:
                    if event.button == 1:      # left → start
                        start = cell
                        path, explored = [], []
                        status = f"Start set to {start}"
                    elif event.button == 3:    # right → end
                        end = cell
                        path, explored = [], []
                        status = f"End set to {end}"
                elif cell:
                    status = "Cell not in map — pick a visited cell."

        renderer.render(walls, visited, explored, path,
                        start, end, args.mapfile, status)
        clock.tick(30)

    pygame.quit()
    sys.exit()

if __name__ == '__main__':
    main()