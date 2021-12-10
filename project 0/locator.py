from typing import Any, Set, Tuple
from grid import Grid
import utils


def locate(grid: Grid, item: Any) -> Set[Tuple[int, int]]:
    '''
    This function takes a 2D grid and an item
    It should return a list of (x, y) coordinates that specify the locations that contain the given item
    To know how to use the Grid class, see the file "grid.py"  
    '''

    tup = ()
    se = set()
    for i in range(grid.width):
        for j in range(grid.height):
            if(grid[i, j] == item):
                tup = (i, j)
                se.add(tup)

    return se
