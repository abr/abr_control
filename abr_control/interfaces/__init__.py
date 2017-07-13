from .vrep import VREP
try:
    from .pygame import PyGame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False