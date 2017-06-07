import termios
import atexit
from select import select
import sys


class KBHit:
    """ Class for dealing with keyboard inputs

    Allows the user to log what key has been pressed. This can be used as a
    trigger for function calls, starting and stopping scripts etc. To use,
    you must first call set_normal_term to set the terminal to its initial
    state. Then you can monitor the keyboard in your main loop.

    KBHIT.set_normal_term()

    if KBHit.kbhit():
        c = KBHIT.getch # variabe to store key press

        if ord(c) == ord('s'):
            print('Starting Script')
    
    Further examples can be found at
    http://home.wlu.edu/~levys/software/kbhit.py
    http://code.activestate.com/recipes/572182-how-to-implement-kbhit-on-linux/
    """

    def __init__(self):
        # Save the terminal settings
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)

        # New terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

        # Support normal-terminal reset at exit
        atexit.register(self.set_normal_term)

    def set_normal_term(self):
        """ Resets to normal terminal.
        """
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    def getch(self):
        """ Returns a keyboard character after kbhit() has been called.
        """
        return sys.stdin.read(1)

    def kbhit(self):
        """ Returns True if keyboard character was hit, False otherwise.
        """
        dr, dw, de = select([sys.stdin], [], [], 0)
        return dr != []
