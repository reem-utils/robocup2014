#! /usr/bin/env python
import rospy


class Colors():
    """Colors class.

    Use this class to print some text with colors on the screen (terminal).
    Useful to debug and print important informations.

    Example of use:
        c = Colors()
        rospy.loginfo(c.GREEN + "This is a green text!" + c.NATIVE_COLOR)

    """
    def __init__(self, debug_colors=False):
        """Constructor for Colors class.

        @type debug_colors: boolean
        @param debug_colors: If True, all the possible colors on this class will be printed with it colors when a Color object is created.

        """
        self.RED = "\033[00;31m"
        self.RED_BOLD = "\033[01;31m"
        self.GREEN = "\033[00;32m"
        self.GREEN_BOLD = "\033[01;32m"
        self.YELLOW_BOLD = "\033[01;33m"

        self.NATIVE_COLOR = "\033[m"

        self.BLACK = "\033[00;30m"
        self.RED = "\033[00;31m"
        self.GREEN = "\033[00;32m"
        self.YELLOW = "\033[00;33m"
        self.BLUE = "\033[00;34m"
        self.MAGENTA = "\033[00;35m"
        self.CYAN = "\033[00;36m"
        self.WHITE = "\033[00;37m"

        self.BLACK_BOLD = "\033[01;30m"
        self.RED_BOLD = "\033[01;31m"
        self.GREEN_BOLD = "\033[01;32m"
        self.YELLOW_BOLD = "\033[01;33m"
        self.BLUE_BOLD = "\033[01;34m"
        self.MAGENTA_BOLD = "\033[01;35m"
        self.CYAN_BOLD = "\033[01;36m"
        self.WHITE_BOLD = "\033[01;37m"

        self.BLACK_UNDERSCORE = "\033[04;30m"
        self.RED_UNDERSCORE = "\033[04;31m"
        self.GREEN_UNDERSCORE = "\033[04;32m"
        self.YELLOW_UNDERSCORE = "\033[04;33m"
        self.BLUE_UNDERSCORE = "\033[04;34m"
        self.MAGENTA_UNDERSCORE = "\033[04;35m"
        self.CYAN_UNDERSCORE = "\033[04;36m"
        self.WHITE_UNDERSCORE = "\033[04;37m"

        self.BACKGROUND_BLACK = "\033[01;40m"
        self.BACKGROUND_RED = "\033[01;41m"
        self.BACKGROUND_GREEN = "\033[01;42m"
        self.BACKGROUND_YELLOW = "\033[01;43m"
        self.BACKGROUND_BLUE = "\033[01;44m"
        self.BACKGROUND_MAGENTA = "\033[01;45m"
        self.BACKGROUND_CYAN = "\033[01;46m"
        self.BACKGROUND_WHITE = "\033[01;47m"

        if debug_colors is True:
            colors = self.__dict__.keys()
            colors.sort()
            for color in colors:
                rospy.loginfo(self.__dict__[color] + color + self.NATIVE_COLOR)

colors = Colors()

