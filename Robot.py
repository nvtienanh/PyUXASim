class Robot(object):
    """Class Link

        Longer class information....
        Longer class information....

        Attributes:
            Attributes 1: information....
            Attributes 2: information....
    """
    # TODO(nvtienanh): Adding more comment.

    def __init__(self):
        """Inits Robot class"""
        self.links = []
        # self.CoM = 0
        # self.ZMP

    def public_method(self):
        """Performs operation blah."""

    def add_link(self, link):
        """Adding link to robot model."""
        self.links.append(link)

    def display_info(self, link):
        """Display robot links info."""
        for index in range(len(self.links)):
            self.links[index].info
