class Link(object):
    """Class Link

        Longer class information....
        Longer class information....

        Attributes:
            Attributes 1: information....
            Attributes 2: information....
    """
    # TODO(nvtienanh): Adding more comment.

    def __init__(self, Name, Id, Mass, Sister, Child):
        self.name = Name
        self.id = Id
        self.m = Mass
        self.sister = Sister
        self.child = Child

    def info(self):
        print(self.name)

