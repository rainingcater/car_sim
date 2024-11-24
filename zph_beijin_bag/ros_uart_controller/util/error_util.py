class NoPathFileError(Exception):
    """Raised when no pathfile matches a requested option."""

    def __init__(self, pathfile):
        Exception.__init__(self, 'No PathFile: %r' % (pathfile,))
        self.pathfile = pathfile
        self.args = (pathfile, )
class TypeError(Exception):
    """Raised when no type matches a requested option."""

    def __init__(self, type):
        Exception.__init__(self, 'TypeError: %r' % (type,))
        self.type = type
        self.args = (type, )
if __name__=="__main__":
    # raise NoPathFileError("sdldfjls")
    raise TypeError("hahaah")

