"""To enable `roboautotask.__version__`"""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("roboautotask")
except PackageNotFoundError:
    __version__ = "unknown"
