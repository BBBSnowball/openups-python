# dummy implementation of Python's logging module because that module seems to be missing on OpenWRT
class Logger:
  def setLevel(*args): pass
  def addHandler(*args): pass
  def info(*args): pass
  def warn(*args): print(repr(args))
  def error(*args): print(repr(args))
class Handler:
  pass
logger = Logger()
CRITICAL = 10
def getLogger(*args): return logger
