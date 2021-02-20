
class State(object):
  """
  State parent class.
  """

  def __init__(self):
    print(f"Processing current state: {str(self)}")

  def execute(self):
    pass

  def __repr__(self):
    return self.__str__()

  def __str__(self):
    """
    Returns the name of the State.
    """
    return self.__class__.__name__