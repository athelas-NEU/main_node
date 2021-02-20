from states.state import State
# import scripts.states.position_arm import PositionArmXY

class Idle(State):
  """
  Waits for command, then transitions to PostionArm state.
  """

  def execute(self):
    super().execute()

    biosensor = input("Enter the biosensor name: ")
    print(biosensor)
    return self
    # return PositionArmXY(biosensor)

