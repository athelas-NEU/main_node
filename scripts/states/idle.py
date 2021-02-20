import scripts.states.state import State
import scripts.states.position_arm import PositionArmXY
import scripts.main_node

class Idle(State):
  """
  Waits for command, then transitions to PostionArm state.
  """

  def execute(self):
    super().execute()

    biosensor = input("Enter the biosensor name: ") 
    main_node.state = PositionArmXY(biosensor)
