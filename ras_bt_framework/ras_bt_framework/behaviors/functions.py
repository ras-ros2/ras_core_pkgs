
from ..behavior_template.instruction import FunctionalInstruction

@FunctionalInstruction
def Hello():
    print("hello world!",123469)

@FunctionalInstruction
def SaySomethingPy(to_say:str,next_line:str):
    print("someone says:",to_say)
    print(next_line)