from entities.coach.coach import BaseCoach
from entities.coach.coach_rcx2023 import Coach as RCX2023

#from entities.coach.test_coach import Coach as TestCoach

_coach_list = [
    # Tournament coaches
    RCX2023
]

COACHES = {c.NAME: c for c in _coach_list}
