(define (problem actionmove-test) (:domain robocup-work-transport)
(:objects
    robot1 - robot
    start wp1 - location
)

(:init
    (robot-at robot1 start)
)

(:goal (and
    (robot-at robot1 wp1)
))
)
