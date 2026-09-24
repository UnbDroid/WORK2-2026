(define (problem actionmove-test) (:domain robocup-work-transport)
(:objects
    robot1 - robot
    start wp1 - location
)

(:init
    (at-robot robot1 start)
)

(:goal (and
    (at-robot robot1 wp1)
))
)
