(define (problem actionmove-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        start wp1 wp2 - location
        cube1 cube2 cube3 - object
    )

    (:init
        (at-robot robot1 start)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)
        (gripper-free robot1)
        (object-at cube1 loc3)
        (object-at cube2 loc3)
        (object-at cube3 loc3)


    )

    (:goal
        (and
            (at-robot robot1 loc3)
            (holding-in robot1 cube1 slot1)
            (holding-in robot1 cube2 slot2)
            (holding-in robot1 cube3 slot3)
        )
    )
)