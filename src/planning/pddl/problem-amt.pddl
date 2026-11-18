(define (problem amt-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        cav1 - cavity
        locstart locpp locfinish - location
        cube1 - object
    )

    (:init
        (at-robot robot1 locstart)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)
        (gripper-free robot1)
        (cavity-free locpp cav1)
        (obj-at cube1 locpp)
        (connected locstart locpp)
        (connected locpp locfinish)
    )

    (:goal
        (and
            (in-cavity cube1 locpp cav1)
            (obj-at cube1 locpp)
            (at-robot robot1 locfinish)
        )
    )
)