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
        (= (carga robot1) 0)

        (= (restantes locstart) 0)
        (= (restantes locpp) 1)
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (obj-at cube1 locpp)
        (cavity-free locpp cav1)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (connected locstart locpp)
        (connected locpp locfinish)
    )

    (:goal
        (and
            (in-cavity cube1 locpp cav1)
            (at-robot robot1 locfinish)
        )
    )
)