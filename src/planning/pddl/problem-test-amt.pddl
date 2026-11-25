(define (problem amt-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        cav1 - cavity
        locstart locpp1 locfinish - location
        cube1 - object
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart) 0)
        (= (restantes locpp1) 1)
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (obj-at cube1 locpp1)
        (cavity-free locpp1 cav1)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)
        (connected locstart locpp1)
        (connected locpp1 locfinish)
    )

    (:goal
        (and
            (in-cavity cube1 locpp1
 cav1)
            (at-robot robot1 locfinish)
        )
    )
)