(define (problem actionmove-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart loc1 loc4 loc7 locfinish - location
        cube1 cube2 cube3 - object
        pf2l1 pf2l4 pf2l7 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart) 0)
        (= (restantes loc1) 1)
        (= (restantes loc4) 1)
        (= (restantes loc7) 1)
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube1 loc1)
        (obj-at cube2 loc4)
        (obj-at cube3 loc7)

        (position-free loc1 pf2l1)
        (position-free loc4 pf2l4)
        (position-free loc7 pf2l7)

        (connected locstart loc1)
        (connected loc1 loc4)
        (connected loc4 loc7)
        (connected loc7 loc1)
        (connected loc1 locfinish)
    )

    (:goal
        (and
            (obj-at cube1 loc4)
            (obj-at cube2 loc7)
            (obj-at cube3 loc1)
            (at-robot robot1 locfinish)
        )
    )
)