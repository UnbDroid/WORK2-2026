(define (problem actionmove-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart loc1 loc2 loc3 locfinish - location
        cube1 cube2 cube3 cube4 - object
        p1l1 p2l1 p1l2 p2l2 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes loc1) 0)
        (= (restantes loc2) 0)
        (= (restantes loc3) 4)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube1 loc3)
        (obj-at cube2 loc3)
        (obj-at cube3 loc3)
        (obj-at cube4 loc3)

        (position-free loc1 p1l1)
        (position-free loc2 p1l2)
        (position-free loc2 p2l2)
        (position-free loc1 p2l1)

        (connected locstart loc3)
        (connected loc3 loc2)
        (connected loc2 loc3)
        (connected loc2 loc1)
        (connected loc1 locfinish)
    )

    (:goal
        (and
            (obj-at cube3 loc1)
            (obj-at cube2 loc2)
            (obj-at cube1 loc2)
            (obj-at cube4 loc1)
            (at-robot robot1 locfinish)
        )
    )
)