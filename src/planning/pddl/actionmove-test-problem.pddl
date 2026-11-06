(define (problem actionmove-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart loc8 loc9 loc7 locfinish - location
        cube1 cube2 cube3 - object
        p1l1 p1l4 p1l7 - position
    )

    (:init
        (= (carga robot1) 0)
        (= (restantes locstart) 0)
        (= (restantes loc9) 1) 
        (= (restantes loc8) 1) 
        (= (restantes loc7) 1) 
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube1 loc7)
        (obj-at cube2 loc8)
        (obj-at cube3 loc9)

        (position-free loc9 p1l1)
        (position-free loc8 p1l4)
        (position-free loc7 p1l7)

        (connected locstart loc7)
        (connected loc7 loc8)
        (connected loc8 loc9)
        (connected loc9 loc7)
        (connected loc7 locfinish)
    )

    (:goal
        (and
            (obj-at cube1 loc8)
            (obj-at cube2 loc9)
            (obj-at cube3 loc7)
            (at-robot robot1 locfinish)
        )
    ) 
)
