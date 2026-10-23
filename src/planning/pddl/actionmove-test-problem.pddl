(define (problem actionmove-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        start loc2 loc3 - location
        cube1 cube2 cube3 - object
        p1l1 p2l1 p3l1 - position
    )

    (:init
        (at-robot robot1 start)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)
        (gripper-free robot1)
        (obj-at cube1 loc2)
        (obj-at cube2 loc2)
        (obj-at cube3 loc2)
        (position-free loc3 p1l1)
        (position-free loc3 p2l1)
        (position-free loc3 p3l1)
        (connected start loc2)
        (connected loc2 loc3)
    )

    (:goal
        (and
            (obj-at cube3 loc3)
            (obj-at cube2 loc3)
            (obj-at cube1 loc3)    
        )
    )
)