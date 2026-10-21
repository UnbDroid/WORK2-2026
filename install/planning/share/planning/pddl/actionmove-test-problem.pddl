(define (problem actionmove-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        start loc1 loc3 - location
        cube1 cube2 cube3 - object
    )

    (:init
        (at-robot robot1 start)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)
        (gripper-free robot1)
        (obj-at cube1 loc3)
        (obj-at cube2 loc3)
        (obj-at cube3 loc3)


    )

    (:goal
        (and
            (at-robot robot1 loc1)
            (obj-at cube3 loc1)
            (obj-at cube2 loc1)
            (obj-at cube1 loc1)   
        )
    )
)