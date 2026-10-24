(define (problem actionmove-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        start loc2 loc3 - location
        cube1 cube2 cube3 - object
        pos1 frente pos3 - position
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
        (position-free loc3 pos1)
        (position-free loc3 frente)
        (position-free loc3 pos3)
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
