(define

    (problem placeatlocation)

    (:domain robocup-work-transport)

    (:objects 
    
        robot1 - robot
        cube1 - object
        cube2 - object
        cube3 - object
        loc1 - location
        slot1 - slot
        slot2 - slot
        slot3 - slot
        pos1 - position
        pos2 - position
        pos3 - position

    )

    (:init
    
        (gripper-free robot1)
        (at-robot robot1 loc1)
        (holding-in robot1 cube1 slot2)
        (holding-in robot1 cube2 slot3)
        (holding-in robot1 cube3 slot1)
        (position-free loc1 pos1)
        (position-free loc1 pos2)
        (position-free loc1 pos3)

    )

    (:goal (and

            (obj-at cube1 loc1)
            (obj-at cube2 loc1)
            (obj-at cube3 loc1)

        ) 
    )
)