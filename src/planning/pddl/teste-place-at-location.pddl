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

    )

    (:init
    
        (gripper-free robot1)
        (at-robot robot1 loc1)
        (holding robot1 cube1)
        (holding-in robot1 cube1 slot2)
        (holding robot1 cube2)
        (holding-in robot1 cube2 slot3)
        (holding robot1 cube3)
        (holding-in robot1 cube3 slot1)

    )

    (:goal (and

            (on cube1 cube2 loc1)
            (obj-at cube3 loc1)

        ) 
    )
)