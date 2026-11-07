(define

    (problem pickfromlocation)

    (:domain robocup-work-transport)

    (:objects 
    
        robot1 - robot
        cube1 - object
        cube2 - object
        cube3 - object
        loc3 - location
        slot1 - slot
        slot2 - slot
        slot3 - slot

    )

    (:init
    
        (at-robot robot1 loc3)
        (obj-at cube3 loc3)
        (gripper-free robot1)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

    )

    (:goal (and

            (holding-in robot1 cube3 slot1)

        ) 
    )
)