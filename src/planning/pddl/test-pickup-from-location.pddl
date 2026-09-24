(define

    (problem pickfromlocation)

    (:domain robocup-work-transport)

    (:objects 
    
        robot1 - robot
        cube1 - object
        loc1 - location
        slot1 - slot

    )

    (:init
    
        (at-robot robot1 loc1)
        (obj-at cube1 loc1)
        (slot-free robot1 slot1)

    )

    (:goal (and

            (holding robot1 cube1)

        ) 
    )
)