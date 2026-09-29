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
    
        (at-robot robot1 loc1)
        (holding-in robot1 cube1 slot1)
        (holding-in robot1 cube2 slot2)

    )

    (:goal (and

        (on cube1 cube2 loc1)

        ) 
    )
)