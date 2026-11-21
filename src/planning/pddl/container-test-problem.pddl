(define (problem container-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        cube1 - object
        container-2 - container
        locstart loc1 locfinish - location
    )

    (:init
        (= (carga robot1) 0)
        (= (restantes locstart) 0)
        (= (restantes loc1) 1)
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube1 loc1)
        
        (container-at container-2 loc1) 

        (connected locstart loc1)
        (connected loc1 locstart)
        (connected loc1 locfinish)
        (connected locfinish locstart)
        (connected locfinish loc1)
    )

    (:goal
        (and
            (in-container cube1 container-2)
            (at-robot robot1 locfinish)
        )
    )
)