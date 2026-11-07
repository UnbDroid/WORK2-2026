(define (problem actionmove-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart loc7 loc8 locfinish - location
        cube3 - object
        p1l7 - position
    )

    (:init
        (= (carga robot1) 0)
        (= (restantes locstart) 0)
        (= (restantes loc7) 1)
	(= (restantes loc8) 0) 
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube3 loc7)

        (connected locstart loc7)
        (connected loc7 loc8)
        (connected loc8 locfinish)

	(position-free loc8 p1l7)
    )

    (:goal
        (and
	    (at-robot robot1 locfinish)
            (obj-at cube3 loc8)

        )
    ) 
)