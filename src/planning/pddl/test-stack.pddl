(define

    (problem placeatlocation)

    (:domain robocup-work-transport)

    (:objects 
    
        robot1 - robot
        cube1 cube2 - object
        locstart loc7 loc4 locfinish - location
        slot1 slot2 slot3 - slot 
        p2l4 - position

    )

    (:init

        (= (carga robot1) 0)

        (= (restantes locstart) 0)
        (= (restantes loc7) 2)
        (= (restantes loc4) 0)
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube1 loc7)
        (obj-at cube2 loc7)

        (connected locstart loc7)
        (connected loc7 loc4)
        (connected loc4 locfinish)

        (position-free loc4 p2l4)

    )

    (:goal (and

        (at-robot robot1 locfinish)
        (on cube1 cube2 loc4)

        ) 
    )
)