(define (problem shelf-instance) (:domain robocup-work-transport)

  (:objects
    robot1 - robot
    locstart loc1 locshelf1 locshelf2 locfinish - location
    slot1 slot2 slot3 - slot
    cube1 cube2 - object
    pfl1 pfls - position
  )

  (:init
    (= (carga robot1) 0)

    (= (restantes locstart) 0)
    (= (restantes loc1) 1)
    (= (restantes locshelf1) 1)
    (= (restantes locshelf2) 0)
    (= (restantes locfinish) 0)

    (gripper-free robot1)
    (at-robot robot1 locstart)

    (slot-free robot1 slot1)
    (slot-free robot1 slot2)
    (slot-free robot1 slot3)

    (obj-at cube1 loc1)
    (obj-at cube2 locshelf1)

    (connected locstart loc1)
    (connected loc1 locshelf1)
    (connected locshelf1 locshelf2)
    (connected locshelf2 loc1)
    (connected loc1 locfinish)

    (position-free loc1 pfl1)
    (position-free locshelf2 pfls)
  )

  (:goal
    (and
      (obj-at cube1 locshelf2)
      (obj-at cube2 loc1)
      (at-robot robot1 locfinish)
    )
  )
)