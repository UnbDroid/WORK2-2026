; os numeros sao todos definidos aleatoriamente e deverao ser conferidos
; de acordo com o que for pedido no momento da competicao

(define (problem btt1-instance) (:domain robocup-work-transport)

  (:objects
    robot1 - robot
    start ws3 ws4 ws5 - location
    s1 s2 s3 - slot
    obj1 obj3 obj5 - object
  )

  (:init
    (at-robot robot1 start)
    (slot-free robot1 s1)
    (slot-free robot1 s2)
    (slot-free robot1 s3)

    (obj-at obj1 ws3)
    (obj-at obj3 ws4)
    (obj-at obj5 ws5)
  )

  (:goal (and
    (obj-at obj1 ws4)
    (obj-at obj3 ws5)
    (obj-at obj5 ws3)
  ))
)
