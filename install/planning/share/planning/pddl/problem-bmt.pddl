; os numeros sao todos definidos aleatoriamente e deverao ser conferidos
; de acordo com o que for pedido no momento da competicao

(define (problem bmt-instance) (:domain robocup-work-transport)

  (:objects
    robot1 - robot
    start ws1 ws4 ws8 - location
    s1 s2 s3 - slot
    obj1 obj2 obj4 obj7 - object
  )

  (:init
    (at-robot robot1 start)
    (slot-free robot1 s1)
    (slot-free robot1 s2)
    (slot-free robot1 s3)

    (obj-at obj1 ws1)
    (obj-at obj2 ws1)
    (obj-at obj4 ws1)
    (obj-at obj7 ws1)   ; obj7 = isca, nao deve ser manipulado
  )

  (:goal (and
    (obj-at obj1 ws4)
    (obj-at obj4 ws4)
    (obj-at obj2 ws8)

    ;; garante explicitamente que a isca nao foi tocada
    (obj-at obj7 ws1)
  ))
)
