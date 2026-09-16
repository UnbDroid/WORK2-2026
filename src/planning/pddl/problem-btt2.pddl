; os numeros sao todos definidos aleatoriamente e deverao ser conferidos
; de acordo com o que for pedido no momento da competicao

(define (problem btt2-instance) (:domain robocup-work-transport)
(:objects
    robot1 - robot
    start ws1 ws2 ws3 ws7 - location
    s1 s2 s3 - slot
    obj1 obj3 obj5 obj20 obj22 - object
)

(:init
    (at-robot robot1 start)
    (slot-free robot1 s1)
    (slot-free robot1 s2)
    (slot-free robot1 s3)

    (obj-at obj1 ws1)
    (obj-at obj20 ws1)

    (obj-at obj5 ws3)

    (obj-at obj3 ws7)
    (obj-at obj22 ws7)
)

(:goal (and
    (obj-at obj3 ws1)

    (obj-at obj1 ws2)
    
    (obj-at obj20 ws3)
    (obj-at obj22 ws3)
  
    (obj-at obj5 ws7)
))
)