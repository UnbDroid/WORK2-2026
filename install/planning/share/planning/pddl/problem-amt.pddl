; os numeros sao todos definidos aleatoriamente e deverao ser conferidos
; de acordo com o que for pedido no momento da competicao

(define (problem amt-instance) (:domain robocup-work-transport)

  (:objects
    robot1 - robot
    start pp1 pp2 pp3 pp4 pp5 pp6 pp7 - location
    s1 s2 s3 - slot
    tag1 tag2 tag3 tag4 tag5 tag6 - object
  )

  (:init
    (at-robot robot1 start)
    (slot-free robot1 s1)
    (slot-free robot1 s2)
    (slot-free robot1 s3)

    ; pp3 comeca vazio (slot vazio da regra)
    ; ordem aleatoria
    (obj-at tag4 pp1)
    (obj-at tag1 pp2)
    (obj-at tag6 pp4)
    (obj-at tag2 pp5)
    (obj-at tag5 pp6)
    (obj-at tag3 pp7)
  )

  (:goal (and
    (obj-at tag1 pp1)
    (obj-at tag2 pp2)
    (obj-at tag3 pp3)
    (obj-at tag4 pp4)
    (obj-at tag5 pp5)
    (obj-at tag6 pp6)
    ; pp7 fica livre ao final
  ))
)