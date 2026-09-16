; os numeros sao todos definidos aleatoriamente e deverao ser conferidos
; de acordo com o que for pedido no momento da competicao

(define (problem att1-instance) (:domain robocup-work-transport)

  (:objects
    robot1 - robot
    start ws1 ws3 ws4 ws6 ws8 sh2 - location
    s1 s2 s3 - slot
    cont10 cont16 - container
    obj20 obj21 obj22 obj23 obj24 obj25 - object   ; ATTCs
    obj12 obj15 obj16 - object                      ; objetos de distracao (iscas)
  )

  (:init
    (at-robot robot1 start)
    (slot-free robot1 s1)
    (slot-free robot1 s2)
    (slot-free robot1 s3)

    (container-at cont10 ws1)   ; container azul
    (container-at cont16 ws8)   ; container vermelho


    (obj-at obj12 ws1)   ; isca
    (obj-at obj20 ws1)
    (obj-at obj22 ws1)

    (obj-at obj15 ws3)   ; isca

    (obj-at obj21 ws4)
    (obj-at obj23 ws4)
    (obj-at obj24 ws4)

    (obj-at obj16 ws6)   ; isca
    (obj-at obj25 ws6)
  )

  (:goal (and
    (in-container obj21 cont10)     ; azul
    (in-container obj23 cont16)     ; vermelho

    (obj-at obj24 sh2)              ; prateleira

    (obj-at obj20 ws3)              ; par empilhado
    (obj-at obj22 ws3)
    (on obj22 obj20)                ; obj22 empilhado sobre obj20

    (obj-at obj25 ws1)              ; transporte
  ))
)