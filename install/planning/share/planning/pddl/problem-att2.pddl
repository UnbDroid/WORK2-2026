; os numeros sao todos definidos aleatoriamente e deverao ser conferidos
; de acordo com o que for pedido no momento da competicao

(define (problem att2-instance) (:domain robocup-work-transport)

  (:objects
    robot1 - robot
    start ws1 ws2 ws4 pp sh1 sh2 - location
    s1 s2 s3 - slot
    cont10 cont16 - container
    obj1 obj3 obj5 obj6 obj19 obj20 obj21 obj22 - object
  )

  (:init
    (at-robot robot1 start)
    (slot-free robot1 s1)
    (slot-free robot1 s2)
    (slot-free robot1 s3)

    ; containers fixos (colocação inicial = mesma da colocação final)
    (container-at cont10 ws1)   ; container azul
    (container-at cont16 ws2)   ; container vermelho

    ; start_state
    (obj-at obj1 ws2)
    (obj-at obj19 ws2)
    (obj-at obj3 ws4)
    (obj-at obj6 ws4)
    (obj-at obj5 pp)
    (obj-at obj21 pp)
    (obj-at obj20 sh1)
    (obj-at obj22 sh2)
  )

  (:goal
    (and
      ; WS_1: blue_container_10 -> [1, 3]
      (in-container obj1 cont10)
      (in-container obj3 cont10)

      ; WS_2: red_container_16 -> [5, 19]
      (in-container obj5 cont16)
      (in-container obj19 cont16)

      ; PP: [6, 20]
      (obj-at obj6 pp)
      (obj-at obj20 pp)

      ; SH_1: [21]
      (obj-at obj21 sh1)

      ; SH_2: [22] (já está no estado inicial, sem ação necessária)
      (obj-at obj22 sh2)
  ))
)