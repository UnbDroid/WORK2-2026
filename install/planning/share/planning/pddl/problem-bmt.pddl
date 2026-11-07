; os numeros sao todos definidos aleatoriamente e deverao ser conferidos
; de acordo com o que for pedido no momento da competicao

(define (problem bmt-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        cube1 cube2 cube3 - object               ; Trocar para número de april tags que devem ser pegas
        locstart loc1 loc2 locfinish - location  ; Trocar 1 para altura da location
        p1l2 p2l2 p3l2 - position                ; Trocar numero depois de l para location correspondente
    )

    (:init
        (= (carga robot1) 0)                     ; Robo sempre começa carregando 0 cubos

        (= (restantes loc1) 3)                   ; Inserir aqui apenas localizações que começam com cubos, e suas quantidades

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube1 loc1)
        (obj-at cube2 loc1)
        (obj-at cube3 loc1)

        (position-free loc2 p1l2)
        (position-free loc2 p2l2)
        (position-free loc2 p3l2)

        (connected locstart loc1)
        (connected loc1 loc2)
        (connected loc2 locfinish)
    )

    (:goal
        (and
            (obj-at cube1 loc2)
            (obj-at cube2 loc2)
            (obj-at cube3 loc2)
            (at-robot robot1 locfinish)
        )
    )
)