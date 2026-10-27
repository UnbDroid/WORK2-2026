; os numeros sao todos definidos aleatoriamente e deverao ser conferidos
; de acordo com o que for pedido no momento da competicao

(define (problem bmt-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        cube1 cube2 cube3 cube4 cube5 - object             ; Trocar para número de april tags que devem ser pegas
        locstart loc1 loc2 loc3 loc4 locfinish - location  ; Trocar 1 para altura da location
        p1l1 p2l1 p1l2 p2l2 p1l3 - position                ; Trocar numero depois de l para location correspondente
    )

    (:init
        (= (carga robot1) 0)                     ; Robo sempre começa carregando 0 cubos

        (= (restantes loc1) 2)                   ; Inserir aqui apenas localizações que começam com cubos, e suas quantidades
        (= (restantes loc2) 1)
        (= (restantes loc3) 2)
        (= (restantes loc4) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube1 loc1)
        (obj-at cube2 loc1)
        (obj-at cube3 loc2)
        (obj-at cube4 loc3)
        (obj-at cube5 loc3)

        (position-free loc1 p1l1)
        (position-free loc1 p2l1)
        (position-free loc2 p1l2)
        (position-free loc2 p2l2)
        (position-free loc3 p1l3)

        (connected locstart loc1)   ; Dois métodos, se for fácil, tentar guiar caminho, senão colocar todos assim e ve o que é calulado
        (connected locstart loc2)
        (connected locstart loc3)
        (connected locstart loc4)
        (connected locstart locfinish)
        (connected loc1 locstart)
        (connected loc1 loc2)
        (connected loc1 loc3)
        (connected loc1 loc4)
        (connected loc1 locfinish)
        (connected loc2 locstart)
        (connected loc2 loc1)
        (connected loc2 loc3)
        (connected loc2 loc4)
        (connected loc2 locfinish)
        (connected loc3 locstart)
        (connected loc3 loc1)
        (connected loc3 loc2)
        (connected loc3 loc4)
        (connected loc3 locfinish)
        (connected loc4 locstart)
        (connected loc4 loc1)
        (connected loc4 loc2)
        (connected loc4 loc3)
        (connected loc4 locfinish)
        (connected locfinish locstart)
        (connected locfinish loc1)
        (connected locfinish loc2)
        (connected locfinish loc3)
        (connected locfinish loc4)
    )

    (:goal
        (and
            (obj-at cube1 loc2)
            (obj-at cube2 loc2)
            (obj-at cube3 loc3)
            (obj-at cube4 loc1)
            (obj-at cube5 loc1)
            (at-robot robot1 locfinish)
        )
    )
)