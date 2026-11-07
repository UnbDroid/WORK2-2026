; os numeros sao todos definidos aleatoriamente e deverao ser conferidos
; de acordo com o que for pedido no momento da competicao

(define (problem btt2-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        cube1 cube2 cube3 - object                         ; Trocar para número de april tags que devem ser pegas
        locstart loc1 loc4 loc7 locfinish - location       ; Trocar 1 para altura da location
        p1l1 p2l1 p1l4 p2l4 p1l7 p2l7 - position           ; Trocar numero depois de l para location correspondente
    )

    (:init
        (= (carga robot1) 0)                     ; Robo sempre começa carregando 0 cubos

        (= (restantes locstart) 0)
        (= (restantes loc1) 1)
        (= (restantes loc4) 1)
        (= (restantes loc7) 1)

        (gripper-free robot1)
        (at-robot robot1 locstart)

        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube1 loc1)
        (obj-at cube2 loc4)
        (obj-at cube3 loc7)

        (position-free loc1 p2l1)
        (position-free loc4 p2l4)
        (position-free loc7 p2l7)

        (connected locstart loc1)   ; Dois métodos, se for fácil, tentar guiar caminho, senão colocar todos assim e ve o que é calulado
        (connected locstart loc4)
        (connected locstart loc7)
        (connected locstart locfinish)
        (connected loc1 locstart)
        (connected loc1 loc4)
        (connected loc1 loc7)
        (connected loc1 locfinish)
        (connected loc4 locstart)
        (connected loc4 loc1)
        (connected loc4 loc7)
        (connected loc4 locfinish)
        (connected loc7 locstart)
        (connected loc7 loc1)
        (connected loc7 loc4)
        (connected loc7 locfinish)
        (connected locfinish locstart)
        (connected locfinish loc1)
        (connected locfinish loc4)
        (connected locfinish loc7)
    )

    (:goal
        (and
            (obj-at cube1 loc4)
            (obj-at cube2 loc7)
            (obj-at cube3 loc1)
            (at-robot robot1 locfinish)
        )
    )
)