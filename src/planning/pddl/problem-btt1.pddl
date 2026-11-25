; Gerado por tools/yaml_para_pddl.py a partir de BTT1 (Basic Transportation Test I)
; duracao da prova: 10 min
;
; Areas do YAML -> locations do PDDL:
;   WS_1   -> loc1
;   WS_2   -> loc2
;   WS_3   -> loc3
;
; Cubos (tag -> cor): 4=azul, 11=vermelho, 23=azul

(define (problem btt1-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart loc1 loc2 loc3 locfinish - location
        cube4 cube11 cube23 - object
        p1l2 p2l2 p1l3 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart) 0)
        (= (restantes loc1) 3)
        (= (restantes loc2) 0)
        (= (restantes loc3) 0)
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube4 loc1)
        (obj-at cube11 loc1)
        (obj-at cube23 loc1)

        (position-free loc2 p1l2)
        (position-free loc2 p2l2)
        (position-free loc3 p1l3)

        (connected locstart loc1)
        (connected locstart loc2)
        (connected locstart loc3)
        (connected locstart locfinish)
        (connected loc1 locstart)
        (connected loc1 loc2)
        (connected loc1 loc3)
        (connected loc1 locfinish)
        (connected loc2 locstart)
        (connected loc2 loc1)
        (connected loc2 loc3)
        (connected loc2 locfinish)
        (connected loc3 locstart)
        (connected loc3 loc1)
        (connected loc3 loc2)
        (connected loc3 locfinish)
    )

    (:goal
        (and
            (obj-at cube4 loc2)   ; WS_2
            (at-position cube4 loc2 p2l2)
            (obj-at cube11 loc3)   ; WS_3
            (at-position cube11 loc3 p1l3)
            (obj-at cube23 loc2)   ; WS_2
            (at-position cube23 loc2 p1l2)
            (at-robot robot1 locfinish)
        )
    )
)
