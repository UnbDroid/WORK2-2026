; Gerado por tools/yaml_para_pddl.py a partir de BTT2 (Basic Transportation Test II)
; duracao da prova: 10 min
; inicio: locstart1 | fim: locfinish1
;
; Areas do YAML -> locations do PDDL:
;   WS_2   -> loc2
;   WS_3   -> loc3
;   WS_6   -> loc6
;   WS_7   -> loc7
;
; Cubos (tag -> cor): 3=vermelho, 8=vermelho, 14=vermelho, 20=vermelho, 23=vermelho

(define (problem btt2-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart1 loc2 loc3 loc6 loc7 locfinish1 - location
        cube3 cube8 cube14 cube20 cube23 - object
        pf1l2 pf1l3 pf2l3 pf1l6 pf1l7 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart1) 0)
        (= (restantes loc2) 1)
        (= (restantes loc3) 1)
        (= (restantes loc6) 1)
        (= (restantes loc7) 2)
        (= (restantes locfinish1) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart1)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube3 loc7)
        (obj-at cube8 loc6)
        (obj-at cube14 loc2)
        (obj-at cube20 loc3)
        (obj-at cube23 loc7)

        (position-free loc2 pf1l2)
        (position-free loc3 pf1l3)
        (position-free loc3 pf2l3)
        (position-free loc6 pf1l6)
        (position-free loc7 pf1l7)

        (connected locstart1 loc2)
        (connected locstart1 loc3)
        (connected locstart1 loc6)
        (connected locstart1 loc7)
        (connected locstart1 locfinish1)
        (connected loc2 locstart1)
        (connected loc2 loc3)
        (connected loc2 loc6)
        (connected loc2 loc7)
        (connected loc2 locfinish1)
        (connected loc3 locstart1)
        (connected loc3 loc2)
        (connected loc3 loc6)
        (connected loc3 loc7)
        (connected loc3 locfinish1)
        (connected loc6 locstart1)
        (connected loc6 loc2)
        (connected loc6 loc3)
        (connected loc6 loc7)
        (connected loc6 locfinish1)
        (connected loc7 locstart1)
        (connected loc7 loc2)
        (connected loc7 loc3)
        (connected loc7 loc6)
        (connected loc7 locfinish1)
    )

    (:goal
        (and
            (obj-at cube3 loc6)   ; WS_6
            (at-position cube3 loc6 pf1l6)
            (obj-at cube8 loc2)   ; WS_2
            (at-position cube8 loc2 pf1l2)
            (obj-at cube14 loc3)   ; WS_3
            (at-position cube14 loc3 pf2l3)
            (obj-at cube20 loc7)   ; WS_7
            (at-position cube20 loc7 pf1l7)
            (obj-at cube23 loc3)   ; WS_3
            (at-position cube23 loc3 pf1l3)
            (at-robot robot1 locfinish1)
        )
    )
)
