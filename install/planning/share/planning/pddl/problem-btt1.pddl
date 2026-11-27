; Gerado por tools/yaml_para_pddl.py a partir de BTT1 (Basic Transportation Test I)
; duracao da prova: 10 min
; inicio: locstart1 | fim: locfinish1
;
; Areas do YAML -> locations do PDDL:
;   WS_1   -> loc1
;   WS_5   -> loc5
;   WS_7   -> loc7
;
; Cubos (tag -> cor): 5=azul, 7=azul, 27=azul

(define (problem btt1-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart1 loc1 loc5 loc7 locfinish1 - location
        cube5 cube7 cube27 - object
        pf1l1 pf1l5 pf1l7 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart1) 0)
        (= (restantes loc1) 1)
        (= (restantes loc5) 1)
        (= (restantes loc7) 1)
        (= (restantes locfinish1) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart1)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube5 loc5)
        (obj-at cube7 loc7)
        (obj-at cube27 loc1)

        (position-free loc1 pf1l1)
        (position-free loc5 pf1l5)
        (position-free loc7 pf1l7)

        (connected locstart1 loc1)
        (connected locstart1 loc5)
        (connected locstart1 loc7)
        (connected locstart1 locfinish1)
        (connected loc1 locstart1)
        (connected loc1 loc5)
        (connected loc1 loc7)
        (connected loc1 locfinish1)
        (connected loc5 locstart1)
        (connected loc5 loc1)
        (connected loc5 loc7)
        (connected loc5 locfinish1)
        (connected loc7 locstart1)
        (connected loc7 loc1)
        (connected loc7 loc5)
        (connected loc7 locfinish1)
    )

    (:goal
        (and
            (obj-at cube5 loc7)   ; WS_7
            (at-position cube5 loc7 pf1l7)
            (obj-at cube7 loc1)   ; WS_1
            (at-position cube7 loc1 pf1l1)
            (obj-at cube27 loc5)   ; WS_5
            (at-position cube27 loc5 pf1l5)
            (at-robot robot1 locfinish1)
        )
    )
)
