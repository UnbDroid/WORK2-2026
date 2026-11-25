; Gerado por tools/yaml_para_pddl.py a partir de BMT (Basic Manipulation Test)
; duracao da prova: 6 min
; inicio: locstart1 | fim: locfinish1
;
; Areas do YAML -> locations do PDDL:
;   WS_3   -> loc3
;   WS_4   -> loc4
;
; Cubos (tag -> cor): 5=azul, 13=azul, 22=vermelho, 23=vermelho

(define (problem bmt-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart1 loc3 loc4 locfinish1 - location
        cube5 cube13 cube22 cube23 - object
        p1l3 p1l4 p2l4 p3l4 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart1) 0)
        (= (restantes loc3) 3)
        (= (restantes loc4) 0)
        (= (restantes locfinish1) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart1)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube5 loc3)
        (at-position cube5 loc3 p1l3)
        (obj-at cube13 loc3)
        (obj-at cube22 loc3)
        (obj-at cube23 loc3)

        (position-free loc4 p1l4)
        (position-free loc4 p2l4)
        (position-free loc4 p3l4)

        (connected locstart1 loc3)
        (connected locstart1 loc4)
        (connected locstart1 locfinish1)
        (connected loc3 locstart1)
        (connected loc3 loc4)
        (connected loc3 locfinish1)
        (connected loc4 locstart1)
        (connected loc4 loc3)
        (connected loc4 locfinish1)
    )

    (:goal
        (and
            (obj-at cube5 loc3)   ; WS_3
            (at-position cube5 loc3 p1l3)
            (obj-at cube13 loc4)   ; WS_4
            (at-position cube13 loc4 p3l4)
            (obj-at cube22 loc4)   ; WS_4
            (at-position cube22 loc4 p2l4)
            (obj-at cube23 loc4)   ; WS_4
            (at-position cube23 loc4 p1l4)
            (at-robot robot1 locfinish1)
        )
    )
)
