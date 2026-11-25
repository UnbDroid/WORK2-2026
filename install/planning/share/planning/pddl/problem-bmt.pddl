; Gerado por tools/yaml_para_pddl.py a partir de BMT (Basic Manipulation Test)
; duracao da prova: 6 min
; inicio: locstart1 | fim: locfinish1
;
; Areas do YAML -> locations do PDDL:
;   WS_3   -> loc3
;   WS_5   -> loc5
;
; Cubos (tag -> cor): 10=vermelho, 15=vermelho, 18=vermelho

(define (problem bmt-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart1 loc3 loc5 locfinish1 - location
        cube10 cube15 cube18 - object
        p1l5 p2l5 p3l5 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart1) 0)
        (= (restantes loc3) 3)
        (= (restantes loc5) 0)
        (= (restantes locfinish1) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart1)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube10 loc3)
        (obj-at cube15 loc3)
        (obj-at cube18 loc3)

        (position-free loc5 p1l5)
        (position-free loc5 p2l5)
        (position-free loc5 p3l5)

        (connected locstart1 loc3)
        (connected locstart1 loc5)
        (connected locstart1 locfinish1)
        (connected loc3 locstart1)
        (connected loc3 loc5)
        (connected loc3 locfinish1)
        (connected loc5 locstart1)
        (connected loc5 loc3)
        (connected loc5 locfinish1)
    )

    (:goal
        (and
            (obj-at cube10 loc5)   ; WS_5
            (at-position cube10 loc5 p1l5)
            (obj-at cube15 loc5)   ; WS_5
            (at-position cube15 loc5 p2l5)
            (obj-at cube18 loc5)   ; WS_5
            (at-position cube18 loc5 p3l5)
            (at-robot robot1 locfinish1)
        )
    )
)
