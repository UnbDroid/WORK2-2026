; Gerado por tools/yaml_para_pddl.py a partir de BMT (Basic Manipulation Test)
; duracao da prova: 6 min
; inicio: locstart1 | fim: locfinish1
;
; Areas do YAML -> locations do PDDL:
;   WS_3   -> loc10
;   WS_6   -> loc8
;
; Cubos (tag -> cor): 2=vermelho, 7=azul, 19=vermelho

(define (problem bmt-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart1 loc10 loc8 locfinish1 - location
        cube10 cube15 cube18 - object
        p1l6 p2l6 p3l6 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart1) 0)
        (= (restantes loc10) 3)
        (= (restantes loc8) 0)
        (= (restantes locfinish1) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart1)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube10 loc10)
        (obj-at cube15 loc10)
        (obj-at cube18 loc10)

        (position-free loc8 p1l6)
        (position-free loc8 p2l6)
        (position-free loc8 p3l6)

        (connected locstart1 loc10)
        (connected locstart1 loc8)
        (connected locstart1 locfinish1)
        (connected loc10 locstart1)
        (connected loc10 loc8)
        (connected loc10 locfinish1)
        (connected loc8 locstart1)
        (connected loc8 loc10)
        (connected loc8 locfinish1)
    )

    (:goal
        (and
            (obj-at cube10 loc8)   ; WS_6
            (at-position cube10 loc8 p3l6)
            (obj-at cube15 loc8)   ; WS_6
            (at-position cube15 loc8 p1l6)
            (obj-at cube18 loc8)   ; WS_6
            (at-position cube18 loc8 p2l6)
            (at-robot robot1 locfinish1)
        )
    )
)
