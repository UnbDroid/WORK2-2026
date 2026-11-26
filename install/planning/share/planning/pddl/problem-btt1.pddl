; Gerado por tools/yaml_para_pddl.py a partir de BTT1 (Basic Transportation Test I)
; duracao da prova: 10 min
; inicio: locstart1 | fim: locfinish1
;
; Areas do YAML -> locations do PDDL:
;   WS_7   -> loc7
;   WS_8   -> loc8
;   WS_10  -> loc10
;
; Cubos (tag -> cor): 4=vermelho, 11=azul, 16=vermelho, 25=vermelho

(define (problem btt1-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart1 loc7 loc8 loc10 locfinish1 - location
        cube4 cube11 cube16 cube25 - object
        p1l7 p1l8 p2l8 p1l10 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart1) 0)
        (= (restantes loc7) 3)
        (= (restantes loc8) 1)
        (= (restantes loc10) 0)
        (= (restantes locfinish1) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart1)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube4 loc7)
        (obj-at cube11 loc7)
        (obj-at cube16 loc8)
        (obj-at cube25 loc7)

        (position-free loc7 p1l7)
        (position-free loc8 p1l8)
        (position-free loc8 p2l8)
        (position-free loc10 p1l10)

        (connected locstart1 loc7)
        (connected locstart1 loc8)
        (connected locstart1 loc10)
        (connected locstart1 locfinish1)
        (connected loc7 locstart1)
        (connected loc7 loc8)
        (connected loc7 loc10)
        (connected loc7 locfinish1)
        (connected loc8 locstart1)
        (connected loc8 loc7)
        (connected loc8 loc10)
        (connected loc8 locfinish1)
        (connected loc10 locstart1)
        (connected loc10 loc7)
        (connected loc10 loc8)
        (connected loc10 locfinish1)
    )

    (:goal
        (and
            (obj-at cube4 loc8)   ; WS_8
            (at-position cube4 loc8 p2l8)
            (obj-at cube11 loc10)   ; WS_10
            (at-position cube11 loc10 p1l10)
            (obj-at cube16 loc7)   ; WS_7
            (at-position cube16 loc7 p1l7)
            (obj-at cube25 loc8)   ; WS_8
            (at-position cube25 loc8 p1l8)
            (at-robot robot1 locfinish1)
        )
    )
)
