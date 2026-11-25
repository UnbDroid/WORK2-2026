; Gerado por tools/yaml_para_pddl.py a partir de BMT (Basic Manipulation Test)
; duracao da prova: 6 min
;
; Areas do YAML -> locations do PDDL:
;   WS_3   -> loc3
;   WS_6   -> loc6
;
; Cubos (tag -> cor): 2=vermelho, 7=azul, 19=vermelho

(define (problem bmt-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart1 loc7 loc8 locfinish1 - location
        cube2 cube7 cube19 - object
        p1l6 p2l6 p3l6 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart1) 0)
        (= (restantes loc10) 0)
        (= (restantes loc8) 3)
        (= (restantes locfinish1) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart1)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube2 loc10)
        (obj-at cube7 loc10)
        (obj-at cube19 loc10)

        (position-free loc8 p1l6)
        (position-free loc8 p2l6)
        (position-free loc8 p3l6)

        (connected locstart loc10)
        (connected locstart loc8)
        (connected locstart locfinish)
        (connected loc10 locstart)
        (connected loc10 loc8)
        (connected loc10 locfinish)
        (connected loc8 locstart)
        (connected loc8 loc10)
        (connected loc8 locfinish)
    )

    (:goal
        (and
            (obj-at cube2 loc8)   ; WS_6
            (at-position cube2 loc8 p3l6)
            (obj-at cube7 loc8)   ; WS_6
            (at-position cube7 loc8 p1l6)
            (obj-at cube19 loc8)   ; WS_6
            (at-position cube19 loc8 p2l6)
            (at-robot robot1 locfinish)
        )
    )
)
