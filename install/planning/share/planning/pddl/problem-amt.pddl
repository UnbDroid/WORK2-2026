; Gerado por tools/yaml_para_pddl.py a partir de AMT (Advanced Manipulation Test)
; duracao da prova: 15 min
;
; Areas do YAML -> locations do PDDL:
;   PP     -> locpp1
;
; Cubos (tag -> cor): 1=azul, 2=vermelho, 3=vermelho, 4=azul, 5=vermelho, 6=azul
;
; Para place_at_cavity_two_action.py (cavidade de destino de cada cubo):
;   ZONA_DO_CUBO = {1: 'zone101', 2: 'zone102', 3: 'zone103', 4: 'zone104', 5: 'zone105', 6: 'zone106'}

(define (problem amt-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart locpp1 locfinish - location
        cube1 cube2 cube3 cube4 cube5 cube6 - object
        cav101 cav102 cav103 cav104 cav105 cav106 cav107 - cavity
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart) 0)
        (= (restantes locpp1) 6)
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (obj-at cube1 locpp1)
        (obj-at cube2 locpp1)
        (obj-at cube3 locpp1)
        (obj-at cube4 locpp1)
        (obj-at cube5 locpp1)
        (obj-at cube6 locpp1)

        (cavity-free locpp1 cav101)
        (cavity-free locpp1 cav102)
        (cavity-free locpp1 cav103)
        (cavity-free locpp1 cav104)
        (cavity-free locpp1 cav105)
        (cavity-free locpp1 cav106)
        (cavity-free locpp1 cav107)

        (connected locstart locpp1)
        (connected locstart locfinish)
        (connected locpp1 locstart)
        (connected locpp1 locfinish)
    )

    (:goal
        (and
            (in-cavity cube1 locpp1 cav101)   ; PP slot 101
            (in-cavity cube2 locpp1 cav102)   ; PP slot 102
            (in-cavity cube3 locpp1 cav103)   ; PP slot 103
            (in-cavity cube4 locpp1 cav104)   ; PP slot 104
            (in-cavity cube5 locpp1 cav105)   ; PP slot 105
            (in-cavity cube6 locpp1 cav106)   ; PP slot 106
            (at-robot robot1 locfinish)
        )
    )
)
