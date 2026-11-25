; Gerado por tools/yaml_para_pddl.py a partir de ATT2 (Advanced Transportation Test II)
; duracao da prova: 20 min
;
; Areas do YAML -> locations do PDDL:
;   WS_2   -> loc2
;   WS_5   -> loc5
;   WS_6   -> loc6
;   SH_1   -> locshelf1, locshelf2
;   SH_4   -> locshelf7, locshelf8
;   PP     -> locpp1
;
; Cubos (tag -> cor): 5=azul, 10=vermelho, 12=azul, 17=vermelho, 22=vermelho, 29=azul
;
; Para place_at_cavity_two_action.py (cavidade de destino de cada cubo):
;   ZONA_DO_CUBO = {17: 'zone102'}
; AVISO: locpp1: cubos [17] chegam de outra area para um slot da mesa de precisao. O domain nao tem acao "segurando -> cavidade"; o plano vai usar place-at-location (posicao pfNlpp) + place-at-cavity, e o node place_at_cavity_two hoje roda a rotina de ordenacao do AMT. Falta uma acao propria para isso.

(define (problem att2-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart loc2 loc5 loc6 locshelf1 locshelf2 locshelf7 locshelf8 locpp1 locfinish - location
        cube5 cube10 cube12 cube17 cube22 cube29 - object
        container_azul_loc5-2 container_vermelho_loc5-1 - container
        p1l6 p1lshelf8 pf1lpp1 - position
        cav101 cav102 cav103 cav104 cav105 cav106 cav107 - cavity
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart) 0)
        (= (restantes loc2) 3)
        (= (restantes loc5) 0)
        (= (restantes loc6) 1)
        (= (restantes locshelf1) 2)
        (= (restantes locshelf2) 0)
        (= (restantes locshelf7) 0)
        (= (restantes locshelf8) 0)
        (= (restantes locpp1) 0)
        (= (restantes locfinish) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (container-at container_azul_loc5-2 loc5)
        (container-at container_vermelho_loc5-1 loc5)

        (obj-at cube5 loc2)
        (obj-at cube10 locshelf1)
        (obj-at cube12 loc6)
        (obj-at cube17 loc2)
        (obj-at cube22 loc2)
        (obj-at cube29 locshelf1)

        (position-free loc6 p1l6)
        (position-free locshelf8 p1lshelf8)
        (position-free locpp1 pf1lpp1)
        (cavity-free locpp1 cav101)
        (cavity-free locpp1 cav102)
        (cavity-free locpp1 cav103)
        (cavity-free locpp1 cav104)
        (cavity-free locpp1 cav105)
        (cavity-free locpp1 cav106)
        (cavity-free locpp1 cav107)

        (connected locstart loc2)
        (connected locstart loc5)
        (connected locstart loc6)
        (connected locstart locshelf1)
        (connected locstart locshelf2)
        (connected locstart locshelf7)
        (connected locstart locshelf8)
        (connected locstart locpp1)
        (connected locstart locfinish)
        (connected loc2 locstart)
        (connected loc2 loc5)
        (connected loc2 loc6)
        (connected loc2 locshelf1)
        (connected loc2 locshelf2)
        (connected loc2 locshelf7)
        (connected loc2 locshelf8)
        (connected loc2 locpp1)
        (connected loc2 locfinish)
        (connected loc5 locstart)
        (connected loc5 loc2)
        (connected loc5 loc6)
        (connected loc5 locshelf1)
        (connected loc5 locshelf2)
        (connected loc5 locshelf7)
        (connected loc5 locshelf8)
        (connected loc5 locpp1)
        (connected loc5 locfinish)
        (connected loc6 locstart)
        (connected loc6 loc2)
        (connected loc6 loc5)
        (connected loc6 locshelf1)
        (connected loc6 locshelf2)
        (connected loc6 locshelf7)
        (connected loc6 locshelf8)
        (connected loc6 locpp1)
        (connected loc6 locfinish)
        (connected locshelf1 locstart)
        (connected locshelf1 loc2)
        (connected locshelf1 loc5)
        (connected locshelf1 loc6)
        (connected locshelf1 locshelf2)
        (connected locshelf1 locshelf7)
        (connected locshelf1 locshelf8)
        (connected locshelf1 locpp1)
        (connected locshelf1 locfinish)
        (connected locshelf2 locstart)
        (connected locshelf2 loc2)
        (connected locshelf2 loc5)
        (connected locshelf2 loc6)
        (connected locshelf2 locshelf1)
        (connected locshelf2 locshelf7)
        (connected locshelf2 locshelf8)
        (connected locshelf2 locpp1)
        (connected locshelf2 locfinish)
        (connected locshelf7 locstart)
        (connected locshelf7 loc2)
        (connected locshelf7 loc5)
        (connected locshelf7 loc6)
        (connected locshelf7 locshelf1)
        (connected locshelf7 locshelf2)
        (connected locshelf7 locshelf8)
        (connected locshelf7 locpp1)
        (connected locshelf7 locfinish)
        (connected locshelf8 locstart)
        (connected locshelf8 loc2)
        (connected locshelf8 loc5)
        (connected locshelf8 loc6)
        (connected locshelf8 locshelf1)
        (connected locshelf8 locshelf2)
        (connected locshelf8 locshelf7)
        (connected locshelf8 locpp1)
        (connected locshelf8 locfinish)
        (connected locpp1 locstart)
        (connected locpp1 loc2)
        (connected locpp1 loc5)
        (connected locpp1 loc6)
        (connected locpp1 locshelf1)
        (connected locpp1 locshelf2)
        (connected locpp1 locshelf7)
        (connected locpp1 locshelf8)
        (connected locpp1 locfinish)
    )

    (:goal
        (and
            (in-container cube5 container_azul_loc5-2)   ; WS_5 container azul
            (obj-at cube10 loc6)   ; WS_6
            (at-position cube10 loc6 p1l6)
            (obj-at cube12 locshelf8)   ; SH_4
            (at-position cube12 locshelf8 p1lshelf8)
            (in-cavity cube17 locpp1 cav102)   ; PP slot 102
            (in-container cube22 container_vermelho_loc5-1)   ; WS_5 container vermelho
            (in-container cube29 container_azul_loc5-2)   ; WS_5 container azul
            (at-robot robot1 locfinish)
        )
    )
)
