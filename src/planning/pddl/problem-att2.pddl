; Gerado por tools/yaml_para_pddl.py a partir de ATT2 (Advanced Transportation Test II)
; duracao da prova: 20 min
; inicio: locstart1 | fim: locfinish1
;
; Areas do YAML -> locations do PDDL:
;   WS_2   -> loc2
;   WS_5   -> loc5
;   WS_6   -> loc6
;   SH_1   -> locshelf1, locshelf2
;   SH_4   -> locshelf7, locshelf8
;
; Cubos (tag -> cor): 5=azul, 10=vermelho, 12=azul, 22=vermelho, 29=azul

(define (problem att2-instance)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        locstart1 loc2 loc5 loc6 locshelf1 locshelf2 locshelf7 locshelf8 locfinish1 - location
        cube5 cube10 cube12 cube22 cube29 - object
        container_azul_loc5-2 container_vermelho_loc5-1 - container
        pf1l6 pf1lshelf8 - position
    )

    (:init
        (= (carga robot1) 0)

        (= (restantes locstart1) 0)
        (= (restantes loc2) 2)
        (= (restantes loc5) 0)
        (= (restantes loc6) 1)
        (= (restantes locshelf1) 2)
        (= (restantes locshelf2) 0)
        (= (restantes locshelf7) 0)
        (= (restantes locshelf8) 0)
        (= (restantes locfinish1) 0)

        (gripper-free robot1)
        (at-robot robot1 locstart1)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)

        (container-at container_azul_loc5-2 loc5)
        (container-at container_vermelho_loc5-1 loc5)

        (obj-at cube5 loc2)
        (obj-at cube10 locshelf1)
        (obj-at cube12 loc6)
        (obj-at cube22 loc2)
        (obj-at cube29 locshelf1)

        (position-free loc6 pf1l6)
        (position-free locshelf8 pf1lshelf8)

        (connected locstart1 loc2)
        (connected locstart1 loc5)
        (connected locstart1 loc6)
        (connected locstart1 locshelf1)
        (connected locstart1 locshelf2)
        (connected locstart1 locshelf7)
        (connected locstart1 locshelf8)
        (connected locstart1 locfinish1)
        (connected loc2 locstart1)
        (connected loc2 loc5)
        (connected loc2 loc6)
        (connected loc2 locshelf1)
        (connected loc2 locshelf2)
        (connected loc2 locshelf7)
        (connected loc2 locshelf8)
        (connected loc2 locfinish1)
        (connected loc5 locstart1)
        (connected loc5 loc2)
        (connected loc5 loc6)
        (connected loc5 locshelf1)
        (connected loc5 locshelf2)
        (connected loc5 locshelf7)
        (connected loc5 locshelf8)
        (connected loc5 locfinish1)
        (connected loc6 locstart1)
        (connected loc6 loc2)
        (connected loc6 loc5)
        (connected loc6 locshelf1)
        (connected loc6 locshelf2)
        (connected loc6 locshelf7)
        (connected loc6 locshelf8)
        (connected loc6 locfinish1)
        (connected locshelf1 locstart1)
        (connected locshelf1 loc2)
        (connected locshelf1 loc5)
        (connected locshelf1 loc6)
        (connected locshelf1 locshelf2)
        (connected locshelf1 locshelf7)
        (connected locshelf1 locshelf8)
        (connected locshelf1 locfinish1)
        (connected locshelf2 locstart1)
        (connected locshelf2 loc2)
        (connected locshelf2 loc5)
        (connected locshelf2 loc6)
        (connected locshelf2 locshelf1)
        (connected locshelf2 locshelf7)
        (connected locshelf2 locshelf8)
        (connected locshelf2 locfinish1)
        (connected locshelf7 locstart1)
        (connected locshelf7 loc2)
        (connected locshelf7 loc5)
        (connected locshelf7 loc6)
        (connected locshelf7 locshelf1)
        (connected locshelf7 locshelf2)
        (connected locshelf7 locshelf8)
        (connected locshelf7 locfinish1)
        (connected locshelf8 locstart1)
        (connected locshelf8 loc2)
        (connected locshelf8 loc5)
        (connected locshelf8 loc6)
        (connected locshelf8 locshelf1)
        (connected locshelf8 locshelf2)
        (connected locshelf8 locshelf7)
        (connected locshelf8 locfinish1)
    )

    (:goal
        (and
            (in-container cube5 container_azul_loc5-2)   ; WS_5 container azul
            (obj-at cube10 loc6)   ; WS_6
            (at-position cube10 loc6 pf1l6)
            (obj-at cube12 locshelf8)   ; SH_4
            (at-position cube12 locshelf8 pf1lshelf8)
            (in-container cube22 container_vermelho_loc5-1)   ; WS_5 container vermelho
            (in-container cube29 container_azul_loc5-2)   ; WS_5 container azul
            (at-robot robot1 locfinish1)
        )
    )
)
