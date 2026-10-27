(define (domain robocup-work-transport)

  (:requirements :strips :typing :durative-actions :fluents)

  (:types
    robot
    location   ; workstations (WS_x), prateleiras (SH_x), mesa de precisão (PP), estado inicial (start)
    object     ; ATTCs e objetos ADVANCED (AprilTags)
    container  ; containers azuis e vermelhos
    slot       ; unidades de capacidade de armazenamento de cubos no robô (3)
    position    ; três posições possíveis na mesa para deixar três cubos
  )

  (:predicates
    (at-robot ?r - robot ?l - location)
    (obj-at ?o - object ?l - location)                 ; objeto solto em uma localização
    (container-at ?c - container ?l - location)        ; container fixo em uma localização
    (in-container ?o - object ?c - container)           ; objeto dentro de um container
    (holding-in ?r - robot ?o - object ?s - slot)        ; vincula objeto ao slot de carga usado
    (slot-free ?r - robot ?s - slot)                     ; slot de carga disponível
    (position-free ?l - location ?p - position)          ; posições de deixar os cubos disponíveis
    (on ?top - object ?bottom - object ?l - location)    ; ?top está empilhado sobre ?bottom e especifica onde eles ficam empilhados
    (gripper-free ?r - robot)                            ; novo predicado: garra/executor ocupado
    (connected ?from - location ?to - location)
  )

  (:functions
    (carga ?r - robot)         ; cubos que o robô carrega agora (0 a 3)
    (restantes ?l - location)  ; cubos que ainda faltam coletar em ?l (0 nos locais que não são de coleta tirando locstart e locfinish)
  )

  (:durative-action move-cheio
    :parameters (?r - robot ?from - location ?to - location)
    :duration (= ?duration 5)
    :condition (and
      (at start (at-robot ?r ?from))
      (over all (connected ?from ?to))
      (at start (gripper-free ?r))
      (over all (gripper-free ?r))
      (at start (>= (carga ?r) 3))
    )
    :effect (and
      (at start (not (at-robot ?r ?from)))
      (at end (at-robot ?r ?to)))
  )

  (:durative-action move-nao-cheio
    :parameters (?r - robot ?from - location ?to - location)
    :duration (= ?duration 5)
    :condition (and
      (at start (at-robot ?r ?from))
      (over all (connected ?from ?to))
      (at start (gripper-free ?r))
      (over all (gripper-free ?r))
      (at start (<= (restantes ?from) 0))
    )
    :effect (and
      (at start (not (at-robot ?r ?from)))
      (at end (at-robot ?r ?to)))
  )

  (:durative-action pick-from-location
    :parameters (?r - robot ?o - object ?l - location ?s - slot)
    :duration (= ?duration 1)
    :condition (and 
      (at start (gripper-free ?r))
      (over all (at-robot ?r ?l))
      (at start (obj-at ?o ?l)) 
      (at start (slot-free ?r ?s)))
    :effect (and
      (at start (not (gripper-free ?r)))
      (at end (gripper-free ?r))
      (at end (not (obj-at ?o ?l)))
      (at end (not (slot-free ?r ?s)))
      (at end (holding-in ?r ?o ?s))
      (at end (increase (carga ?r) 1))        ; Aumenta a carga do robô em um, limite três
      (at end (decrease (restantes ?l) 1)))   ; Retira um cubo do total na loc especificada
  )

  (:durative-action pick-from-container
    :parameters (?r - robot ?o - object ?c - container ?l - location ?s - slot)
    :duration (= ?duration 1)
    :condition (and
      (at start (gripper-free ?r))
      (over all (at-robot ?r ?l))
      (over all (container-at ?c ?l))
      (at start (in-container ?o ?c))
      (at start (slot-free ?r ?s)))
    :effect (and
      (at start (not (gripper-free ?r)))
      (at end (gripper-free ?r))
      (at end (not (in-container ?o ?c)))
      (at end (not (slot-free ?r ?s)))
      (at end (holding-in ?r ?o ?s))
      (at end (increase (carga ?r) 1))
      (at end (decrease (restantes ?l) 1)))
  )

  (:durative-action place-at-location
  :parameters (?r - robot ?o - object ?l - location ?s - slot ?p - position)
  :duration (= ?duration 1)
  :condition (and
    (at start (gripper-free ?r))
    (over all (at-robot ?r ?l))
    (at start (holding-in ?r ?o ?s))
    (at start (position-free ?l ?p)))
  :effect (and
    (at start (not (gripper-free ?r)))
    (at end (gripper-free ?r))
    (at end (not (holding-in ?r ?o ?s)))
    (at end (slot-free ?r ?s))
    (at end (obj-at ?o ?l))
    (at end (not (position-free ?l ?p)))
    (at end (decrease (carga ?r) 1)))
)

  (:durative-action place-in-container
    :parameters (?r - robot ?o - object ?c - container ?l - location ?s - slot)
    :duration (= ?duration 1)
    :condition (and
      (at start (gripper-free ?r))
      (over all (at-robot ?r ?l))
      (over all (container-at ?c ?l))
      (at start (holding-in ?r ?o ?s)))
    :effect (and
      (at start (not (gripper-free ?r)))
      (at end (gripper-free ?r))
      (at end (not (holding-in ?r ?o ?s)))
      (at end (slot-free ?r ?s))
      (at end (in-container ?o ?c))
      (at end (decrease (carga ?r) 1)))
  )

  (:durative-action stack
    :parameters (?r - robot ?bottom - object ?top - object ?l - location ?bottomslot - slot ?topslot - slot)
    :duration (= ?duration 1)
    :condition (and
      (at start (gripper-free ?r))
      (over all (at-robot ?r ?l))
      (at start (holding-in ?r ?top ?topslot))
      (at start (holding-in ?r ?bottom ?bottomslot)))
    :effect (and
      (at start (not (gripper-free ?r)))
      (at end (gripper-free ?r))
      (at end (not (holding-in ?r ?top ?topslot)))
      (at end (not (holding-in ?r ?bottom ?bottomslot)))
      (at end (slot-free ?r ?topslot))
      (at end (slot-free ?r ?bottomslot))
      (at end (on ?top ?bottom ?l))
      (at end (decrease (carga ?r) 2)))
  )
)