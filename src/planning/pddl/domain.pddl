(define (domain robocup-work-transport))

  (:requirements :strips :typing :durative-actions)

  (:types
    robot
    location   ; workstations (WS_x), prateleiras (SH_x), mesa de precisão (PP), estado inicial (start)
    object     ; ATTCs e objetos ADVANCED (AprilTags)
    container  ; containers azuis e vermelhos
    slot       ; unidades de capacidade de armazenamento de cubos no robô (3)
  )

  (:predicates
    (at-robot ?r - robot ?l - location)
    (obj-at ?o - object ?l - location)                 ; objeto solto em uma localização
    (container-at ?c - container ?l - location)        ; container fixo em uma localização
    (in-container ?o - object ?c - container)           ; objeto dentro de um container
    (holding ?r - robot ?o - object)                    ; robô segurando o objeto
    (holding-in ?r - robot ?o - object ?s - slot)        ; vincula objeto ao slot de carga usado
    (slot-free ?r - robot ?s - slot)                     ; slot de carga disponível
    (on ?top - object ?bottom - object ?l - location)    ; ?top está empilhado sobre ?bottom e especifica onde eles ficam empilhados
    (gripper-free ?r - robot)                            ; novo predicado: garra/executor ocupado
  )


  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (at-robot ?r ?from)
    :effect (and (not (at-robot ?r ?from)) (at-robot ?r ?to))
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
      (at end (holding ?r ?o))
      (at end (not (slot-free ?r ?s)))
      (at end (holding-in ?r ?o ?s)))
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
      (at end (holding ?r ?o))
      (at end (not (slot-free ?r ?s)))
      (at end (holding-in ?r ?o ?s)))
  )

  (:durative-action place-at-location
  :parameters (?r - robot ?o - object ?l - location ?s - slot)
  :duration (= ?duration 1)
  :condition (and
    (at start (gripper-free ?r))
    (over all (at-robot ?r ?l))
    (at start (holding ?r ?o))
    (at start (holding-in ?r ?o ?s)))
  :effect (and
    (at start (not (gripper-free ?r)))
    (at end (gripper-free ?r))
    (at end (not (holding ?r ?o)))
    (at end (not (holding-in ?r ?o ?s)))
    (at end (slot-free ?r ?s))
    (at end (obj-at ?o ?l)))
)

  (:action place-in-container
    :parameters (?r - robot ?o - object ?c - container ?l - location ?s - slot)
    :precondition (and
      (at-robot ?r ?l)
      (container-at ?c ?l)
      (holding ?r ?o)
      (holding-in ?r ?o ?s))
    :effect (and
      (not (holding ?r ?o))
      (not (holding-in ?r ?o ?s))
      (slot-free ?r ?s)
      (in-container ?o ?c))
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
      (at end (on ?top ?bottom ?l)))
  )
)
