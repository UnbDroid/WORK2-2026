(define (domain robocup-work-transport)

  (:requirements :strips :typing)

  (:types
    robot
    location   ; workstations (WS_x), prateleiras (SH_x), mesa de precisão (PP), estado inicial (start)
    object     ; ATTCs e objetos ADVANCED (AprilTags)
    container  ; containers azuis e vermelhos
    slot       ; unidades de capacidade de armazenamento de cubos no robô (3))
  )

  (:predicates
    (at-robot ?r - robot ?l - location)
    (obj-at ?o - object ?l - location)                 ; objeto solto em uma localização
    (container-at ?c - container ?l - location)        ; container fixo em uma localização
    (in-container ?o - object ?c - container)           ; objeto dentro de um container
    (holding ?r - robot ?o - object)                    ; robô segurando o objeto
    (holding-in ?r - robot ?o - object ?s - slot)        ; vincula objeto ao slot de carga usado
    (slot-free ?r - robot ?s - slot)                     ; slot de carga disponível
    (on ?top - object ?bottom - object)                 ; ?top está empilhado sobre ?bottom
  )


  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (at-robot ?r ?from)
    :effect (and (not (at-robot ?r ?from)) (at-robot ?r ?to))
  )

  (:action pick-from-location
    :parameters (?r - robot ?o - object ?l - location ?s - slot)
    :precondition (and (at-robot ?r ?l) (obj-at ?o ?l) (slot-free ?r ?s))
    :effect (and
      (not (obj-at ?o ?l))
      (holding ?r ?o)
      (not (slot-free ?r ?s))
      (holding-in ?r ?o ?s))
  )

  (:action pick-from-container
    :parameters (?r - robot ?o - object ?c - container ?l - location ?s - slot)
    :precondition (and
      (at-robot ?r ?l)
      (container-at ?c ?l)
      (in-container ?o ?c)
      (slot-free ?r ?s))
    :effect (and
      (not (in-container ?o ?c))
      (holding ?r ?o)
      (not (slot-free ?r ?s))
      (holding-in ?r ?o ?s))
  )

  (:action place-at-location
    :parameters (?r - robot ?o - object ?l - location ?s - slot)
    :precondition (and (at-robot ?r ?l) (holding ?r ?o) (holding-in ?r ?o ?s))
    :effect (and
      (not (holding ?r ?o))
      (not (holding-in ?r ?o ?s))
      (slot-free ?r ?s)
      (obj-at ?o ?l))
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

  (:action stack
    :parameters (?r - robot ?top - object ?bottom - object ?l - location ?s - slot)
    :precondition (and
      (at-robot ?r ?l)
      (obj-at ?bottom ?l)
      (holding ?r ?top)
      (holding-in ?r ?top ?s))
    :effect (and
      (not (holding ?r ?top))
      (not (holding-in ?r ?top ?s))
      (slot-free ?r ?s)
      (obj-at ?top ?l)
      (on ?top ?bottom))
  )

  (:action unstack
    :parameters (?r - robot ?top - object ?bottom - object ?l - location ?s - slot)
    :precondition (and
      (at-robot ?r ?l)
      (on ?top ?bottom)
      (obj-at ?top ?l)
      (slot-free ?r ?s))
    :effect (and
      (not (on ?top ?bottom))
      (not (obj-at ?top ?l))
      (holding ?r ?top)
      (holding-in ?r ?top ?s))
  )
)
