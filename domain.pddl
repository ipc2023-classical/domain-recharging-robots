(define (domain recharging-robots)
(:requirements :typing :adl :action-costs)
(:types
    location - object
    robot - object
    battery-level - object
    config - object
)

(:predicates
    (BATTERY-PREDECESSOR ?f1 - battery-level ?f2 - battery-level)
    (CONNECTED ?l1 - location ?l2 - location)
    (GUARD-CONFIG ?c - config ?l - location)
    (at ?r - robot ?l - location)
    (stopped ?r - robot)
    (battery ?r - robot ?f - battery-level)
    (guarded ?l - location)
    (config-fullfilled ?c - config)
)

(:functions
    (move-cost) - number
    (recharge-cost) - number
    (total-cost) - number
)

(:action move
    :parameters (?r - robot ?from - location ?to - location
                 ?fpre - battery-level ?fpost - battery-level)
    :precondition
        (and
            (not (stopped ?r))
            (at ?r ?from)
            (battery ?r ?fpre)
            (BATTERY-PREDECESSOR ?fpost ?fpre)
            (or (CONNECTED ?from ?to) (CONNECTED ?to ?from))
        )
    :effect
        (and
            (not (at ?r ?from))
            (at ?r ?to)
            (not (battery ?r ?fpre))
            (battery ?r ?fpost)
            (increase (total-cost) (move-cost))
        )
)

(:action recharge
    :parameters (?rfrom - robot ?rto - robot ?loc - location
                 ?fpre-from - battery-level ?fpost-from - battery-level
                 ?fpre-to - battery-level ?fpost-to - battery-level)
    :precondition
        (and
            (not (= ?rfrom ?rto))
            (at ?rfrom ?loc)
            (at ?rto ?loc)
            (battery ?rfrom ?fpre-from)
            (battery ?rto ?fpre-to)
            (BATTERY-PREDECESSOR ?fpost-from ?fpre-from)
            (BATTERY-PREDECESSOR ?fpre-to ?fpost-to)
        )
    :effect
        (and
            (not (battery ?rfrom ?fpre-from))
            (battery ?rfrom ?fpost-from)
            (not (battery ?rto ?fpre-to))
            (battery ?rto ?fpost-to)
            (increase (total-cost) (recharge-cost))
        )
)

(:action stop-and-guard
    :parameters (?r - robot ?l - location)
    :precondition
        (and
            (not (stopped ?r))
            (at ?r ?l)
        )
    :effect
        (and
            (stopped ?r)
            (guarded ?l)
            (forall (?l2 - location)
                (when (or (CONNECTED ?l ?l2) (CONNECTED ?l2 ?l))
                      (guarded ?l2)
                )
            )
        )
)

(:action verify-guard-config
    :parameters (?c - config)
    :precondition
        (and
            (forall (?l - location)
                (imply (GUARD-CONFIG ?c ?l) (guarded ?l))
            )
        )
    :effect
        (and
            (forall (?r - robot) (not (stopped ?r)))
            (forall (?l - location) (not (guarded ?l)))
            (config-fullfilled ?c)
        )
)

)
