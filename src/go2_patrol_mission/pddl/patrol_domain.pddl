(define (domain patrol)
  (:requirements :strips :typing :adl :durative-actions)

  (:types robot room)

  (:predicates
    (robot_at ?r - robot ?rm - room)
    (connected ?rm1 ?rm2 - room)
    (inspected ?rm - room)
  )

  (:durative-action navigate
    :parameters (?r - robot ?from ?to - room)
    :duration (= ?duration 10)
    :condition (and
      (at start (robot_at ?r ?from))
      (at start (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
    )
  )

  (:durative-action inspect
    :parameters (?r - robot ?rm - room)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?rm))
      (at start (not (inspected ?rm)))
    )
    :effect (and
      (at end (inspected ?rm))
    )
  )
)
