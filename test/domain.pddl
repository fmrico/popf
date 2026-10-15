(define (domain smoke)
  (:requirements :strips :typing :durative-actions)
  (:types robot room)
  (:predicates
    (robot_at ?r - robot ?z - room)
    (connected ?z1 ?z2 - room))

  (:durative-action move
    :parameters (?r - robot ?z1 ?z2 - room)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?z1))
      (over all (connected ?z1 ?z2)))
    :effect (and
      (at start (not (robot_at ?r ?z1)))
      (at end (robot_at ?r ?z2))))
)
