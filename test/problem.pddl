(define (problem smoke_reachable)
  (:domain smoke)
  (:objects r1 - robot kitchen corridor bedroom - room)
  (:init
    (robot_at r1 kitchen)
    (connected kitchen corridor)
    (connected corridor bedroom))
  (:goal (robot_at r1 bedroom))
)
