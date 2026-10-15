(define (problem smoke_unreachable)
  (:domain smoke)
  (:objects r1 - robot kitchen corridor bedroom - room)
  (:init
    (robot_at r1 kitchen)
    (connected kitchen corridor))
  (:goal (robot_at r1 bedroom))
)
