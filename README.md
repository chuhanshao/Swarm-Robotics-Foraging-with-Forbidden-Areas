# Swarm-Robotics-Foraging-with-Forbidden-Areas

The goal of the robot swarm is to retrieve and transport items from the food source to the nest. The overall performance of the swarm is measured by the number of items it is able to collect during a ﬁxed experiment time. Each experiment is automatically terminated after 1000 seconds (10000 time-steps).

Five states are used to represent diﬀerent behaviour of robots, including WALK, AVOID, STOP, FORAGE and BACK.

## State Flow

- Exploring the Environment (Mark the nest and forbidden areas)
- Taking food
- Back to the Nest (Avoid the forbidden areas)
- Forage Again

## Pre-requirement: ARGoS

https://github.com/ilpincy/argos3
