---
title: Robotics Project format
---

![[\[fig:FloorPlan\]]{#fig:FloorPlan label="fig:FloorPlan"}A generated
floorplan with rooms assigned.](Artboard 1.png){width="0.7\linewidth"}

![[\[fig:ObjectMap\]]{#fig:ObjectMap label="fig:ObjectMap"}A generated
floorplan objects placed in each
room.](Artboard 2.png){width="0.7\linewidth"}

![[\[fig:SimpleMap\]]{#fig:SimpleMap label="fig:SimpleMap"}A generated
and colour coded SimpleMap. The map has been fully
explored.](Screenshot 2023-11-10 173630.png){width="0.7\linewidth"}

Map Generation
==============

Use a variety of rectangles to construct a basic floor plan and assign
each space a room designation. See Figure Figure
[\[fig:FloorPlan\]](#fig:FloorPlan){reference-type="ref"
reference="fig:FloorPlan"}.

Object Assignment
=================

Based on the designation of a specific room (i.e kitchen) assign object
such that they are placed according to our desired spatial relational
graph. See Figure Figure
[\[fig:ObjectMap\]](#fig:ObjectMap){reference-type="ref"
reference="fig:ObjectMap"}.

Simultaneous Localisation And Mapping (SLAM)
============================================

Subscribers
-----------

-   Laserscan

-   Generated real map (GRM)

-   Particle cloud

Publishers
----------

-   estimated position

-   particle cloud

-   SimpleMap (SM)

Use SLAM as described in the lectures to explore the floor plan and
identify the robot's position, returning an additional SimpleMap that is
a grid representation of the explored floor plan.

Using a 100m x 100m space with a resolution of $0.2m^2$ per grid we end
up with an ndarray of size 500 x 500 to represent our simplified map
using integers to represent the state of each square. 0 means the square
is unknown, 1 means the square is explored but can be driven through and
2 means that it is explored but a wall has been found. See Figure
[\[fig:SimpleMap\]](#fig:SimpleMap){reference-type="ref"
reference="fig:SimpleMap"} for a colour coded view of a SimpleMap.

![[\[fig:Relations\]]{#fig:Relations label="fig:Relations"}The spatial
relations plotted on a map.](Artboard 3.png){width="0.7\linewidth"}

![[\[fig:HeatMap\]]{#fig:HeatMap label="fig:HeatMap"}A 2d slice of the
spatial relations used to form a heat
map](Artboard 4.png){width="0.7\linewidth"}

Object Position eStimation (OPS)
================================

Subscribers
-----------

-   SimpleMap

-   Estimated Position

-   Dict of all seen objects + estimated positions

Publishers
----------

-   location to travel to

Using the generated spatial relational graph the OPS section will
identify the next square the robot will travel to in an attempt to find
the desired object. See figure
[\[fig:Relations\]](#fig:Relations){reference-type="ref"
reference="fig:Relations"} for how the objects spatial relations will be
visualised.

All the modeled spatial relations will be used to form a 3d heat map
(see figure [\[fig:HeatMap\]](#fig:HeatMap){reference-type="ref"
reference="fig:HeatMap"} for where we estimate the desired object to be.
Then provided that the square can be navigated to as the crow flies from
an explored non wall square the square in question will be published.

In the event the array of objects contains the desired object then it
will be directly navigated to.

Simulated Visual Object Detection (SVOD)
========================================

Subscribers
-----------

-   Real Map

-   Estimated Position

-   Real Position

Publishers
----------

-   Dict of all seen objects + estimated positions

Simulate observing objects and publish the observed objects (store all
observed instances of all objects and cluster to form array of all found
items)

Real Visual Object Detection (RVOD)
===================================

Subscribers
-----------

-   Camera

-   Estimated Position

Publishers
----------

-   Dict of all seen objects + estimated positions

The same as SVOD but using camera for observing Objects.

Navigation
==========

Subscribers
-----------

-   location to travel to

A\* pathfind to location and the drive there.

Assignments
===========

-   SLAM: 2 people

-   OPS: 2 people

-   VODs + Navigation 1 person

-   Map Generation + Object Assignment TBD
