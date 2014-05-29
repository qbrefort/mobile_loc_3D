Localization of a robot using interval analysis:

The robot retrieves a signal from 5 beacons. Within this signal is the distance from the robot to the beacon. We can compute 5 circles. With each circle, an error is given, due to the inaccuracy of the beacons. Thus we have 5 rings. The intersection of those rings is where the robot is.
The aim the this project is to find an outlier in those 5 rings. Indeed there is no solution for the intersection of the 5 rings.
With the spin box you can choose how many beacons are accurate to find a solution.

Requirements: Qtcreator, Ibex API (http://www.emn.fr/z-info/ibex/)
