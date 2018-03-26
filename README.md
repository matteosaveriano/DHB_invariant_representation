# DHB_invariant_representation

## Components description
The folder _DHB_Invariants_ contains 3 scripts:
- ```computeDHB.m```: Compute DHB invariants given a Cartesian trajectory.
- ```reconstructTrajectory.m```: Reconstruct a Cartesian trajectory from its DHB invariant representation.
- ```mainDHB.m```: Simple demo script.

The code is compatible with Matlab and Octave.

## Software Requirements
The code is developed and tested under _Matlab 2015b_.

## Usage
We prepared a simple demo in ```mainDHB.m``` to show how a Cartesian 6D trajectory is transformed into a set of DHB invariants and vice versa.

## References
Please acknowledge the authors in any acedemic publication that used parts of these codes.
```
@article{Lee2018,
    title = "Bidirectional invariant representation of rigid body motions and its application to gesture recognition and reproduction",
    author = "Lee, Dongheui and Soloperto, Raffaele and Saveriano, Matteo",
    journal = "Autonomous Robots",
    year = "2018",
    volume = "42",
    number = "1",
    pages = "125--145"
}

@article{Soloperto2015,
    title = " A Bidirectional Invariant Representation of Motion for Gesture Recognition and Reproduction",
    author = "Soloperto, Raffaele and Saveriano, Matteo and Lee, Dongheui",
    journal = " International Conference on Robotics and Automation (ICRA)",
    year = "2015",
    pages = "6146-6152"
}
```

## Licence
This repository contains free software: you can redistribute it and/or modify it under the terms of the GNU General Public License version 3 as published by the Free Software Foundation.

This source code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this code. If not, see http://www.gnu.org/licenses/.
