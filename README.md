# Welcome to **Sunflower**! 🌻

Sunflower is an ESP IDF embedded software designed for control of solar trackers. It was designed specfically for my own use with custom 2 axis titlting solar tracker.


## Origins 🌱

The Sunflower project is a large part of my Master in Engineering thesis, where I prepared various sun tracking algorithms and analyzed their respective efficiency gain.

The project started back in 2020, after had rooftop solar panel array installed. Using the monitoring app we quickly discovered, that modules work well below maximum power point almost all the time. This inspired building of an experimental tracker, that would increase mornings' and evenings' power gains, as well as correct for the yearly maximum power point change (related to Sun's path change). Mechanical construction aged well, which was not the case for the electronic and software side.


## Main Features 💡

The main point was to create a reliable framework for further testing and development of solar tracking algorithms. The features include:
- driving of two brushed DC actuators,
- performing analog measurements,
- providing simple to use programming interface for tracking algorithms,
- ensuring safe operation with basic functional safety.


## Further Development 🏗️

I am unsure whether this software will find widespread use in Opensource Community. Nonetheless I'd like to create at least some documentation to simplify integration for other users. Moreover, I would like to improve API for motors control, to enable easy integration with different kinematic chains.


## Contributing ⚙️

If you find this project interesing, please star it to let me know!
Have any ideas? Feel free to open a discussion on GitHub or open a PR with your changes

## Licensing 📄

This project is licensed under the Apache License 2.0.

You are free to use, modify, and distribute this software — including for commercial purposes — as long as you:
- Keep the original license and copyright notice,
- Document any changes you make,
- Include a copy of the license file (LICENSE),
- Do not use the project's name or logo to endorse your work.

This license also includes a patent grant, which helps protect both users and contributors from patent-related issues.
For full details, see the LICENSE file.