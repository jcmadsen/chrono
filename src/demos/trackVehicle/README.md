Tracked Vehicle modeling library
Author: Justin Madsen, 20145
========

Build track vehicle models based on a logical heirarchy of subsystem components: | Vehicle | -> | TrackSystem | -> | sprocket, idler, suspension, powertrain, chain of shoes |.
Each subsystem is placed relative to its parent subsystem reference coordinate system.
Vehicles can be built with any number of TrackSysytems.
Tensioning devices are included in the idler subsystems.

Classes for driving the created track model using a GUI or non-interactively are provided.
Non-interactive drivers apply throttle and braking to the vehicle as input using any ChFunction.
GUI drivers work with runtime Irrlicht visualization, and take user input in the form of forward throttle/steering.
GUI drivers also print selected runtime model state output to the rendered window.
