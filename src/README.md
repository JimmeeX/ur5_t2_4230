# File Definitions && Purposes

- /plugins
    - ConveyorBeltPlugin - Allows user to control conveyor speed via srv
    - ProximityRayPlugin - Allows user to listen to break beam (proximity sensor) via topics
    - ROSConveyorBeltPlugin - Wrapper class of ConveyorBeltPlugin
    - ROSProximityRayPlugin - Wrapper class of ProximityRayPlugin
- spawner.py - Node responsible for object generation
- test_conveyor_beam.py - Node to test break beam and conveyor belt plugins.