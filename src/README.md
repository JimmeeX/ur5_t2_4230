# File Definitions && Purposes

- /plugins
    - ConveyorBeltPlugin - Allows user to control conveyor speed via srv
    - ProximityRayPlugin - Allows user to listen to break beam (proximity sensor) via topics
    - ROSConveyorBeltPlugin - Wrapper class of ConveyorBeltPlugin
    - ROSProximityRayPlugin - Wrapper class of ProximityRayPlugin
- motion.py - Responsible for Robot Arm movement
- order_manager.py - Responsible for the over-arching flow of the environment
- spawner.py - Spawns objects & containers
