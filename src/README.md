# File Definitions && Purposes

- /plugins
    - ConveyorBeltPlugin - Allows user to control conveyor speed via srv
    - ObjectDisposalPlugin - Allows objects to be deleted when it contacts a wall
    - ProximityRayPlugin - Allows user to listen to break beam (proximity sensor) via topics
    - ROSConveyorBeltPlugin - Wrapper class of ConveyorBeltPlugin
    - ROSProximityRayPlugin - Wrapper class of ProximityRayPlugin
    - ROSVacuumGripperPlugin - Wrapper class of VacuumGripperPlugin
    - SideContactPlugin - Helper class for ObjectDisposalPlugin
    - VacuumGripperPlugin - Allows enabling/disabling and attachment of gripper to objects
- motion.py - Responsible for Robot Arm movement
- order_manager.py - Responsible for the over-arching flow of the environment
- spawner.py - Spawns objects & containers
- vision.py - Object Detection
