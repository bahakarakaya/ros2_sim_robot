# **Control System Package for the Mobile Robot and Its Robotic Arm**

This package provides a control system for a mobile robot and its robotic arm.

## **Control Mode Switching**  
Press *"Start"* button on the Xbox gamepad controller to switch between **vehicle mode** and **arm mode**.

## **Arm Mode Control**  
In *arm mode*, commands are published to the `/servo_server/delta_twist_cmds` topic for the **MoveIt Servo** node.
