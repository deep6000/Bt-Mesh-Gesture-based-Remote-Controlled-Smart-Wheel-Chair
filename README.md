# Gesture-based-Remote-Controlled-Smart-Wheel-Chair

# PROBLEM BEING ADDRESSED 
The wheelchairs are usually controlled by a joystick attached to it. It is not possible to control the wheelchair remotely. It is not possible to provide maneuvering assistance if need be. We plan to implement remote maneuvering assistance. 

 # SOLUTION
 - Blue Gecko is used as a low power node which is interfaced with a gesture sensor. 
 - When a gesture is detected Low Power Node communicates with the Friend Node. 
 - Depending upon the gesture the Friend Node moves the wheelchair.  
 - We plan to overcome range limitation by using Mesh Relays. 
 - If the wheelchair is out of range of the Remote, the Remote(LPN) communicates with the nearest Friend Node which relays the message to the Wheelchair(Friend) the Remote wishes to control.
 - Central Device can be used to change the gesture parameters of LPN node and also the speed of the Wheelchair. 

# MODULES IMPLEMENTED
 ## Mesh Provisioning 
 - OOB Authentication is used where a OTP is displayed on LCD and user is asked to enter it.
 ## Gesture based Remote Control
- This a Low Power Node with Client Model.
-	Low Power Node responsible to send changes in a state to Friend Node.
- Generic Level State Server Model
## Services Implemented
- Service 1 : Level States- Left, Right, Forward, Backward 
- Service 2 : Level States - Motor Speed, Motor On/Off.



# PROJECT ACHIEVEMENTS
- Minimized power usage to microamperes by incorporating low power nodes for communication. 
- Improved security by implementing Man In Middle protection scheme using Out of Band Authentication 
- Enhanced reliability and range of Mesh by implementing features like Mesh Hopping and Persistent data. 
- Achieved safety by designing automatic break system in wheelchair on obstacle detection.
