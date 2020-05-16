This is the code repository for UVLED Disinfection Robot. 

<p align="center">
<a href="https://ibb.co/D4XNSm0"><img src="https://i.ibb.co/rkPg1qD/IMG-20200511-113317.jpg" alt="IMG-20200511-113317" border="0" /></a>
</p>

**Group Members**: 

- Shivangi Gambhir 
- Paridhi Gupta 
- Peicheng Lu
- Boyang Si

This project is focused on creating an autonomous robot to disinfect various surfaces that have lingering bacteria on them using deep ultraviolet light. A programmable robot is used to perform these tasks since humans should not come in contact with deep UV light. The code in this repository is organised in the following way:

1. Navigation code present in _UVLED-Disinfection-Robot/Downloads/MEng/moveToCorner.py_
2. Robotic Arm code present in _UVLED-Disinfection-Robot/Downloads/MEng/updated_armcode/ultrasound.py_

**Instructions**  

**Setup**: 

1. **Running the code**
  
-- Clone the repo : https://github.com/yanray/UVLED-Disinfection-Robot.git  
-- SSH into the Raspberry Pi       
-- Double check to make sure Roomba is ON      
-- Connect the servo controller board to the power supply  
     
   **To run moveToCorner.py**:   
      - Go to the folder mentioned above    
   _After running moveToConer.py_:   
      - Roomba will begin to wander until the front ultrasound detects an object that is close i.e. within 25 cm   
      - Then Roomba will move to bottom left corner of table   
      - Roomba begins lawn mowing  
      - After traversing table, Roomba goes back to exploring
                 
  **To run ultrasound.py**:  
      - Go to the folder mentioned above   
   _After running ultrasound.py_:   
      - The robotic arm will be in an initial/reset position and the visible LEDs are OFF  
      - When the roomba/arm is brought under the table, the ultrasound sensors detect the table and arm rises up depending on the height         of the table, the visible LEDs turn ON   
      - If the table is uneven, i.e. of two different heights from the arm, the arm goes to a higher/lower position continually polling         the ultrasound sesnsors, the LEDs are still ON  
      - Once the roomba is no longer under the table, the arm goes back in reset position and the LEDs are turned OFF   
        
2.**To stop everything**  
    
-- From your computer that is SSHed into RPi, hit Ctrl+c to keyboard interrupt  
-- If that doesn’t stop the Roomba from moving, physically lift the Roomba using the handles so that the wheels are off the ground, then    place it back on the ground   
-- Hit the Panic button 27 when running the ultrasound.py program to switch the system off and get everything back to initial position   
