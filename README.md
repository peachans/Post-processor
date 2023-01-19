# Post-processor
Convert G-code to Robotic code
This is my Senior project as Mechanical Engineering student. This project was developed under Biomechanical Engineering lab at Chulalong university lead by Asst. Prof. Pairat Tangpornprasert and Asst. Prof. Chanyaphan Virulsri. This lab is the only one to develop and produce prosthetic femurs in Thailand. The current method used CNC machine to form the hip stem. However, using CNC can be very slow in the initial phase of the process because the metal was forged in the near-net-shape which is very rough and thick. To solve this problem, we need to explore and research new method for grinding in the initial phase before using CNC to smooth the hip-stem surface. This project was inspired by hipstem production in the oversea company which they using robotics arm along with grinding sand belt machine to grind the hipstem surface. Unfortunately, there is not much knowledge about this method in Thailand. So the goal of this project is establish know how on grind prothetic femur using robot arm.
## (Very) Breif explantion of how this project is done.
We generated the grinding path for hipstem by using CAM (Matercam) and generated the G-code. In this case, we used Mazak integrex machine which is a 5 Degree of freedom CNC machine. However, the robot arm that we used is Fanuc 16i/B which is 6 degree of freedom machine. In addition, for the CNC method, the cutting tool is moving around the hipstem, while for the robotic arm process, the robot will hold the hipstem and grind along the cutting tool. So we need to transform the position of the cutting tool in G-code into the moving hipstem. We used homogenoeus transformation to transform the position inversely. We have developed the post-processor to transfrom G-code to robotic code.

The overall diagram of this project are shown below (to understand this you will need a strong understading in Industrial robot).
![Screenshot 2566-01-19 at 18 18 18](https://user-images.githubusercontent.com/106228102/213429143-24b293ab-97de-4ce0-89b0-120d597a84f4.png)
The right picture is cutting tool in the CNC. The middle one is inversly tranform position from Gcode to robot code. The last one is to reference the end of the robot arm to the robot base.

Note that: The coordinating system of integrex machine is a mix between euler angle and fix axis 𝑅𝑜𝑡𝑍′(B) ∗ 𝑇𝑟𝑎𝑛𝑠𝑙𝑋′𝑌′𝑍′(𝑋, 𝑌, 𝑍) ∗ 𝑅𝑜𝑡𝑌′(B'). While in fanuc, it used fix angle axis XYZ convention.

link: https://youtu.be/N91EsE4KeVY
