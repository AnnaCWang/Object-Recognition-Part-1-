Object Recognition- Computer Vision


Requirements:

Run on 64 bit Ubuntu 14.04 LTS Linux Machine

- Will need bags of data to run nodes through
- Will need to download UT Building Wide Intelligence package located on github

	https://github.com/utexas-bwi/bwi



Summary:

Image_process_example in computer_vision src folder:
Takes bag and processes data. Node will quickly process, identify, and point to the neon pink object in the constantly updating images. Node will advertise constant Float32 messages to move_base_client based on position of object 

Move_base_client in src folder:
Given direction of where detected object is in image_process_example, movebaseclient will move robot to given goal based on directions relative to wheels of bot
