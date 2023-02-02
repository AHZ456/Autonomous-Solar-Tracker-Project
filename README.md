# Solar-Tracker-Project


This project consists of a solar tracker which has its actuators and processing unit powered entirely by solar energy making it a fully autonomous tracker.  Powered by the STM32F103C8 board, this solar tracker makes use of an integrated RTC clock to periodically initiate a shutdown event: the motors responsible for orienting the panel are turned off and the microcontroller enters "STAND BY". This event is triggered once the panel reaches an optimal angle with regards to sunlight which increases the tracker’s energy efficiency. A calendar has been implemented using the microcontroller's RTC clock which allows it to deduce the proper sleep duration for the season in order for the system to wake up consistently at sunrise. 
