# Solar-Tracker-Project


This project consists of a solar tracker which has its actuators and processing unit powered entirely by solar energy making it a fully autonomous tracker.  Powered by the STM32F103C8 board, this tracker makes use of an integrated RTC clock to initiate a periodic shutdown event: the motors responsible for orienting the panel are turned off and the microcontroller enters "STANDBY" mode. This event is triggered once the panel reaches an optimal angle with regards to sunlight which increases the tracker’s energy efficiency. A calendar has been implemented using the microcontroller's RTC clock. This allows our microcontroller to deduce the appropriate nocturnal shutdown period for the current season in order for the system to resume operation at sunrise consistently throughout the year. 

## Demonstration Video

[Linkedin Post](https://www.linkedin.com/posts/azinedine_i-proudly-present-to-you-an-autonomous-solar-activity-6820499945582280704-gmtM?utm_source=share&utm_medium=member_desktop)
