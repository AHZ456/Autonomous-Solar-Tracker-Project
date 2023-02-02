# Solar-Tracker-Project

Powered by the STM32F103C8 board, this solar tracker uses an RTC clock to periodically shutdown once it has attained an optimal 
angle (panel perpendicular to sun rays). A calendar has been implemented using The microcontroller's RTC which allows it to deduce the proper sleep 
duration as to trigger the wake up event at sunrise depending on the season.
