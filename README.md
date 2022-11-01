# Solar-Tracker-Project

Powered by the STM32F103C8 board, this solar tracker uses an RTC clock to periodically shutdown once it has attained an optimal 
angle (panel perpendicular to sun rays). The RTC also has a calendar implementation which allows it to know the proper sleep 
duration during the night depending on the season.
