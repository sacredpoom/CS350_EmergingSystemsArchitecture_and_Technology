*Summarize the project and what problem it was solving.*

The first project UART2ECHO demonstrates use of serial communication via UART which is echoed in the terminal. After gaining familiarity with different peripherals including PWM, UART, and I2C, the final project involved programming a thermostat. The thermostat reads the ambient temperature of the room and compares to a setpoint controlled via GPIO buttons by user triggering an LED if ambient is below the setpoint modeling powering of a heating element. This project gives experience working with embedded systems with wireless communication capabilities for future work wit Internet of Things technologies. 

*What did you do particularly well?*

The state-machine diagram created models different states of the thermostat and how it transitions between them. The code is clean, organized, and thoroughly commented successfully modeling required system behaviour. 

*Where could you improve?*

Code can always be made more secure and full wireless capability has not yet been implemented. System should implement system reliability features such as a watchdog timer and more error handling measures. 

*What tools and/or resources are you adding to your support network?*

This project was the first time working with embedded C and required heavy consultation of the board documentation. Learning and adapting to workign with an unfamiliar technology will be a frequently used skill in future jobs. 

*What skills from this project will be particularly transferable to other projects and/or course work?*

Troubleshooting and debugging code will be an applicable skill no matter which field I go into. This project allowed me to gain experience taking project requirements and translating into software designs with diagrams and implementing in code to produce a functioning finished deliverable.

*How did you make this project maintainable, readable, and adaptable?*

This project is implemented through version control (git) to allow for future updates and commits as well as the option to revert to a previous build. The code is thoroughly commented and diagrams are included to model system logic. 
