#SG-EP-DI2re
SubGigaHertz EndPoint with 2 DigitalInputs of reed type

##Enabling semihosting    
1. add “-specs=rdimon.specs -lc -lrdimon” to (Project -> Properties -> C/C++ Build -> Settings -> MCU GCC Linker -> Linker flags)
2. add extern void initialise_monitor_handles(void); above main
3. add initialise_monitor_handles(); at the beggening og main function
4. use printf, putc, puts to output messages via semihosting
**For printf don't forget to use \n to flush buffer to console!** 

