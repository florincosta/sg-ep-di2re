# SG-EP-DI2re
SubGigaHertz EndPoint with 2 DigitalInputs of reed type


## Power Consumption

  |      Phase       |    Time    |  Current  |
  |:-----------------|-----------:|----------:|
  |Read Inputs       |        50us|        5mA|
  |WakeUp Cycle      |       200ms|       20uA|
  |Transmission      |        15ms|      120mA|


  **Average consumption without transmission:**
  ( (200ms * 20uA) + (50us * 5mA) ) / (200ms + 50us) = 
  ( (4000 + 250) * ms * uA ) / 200.05ms = 21.24uA

  **Average consumption with transmission:**
  Transmisson events per day: 5
  ((24 * 60 * 60 * 1000)ms * 21.24uA) + (5 * 15 * 120mA) /  


## General Notes

### Low Power LSE

 1. REED Inputs can be pulled down only by shorting pins by hand.
 2. STM32 LSE oscillators are made to work with very low consumption, and are therefore very sensitive. See AN2867. You can observe them oscillating with the most common 1:10 scope probe (i.e. an extra 15pF or so) on the OUT pin only of course, the slightest extra load on IN pin killes the oscillations. Play with drive levels if you encounter problems **__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH)**

### Enabling semihosting(if required)
 
 1. add “-specs=rdimon.specs -lc -lrdimon” to (Project -> Properties -> C/C++ Build -> Settings -> MCU GCC Linker -> Linker flags)
 2. add extern void initialise_monitor_handles(void); above main
 3. add initialise_monitor_handles(); at the beggening og main function
 4. use printf, putc, puts to output messages via semihosting

**For printf don't forget to use \n to flush buffer to console!**

