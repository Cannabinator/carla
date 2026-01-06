### goals of this program
    - repeatable scenario with vehicle2vehicle communication


### NOTES 

- ist richtung, beschleunigung ect als sensordatensatz aktuell aktiviert ? 
- ist autopilot aktiviert ? 
- 

## how to implement better v2v communication and sensor data handling that fit my needs 
    - 2 Hertz tick rate
    - look up existing v2v frameworks or solutions for carla and adapt the implementations if suiteable
    - if the threshold for communication is between leading car and other actors is reached i want them to share data in both directions
    - look up what data is most crucial to transfer for v2v scenarios
    - i want a ONE LINE per tick only output in the bash with basic information about leading vehicle status
    - look up existing v2v frameworks for carla and adapt the implementations if suiteable
    - display the v2v data information in the frontend aswell
    - create a API

## Solution: 
 
```
 class V2VNetworkEnhanced:
    """
    Enhanced V2V Network Manager with BSM protocol support.
    
    Features:
    - 2 Hz tick rate for V2V updates
    - Bidirectional data sharing
    - BSM (Basic Safety Message) protocol
    - Cooperative perception
    - Threat assessment
    - Message prioritization
    """
``` 

in der klasse `messages.py` sind verschiedene V2V Message Standards und funktionen nach standards definiert BSM (Basic Safety Message) Implementation Based on SAE J2735 and ETSI ITS-G5 standard. Desweiteren befinden sich in der klasse noch collision detections scripts ect. 


