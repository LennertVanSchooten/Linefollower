# H-Bridge proof of concept

Welkom in de readme van het H-brug gedeelte van de linefollower.

**Benodigdheden**  
-  DRV8833 microchip
-  Arduino Nano R3
-  	Aslong JGA12-N20-30 DC-Transmissiemotor 6V - 500RPM

**Stappenplan**  
-  De DRV8833 microchip heeft op het breakoutbord 4 ingangen, 4 uitgangen, VCC, GND, EEP & ULT.

-  De ingangen en uitgangen spreken opzich, zeer belangrijk bij de ingangen is dat de pwm's op de zelfde clock snelheid zitten. PWM 3, 5, 6 & 11 hebben een 8bit clocksnelheid. PWM 9 & 10 hebben een 16bit clocksnelheid. ALs 2 soorten clocksnelheden worden gebruikt zullen de motoren een verschillende snelheid hebben.

-  De EEP pin wordt aan de VCC (5V) van de arduino gekoppeld, dit moet zeker verbonden worden anders zal de H-brug niet werken. Als de EEP geen voeding krijgt zal de H-brug in sleep modus staan waardoor de OUT's niet zullen schakelen.

-  De Ult pin wordt met een pull-up weerstand geschakeld, deze dient als fout detectie. Als de H-brug normaal werkt zal deze een hoog signaal hebben waardoor de arduino een laag signaal leest. Als er een interne fout in de H-brug is zal de uitgang laag zijn waardoor de arduino een hoog signaal zal lezen. Dit zullen we a.d.h.v. een HC-05 door sturen naar een gsm.
