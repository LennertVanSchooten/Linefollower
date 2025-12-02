
# Instructable

Een instructable is een stappenplan - zonder verdere uitleg - hoe je vertrekkend van de bill of materials en gebruik makend van de technische tekeningen de robot kan nabouwen. Ook de nodige stappen om de microcontroller te compileren en te uploaden staan beschreven.  

### stap 0
Bestel de componenten uit de BOM, zoveel mogelijk van dezelfde leverancier (tinytronics).

De pcb, onder technische tekeningen -> elektronisch vindt je de elektronische schemas als de pcb file die je kan importeren in easyeada.

De pcb is ontworpen om de weerstanden, drukknop, schakelaar en led er op telaten zetten tijdens productie en is dus mee gerekend in de pcb kosten.
(dit moet je tijdens het bestellen aanduiden want dit wordt niet automatisch gedaan)

### stap 1
test de componenten uit a.d.h.v. de schemas en codes onder de tab code -> proof of concepts. 
deze zullen een goede werking demonstreren van de componenten.

DRV8833 wordt best eerst getest met led's.
Aangeraden om de sensor op een vaste hoogt te monteren om een duidelijk verschil te zien tussen zwart en wit.

### stap 2
Soldeer de female headerds en terminals op de pcb als de draden op de sensor, aangeraden om 0.8mm² stijvedraden te gebruiken (recup van sgg kabel).

### stap 3
Print het frame dat te vinden is onder technische tekeningen -> mechanisch. 
Hier zal je een stl bestand vinden als een solidworks tekening voor als je zelf aanpassingen wilt uitvoeren.
(spoiler stl is ook beschikbaar)

### stap 4
monteer alle componeten op het frame met M3 bouten en moeren. je moet er maar 5 van elk gebruiken (2 voor de sensor, 2 voor de batterijhouder en 1 voor de pcb).

### stap 5
Als de motoren en batterijhouder op het frame zijn bevestigd kan je de draden hiervan solderen.

LET GOED OP!!!!
Batterijhouder: de rode draden van de batterijhouder zullen wij zien als + en zwart als -. We solderen de rode draden op de pcb links van de schakelaar (aan de kant waar VBAT staat) en de zwarte draden rechts van de schakelaar (aan de kant waar GND staat). 
De batterijen zijn via de pcb in serie geschakeld dus we mogen ze extern NIET KRUISEN, ik heb de buitenste batterij aan de buitenste pads gesoldeerd en de batterij aan de binnen kant aan de binnenste pads.

Motoren: de motoren kunnen simpel gesoldeerd worden als ze geklemmed zitten in het frame. hoe dat deze aangesloten worden is de bovenste draden aan de buitenste terminal slots en de onderste aan de binneste slots van hun respectievelijke terminals (links-links & rechts-rechts).

### stap 6 
Upload de basis code dat te vinden is in code->basis. download alle 4 de bestanden en zet deze in dezelfde map. anders zal het programma niet werken.
dit programma bevat een basis werking van het EEPROM geheugen als normalistatie en interpolatie van de sensor en een P regelaar. 

commandes zijn terug tevinden in de gebruikersaanwijzing.

### stap 7
print de parcours. steek de batterijen in de houder (ROOD +, ZWART -) en zet de schakelaar naar binnen toe. 
als alles goed is verlopen zullen nu signalisatie leds van de arduino nano, hc-08 en motordrive branden.

### stap 8
zet de follower op een wit stuk van het parcour en calibreer wit, doe dit ook voor zwart.

Zet intitieel  power= 80, kp= 10 en diff op 0,4.
Nu ben je klaar om te rijden. 
Stuur het commande "run on" and watch the magic happen.

Je zal de instellingen langzaam moeten tweaken want de waarde zullen voor iedereen verschillen.

### stap 9
upload de final code, deze heeft ook een I & D regelaar.
De follower heeft nu de mogelijkheid om de lijn vlotter tevolgen.
