# start/stop interrupt proof of concept
Dit is een simpele minimale code die aantoont hoe da een toggle functie kan geprogrammeerd worden met een knop en led. 
Uitgangen zijn gekozen a.d.h.v. het elektronisch schema.

De knop wordt uitgelezen en zal een bool (run) op initieel op 1 zetten bij het 1ste gebruik.
Bij de 2de druk, als run = 1, zal diezelfde knop run op 0 zetten waardoor de led ook zal doven.
