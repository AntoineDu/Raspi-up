# Raspi-up

La Raspi-UP est un module domotique pensé et fabriqué au sein du SEMi (Service d'Electronique et de Microélectronique).

Les modules utilisent la communication par courant porteur pour communiquer entre eux, c'est-à-dire que les données sont transférées directement sur le réseau électrique de la maison.
Le protocole de communication par courant porteur est l'UPB (Universal Powerline Bus). 
Ce protocole est basé sur l'envoi d'impulsion sur le réseau à un moment bien précis sur chaque alternance de la sinusoïde.
Son avantage est qu'il permet à chaque module d'être émetteur ou récepteur, il est libre de droit d'utilisation et son coût de fabrication est relativement correct.

Chaque module est capable d'allumer/éteindre une charge branchée dessus, ainsi que d'en faire varier l'intensité.

Il y a nécessité d'imposer un des modules comme "maître" pour qu'ils puissent communiquer. Ceci devant être fait en y branchant une Raspberry Pi dessus.
Cette Raspberry est en fait un microordinateur hébergeant un site web où l'utilisateur peut choisir les commandes à envoyées.

Ces commandes sont donc transférées de la Raspberry au module "maître" qui fait ensuite transiter, vers le(s) module(s) "esclave(s)" ciblés, les données sous formes d'impulsions sur le réseau éléctrique.

En plus de la communication par courant porteur, il est doté d'une communication sans fil en 433 MHz permettant de contrôler des modules interrupteurs, que l'on peut acheter en magasin, qui fonctionnent sur la même bande fréquence.