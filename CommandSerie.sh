#!/usr/bin/env python

import time
import serial
import struct

ser = serial.Serial(
        port = '/dev/ttyAMA0',
        baudrate = 19200,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1
        )

while 1:
        print("\nChangement d'adresse (A), commande (C) ou liste des commandes (L) ?")
        A_ou_C = raw_input()
        if(A_ou_C == 'L'):
                print("\nListe des commandes : ")
                print("0 : Demande de presence")
                print("20 : Allume le triac")
                print("21 : Eteint le triac")
                print("15 : Demande l'etat du triac \n")
                print("22 : Met le triac en variateur à 15%")
                print("23 : Met le triac en variateur à 25%")
                print("24 : Met le triac en variateur à 40%")
                print("25 : Met le triac en variateur à 50%")
                print("26 : Met le triac en variateur à 70%")
                print("27 : Met le triac en variateur à 85%")
                print("30 : Eteint les interrupteurs Flamingo avec l'adresse A")
                print("31 : Allume les interrupteurs Flamingo avec l'adresse A")
                print("32 : Eteint les interrupteurs Flamingo avec l'adresse B")
                print("33 : Allume les interrupteurs Flamingo avec l'adresse B")
                print("34 : Eteint les interrupteurs Flamingo avec l'adresse C")
                print("35 : Allume les interrupteurs Flamingo avec l'adresse C")
                print("36 : Eteint les interrupteurs Flamingo avec l'adresse D")
                print("37 : Allume les interrupteurs Flamingo avec l'adresse D")
                print("38 : Eteint tous les interrupteurs")
                print("39 : Allume tous les interrupteurs")
                print("Changement d'adresse (A) ou commande (C) ?")
                A_ou_C = raw_input()
        A_ou_C = ord(A_ou_C)
        print("Pour quelle adresse ?")
        Adresse = input()
        if(A_ou_C == 67):
                print("Pour quelle commande ?")
                Commande = input()
        else:
                Commande = Adresse
        Dollar = '$'
        Dollar = ord(Dollar)
        Envoi = (Dollar,A_ou_C,Adresse,Commande)
        string = ''
        for i in Envoi:
                string += struct.pack('!B',i)
        ser.write(string)
        rep = ser.readline()
        if(rep != ""):
                print("Reponse : " + rep)

