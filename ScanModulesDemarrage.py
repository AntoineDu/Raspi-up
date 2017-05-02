#!/usr/bin/python
import MySQLdb
import time
import serial
import struct

# Initialisation de la communication série :
ser = serial.Serial(
    port = '/dev/ttyAMA0',
    baudrate = 19200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 0.5
    )

# Connexion à la base de données :
db = MySQLdb.connect(host="localhost", user = "root", passwd = "powerline", db = "raspiup")

cur = db.cursor()

cur.execute("SELECT * FROM infomodules")

Dollar = 36 
C = 67
Commande = 15
Compteur = 1

# Boucle sur le nombre de ligne de la base de donnée :
for row in cur.fetchall():
    Adresse =  row[1]
    Envoi = (Dollar,C,Adresse,Commande)
    string = ''
    for i in Envoi:
        string += struct.pack('!B',i)
    ser.write(string)
    Reponse = ser.readline()
    print Reponse
    if Reponse == "0" or  Reponse == "1":
        cur.execute("UPDATE infomodules SET ALIVE = 1, STATE = %s WHERE ID = %s",(Reponse, Compteur))   
        db.commit()
    else:    
        cur.execute("UPDATE infomodules SET ALIVE = 0, STATE = 0 WHERE ID = %s", Compteur)
        db.commit()
    Compteur = Compteur + 1

db.close()