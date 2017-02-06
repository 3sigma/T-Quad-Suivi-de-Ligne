#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de contrôle du robot T-Quad avec 2 roues classiques et une boule
# omnidirectionnelle, disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.1.2 - 30/01/2017
##################################################################################

# Importe les fonctions Arduino pour Python
from pyduino import *

# Imports Généraux
import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

# Nom de l'hostname (utilisé ensuite pour savoir sur quel système
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega(hostname = hostname)

# Moteurs
Nmoy = 1

omegaArriereDroit = 0.
codeurArriereDroitDeltaPos = 0
codeurArriereDroitDeltaPosPrec = 0

omegaArriereGauche = 0.
codeurArriereGaucheDeltaPos = 0
codeurArriereGaucheDeltaPosPrec = 0

# Variables nécessaires à la commande des moteurs
# Consignes de tension
vref = 0.
vrefArriereDroit = 0.
vrefArriereGauche = 0.

# Variables pour le suivi de ligne
L1 = 0.
L2 = 0.
L3 = 0.
seuil = 2.


# Tension effectivement appliquée
commandeArriereDroit = 0.
commandeArriereGauche = 0.

# Saturations
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur

# Timeout de réception des données
timeout = 2
timeLastReceived = 0
timedOut = False

T0 = time.time()
dt = 0.01
i = 0
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

idecimLectureTension = 0
decimLectureTension = 6000
decimErreurLectureTension = 100

idecimDistance = 0
decimDistance = 20

# Mesure de la tension de la batterie
# On la contraint à être supérieure à 7V, pour éviter une division par
# zéro en cas de problème quelconque
lectureTensionOK = False
tensionAlim = 7.4
while not lectureTensionOK:
    try:
        tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
        lectureTensionOK = True
    except:
        print("Erreur lecture tension")


#--- setup --- 
def setup():
    CommandeMoteurs(0, 0, 0, 0)
    

    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i, T0
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --

def CalculVitesse():
    global omegaArriereDroit, omegaArriereGauche, timeLastReceived, timeout, timedOut, \
        tdebut, codeurArriereDroitDeltaPos, codeurArriereGaucheDeltaPos, \
        commandeArriereDroit, commandeArriereGauche, \
        vrefArriereDroit, vrefArriereGauche, \
        codeurArriereDroitDeltaPosPrec, codeurArriereGaucheDeltaPosPrec, \
        idecimLectureTension, decimLectureTension, decimErreurLectureTension, tensionAlim, \
        L1, L2, L3, seuil
    
    tdebut = time.time()
        
    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    try:
        codeursArriereDeltaPos = mega.read_codeursArriereDeltaPos()
        codeurArriereDroitDeltaPos = codeursArriereDeltaPos[0]
        codeurArriereGaucheDeltaPos = codeursArriereDeltaPos[1]
        
        # Suppression de mesures aberrantes
        if (abs(codeurArriereDroitDeltaPos - codeurArriereDroitDeltaPosPrec) > 20) or (abs(codeurArriereGaucheDeltaPos - codeurArriereGaucheDeltaPosPrec) > 20):
            codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
            codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec

        codeurArriereDroitDeltaPosPrec = codeurArriereDroitDeltaPos
        codeurArriereGaucheDeltaPosPrec = codeurArriereGaucheDeltaPos
    except:
        #print "Error getting data"
        codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
        codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec

    omegaArriereDroit = -2 * ((2 * 3.141592 * codeurArriereDroitDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    omegaArriereGauche = 2 * ((2 * 3.141592 * codeurArriereGaucheDeltaPos) / 1200) / (Nmoy * dt)  # en rad/s
    
    # Lecture des capteurs de suivi de ligne
    try:
        # Passage des millivolts aux volts
        L1 = mega.line_read(1) / 1000.
        L2 = mega.line_read(2) / 1000.
        L3 = mega.line_read(3) / 1000.
    except:
        print("Erreur lecture capteurs ligne")


    # On compare par rapport à un seuil pour savoir si le capteur voit la ligne ou non
    surLigne1 = False
    surLigne2 = False
    surLigne3 = False
    if L1 < seuil:
        surLigne1 = True
    if L2 < seuil:
        surLigne2 = True
    if L3 < seuil:
        surLigne3 = True
    
    # Si le robot est centré sur la ligne, on va tout droit
    if ((surLigne1 == False) and (surLigne2 == True) and (surLigne3 == False)) or ((surLigne1 == True) and (surLigne2 == True) and (surLigne3 == True)):
        vrefArriereDroit = -vref # Tension négative pour faire tourner positivement ce moteur
        vrefArriereGauche = vref
    # Si seul le capteur de gauche est sur la ligne on tourne à gauche fort
    elif (surLigne1 == True) and (surLigne2 == False) and (surLigne3 == False):
        vrefArriereDroit = -vref
        vrefArriereGauche = -vref * 2 / 3
    # Si seul le capteur de droite est sur la ligne on tourne à droite fort
    elif (surLigne1 == False) and (surLigne2 == False) and (surLigne3 == True):
        vrefArriereDroit = vref * 2 / 3
        vrefArriereGauche = vref
    # Si les deux capteurs de gauche sont sur la ligne on tourne à gauche normalement
    elif (surLigne1 == True) and (surLigne2 == True) and (surLigne3 == False):
        vrefArriereDroit = -vref
        vrefArriereGauche = -vref / 2
    # Si les deux capteurs de droite sont sur la ligne on tourne à droite normalement
    elif (surLigne1 == False) and (surLigne2 == True) and (surLigne3 == True):
        vrefArriereDroit = vref / 2
        vrefArriereGauche = vref
        
    CommandeMoteurs(vrefArriereDroit, vrefArriereGauche, 0, 0)

    # Lecture de la tension d'alimentation
    if idecimLectureTension >= decimLectureTension:
        try:
            tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
            idecimLectureTension = 0
        except:
            # On recommence la lecture dans decimErreurLectureTension * dt
            idecimLectureTension = idecimLectureTension - decimErreurLectureTension
            #print("Erreur lecture tension dans Loop")
    else:
        idecimLectureTension = idecimLectureTension + 1

        
    #print time.time() - tdebut
        
        
def CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    global tensionAlim
    
    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tensionArriereDroit = commandeArriereDroit
    tensionArriereGauche = commandeArriereGauche
    tensionAvantDroit = commandeAvantDroit
    tensionAvantGauche = commandeAvantGauche

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int_ArriereDroit = int(255 * tensionArriereDroit / tensionAlim)
    tension_int_ArriereGauche = int(255 * tensionArriereGauche / tensionAlim)
    tension_int_AvantDroit = int(255 * tensionAvantDroit / tensionAlim)
    tension_int_AvantGauche = int(255 * tensionAvantGauche / tensionAlim)

    # Saturation par sécurité
    if (tension_int_ArriereDroit > 255):
        tension_int_ArriereDroit = 255

    if (tension_int_ArriereDroit < -255):
        tension_int_ArriereDroit = -255

    if (tension_int_ArriereGauche > 255):
        tension_int_ArriereGauche = 255

    if (tension_int_ArriereGauche < -255):
        tension_int_ArriereGauche = -255

    if (tension_int_AvantDroit > 255):
        tension_int_AvantDroit = 255

    if (tension_int_AvantDroit < -255):
        tension_int_AvantDroit = -255

    if (tension_int_AvantGauche > 255):
        tension_int_AvantGauche = 255

    if (tension_int_AvantGauche < -255):
        tension_int_AvantGauche = -255

    # Commande PWM
    try:
        mega.moteursArriere(tension_int_ArriereDroit, tension_int_ArriereGauche)
        mega.moteursAvant(tension_int_AvantDroit, tension_int_AvantGauche)
        mega.moteursCRC(tension_int_ArriereDroit + tension_int_ArriereGauche, tension_int_AvantDroit + tension_int_AvantGauche)
    except:
        pass
        #print "Erreur moteurs"

    
def emitData():
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    delay(5000)
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 20)
        self.callback.start()
    

    def on_message(self, message):
        global vref, seuil, vrefArriereDroit, vrefArriereGauche, timeLastReceived, timedOut
        
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
        
        if jsonMessage.get('vref') != None:
            vref = float(jsonMessage.get('vref'))
        if jsonMessage.get('seuil') != None:
            seuil = float(jsonMessage.get('seuil'))
        
        
        # Application de la consigne sur les moteurs arrière
        vrefArriereDroit = vref
        vrefArriereGauche = vref
            
        if not socketOK:
            vrefArriereDroit = 0.
            vrefArriereGauche = 0.


    def on_close(self):
        global socketOK, vrefArriereDroit, vrefArriereGauche
        print 'connection closed...'
        socketOK = False
        vref = 0.
        vrefArriereDroit = 0.
        vrefArriereGauche = 0.

    def sendToSocket(self):
        global socketOK, omegaArriereDroit, omegaArriereGauche, L1, L2, L3, seuil
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), \
                                'Consigne':("%.2f" % vref), \
                                'omegaArriereDroit':("%.2f" % omegaArriereDroit), \
                                'omegaArriereGauche':("%.2f" % omegaArriereGauche), \
                                'L1':("%.1f" % L1), \
                                'L2':("%.1f" % L2), \
                                'L3':("%.1f" % L3), \
                                'seuil_lu':("%.1f" % seuil), \
                                'Raw':("%.2f" % tcourant) \
                                + "," + ("%.2f" % vref) \
                                + "," + ("%.2f" % omegaArriereDroit) \
                                + "," + ("%.2f" % omegaArriereGauche) \
                                + "," + ("%.1f" % L1) \
                                + "," + ("%.1f" % L2) \
                                + "," + ("%.1f" % L3) \
                                })
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    global vrefArriereDroit, vrefArriereGauche
    print 'Sortie du programme'
    CommandeMoteurs(0, 0, 0, 0)
    vref = 0.
    vrefArriereDroit = 0.
    vrefArriereGauche = 0.
    CommandeMoteurs(0, 0, 0, 0)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
    setup() # appelle la fonction setup
    print "Setup done."
    
    th = threading.Thread(None, emitData, None, (), {})
    th.daemon = True
    th.start()
    
    print "Starting Tornado."
    try:
        print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
    except:
        pass
        
    try:
        print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
    except:
        pass
    socketOK = False
    startTornado()


