# MT-V2X

This repository contains our preliminary work related to Mikrotik configuration to enable IEEE 802.11p vehicular connectivity.
A twin private repository is instead configuring Quectel AG15 devices for C-V2X (3GPP) vehicular connectivity.

Here you can find scripts to:
- define, create and associate a board to an IEEE 802.11p network;
- configure a GPS receiver to collect GPS data on board;
- analyze GPS signals to trigger CAM messages;
- create transmitting script, to disseminate in broadcast CAM messages, and/or receiving scripts;
- synchronize through GPS or NTP each node;
- process all the collected data in MATLAB to computer packet error rates, one-way latency, etc.

This research activity is under the supervision of Professor Carlo Augusto Grazia.


## Table of contents

1. [Project structure](#project-structure)
2. [Installation and startup](#installation-and-startup)
3. [General description](#general-description)


## Project structure
    /
    |--> /Elaborazione
    |     |--> /20210324: Log Tests Peak Hour for Quectel and Mikrotik.
    |     |--> /20210322: Log Tests Not Peak Hour for Quectel and Mikrotik.
    |     |--> calcoli.m: Mathlab File for data analysis
    |
    |--> /Guide
    |     |--> ACME_EXAMPLE.txt: examples of normal communications under C-V2X.
    |     |--> ACME_HELP.txt: Acme command for set some parameters for a normal communications.
    |     |--> GPS_MIKROTIK.pdf: How to config a GPS USB for Mikrotik with windows.
    |     |--> Pacchetti Lede: All packets installed on Mikrotik to support USB, gcc, ntpd.
    |     |--> Step per install-exec SDK-acme.txt: Steps to install SDK enviroment for programming Quectel AG15.
    |     |--> info_quectel.txt: How to start AG15, send a file to board, receive a file from board, and start a communication between TX and RX
    |
    |--> /Mikrotik
    |     |--> /Final
    |           |--> serial_TX.c: TX used for tests
    |           |--> serial_RX.c: RX used for tests
    |           |--> Tracking_TX.c: Traker RX used for tests
    |           |--> serial_RX.c: RX used for tests
    |           |--> Disturb_thread.c: Disturbator that simulate multiple boards (Used inputs: 10 20) 10ms and 20 thread
    |
    |-->/Quectel
    |    |--> /CAM
    |           |--> acme.c: Acme modify to send CAM messages
    |    |--> /Original_ACME
    |           |--> acme.c: virgin acme
    |--> readme.md
    |--> Tecnologie IEEE 80211p e C V2X a confronto verso le reti inter veicolari.pdf: Thesis of our work


## Installation and startup

### SDK installation for AG15:
For work correctly follow:

    /Guide/Step per install-exec SDK-acme.txt
    
This allows users to set correctly the architecture and the enviroment used for cross compiling the code.
Once the code is correcly compiled, the executable is send to the board by follow the guide /Guide/info_quectel
in section:

    "Come inviare un generico file via seriale con screen"
    
### Mikrotik:

The only thing you nedd to know is to install the right packet contained on guide:

    /Guide/Pacchetti Lede

Whit Mikrotik you don't need cross compilation,
so the steps are simpler: write a code, 
send or copy the C file into the board and compile whit gcc inside Mikrotik. 

## General description

