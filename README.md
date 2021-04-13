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
