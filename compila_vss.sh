#!/bin/sh

g++ Futebol.cpp Auxiliares.cpp Captura.cpp Controle.cpp Estrategia.cpp Posicoes.cpp Serial.cpp Simulador.cpp `pkg-config gtkmm-2.4 --libs --cflags libusb-1.0` -o vss.out
./vss.out
