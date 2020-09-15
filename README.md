# README #

Repository per il progetto di IRS. Il lavoro prevede la riproduzioni di alcuni esperimenti facendo uso di controller behaviour based e ibridi.
Il pdf all'interno di questa cartella, fornisce tutte le informazioni a riguardo.

#BUILD  

ARGoS non permette ancora l'implimentazione di function loop in lua, dunque è necessario buildarle.
In ogni cartella (SCA e SPC), vanno eseguiti i seguenti passaggi:

mkdir build

cd buil

cmake ./src

make

argos3 -c nome.argos
