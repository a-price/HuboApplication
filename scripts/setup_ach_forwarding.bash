sudo service hubo-motion ach
sudo ach -1 -C "hubo-manip" -m 10 -n 3000 -o 666

sudo achd push -d 192.168.1.245 "hubo-manip"
sudo achd pull -r 192.168.1.245 "hubo-state"
: <<'END'
sudo achd push -r 192.168.1.245 "hubo-RL-control" &
sudo achd push -r 192.168.1.245 "hubo-LL-control" &
sudo achd push -r 192.168.1.245 "hubo-RA-control" &
sudo achd push -r 192.168.1.245 "hubo-LA-control" &
sudo achd push -r 192.168.1.245 "hubo-AUX-control" &

sudo achd push -r 192.168.1.245 "hubo-RF-control" &
sudo achd push -r 192.168.1.245 "hubo-LF-control" &

sudo achd push -r 192.168.1.245 "hubo-ref" &
sudo achd push -r 192.168.1.245 "hubo-board-cmd" &
sudo achd push -r 192.168.1.245 "ctrl-d-state" &
END
