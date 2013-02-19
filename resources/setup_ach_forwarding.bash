sudo achd push -r 192.168.1.245 "hubo-RL-control" &
sudo achd push -r 192.168.1.245 "hubo-LL-control" &
sudo achd push -r 192.168.1.245 "hubo-RA-control" &
sudo achd push -r 192.168.1.245 "hubo-LA-control" &
sudo achd push -r 192.168.1.245 "hubo-AUX-control" &

sudo achd push -r 192.168.1.245 "hubo-ref" &
sudo achd push -r 192.168.1.245 "hubo-param" &
sudo achd push -r 192.168.1.245 "hubo-board-cmd" &

sudo achd pull -r 192.168.1.245 "hubo-state" &
