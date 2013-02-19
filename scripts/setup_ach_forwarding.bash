sudo service tech ach

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

sudo achd pull -r 192.168.1.245 "hubo-state" &

