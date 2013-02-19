export FASTRAK_CHAN=fastrak
sudo ach -1 -C $FASTRAK_CHAN -m 10 -n 3000 -o 666
sudo achd pull -r 192.168.1.96 $FASTRAK_CHAN
