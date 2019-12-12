sudo fallocate -l 1G /swapfile1G
ls -lh /swapfile1G
sudo chmod 600 /swapfile1G
ls -lh /swapfile1G
sudo mkswap /swapfile1G
sudo swapon /swapfile1G
free -h
